"""FastAPI application factory for the RoboTutor chat backend."""

from __future__ import annotations

import logging
import time
from contextlib import asynccontextmanager

from starlette.types import ASGIApp, Receive, Scope, Send

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from google import genai

from backend.config import load_backend_settings
from backend.db.pool import close_pool, create_pool
from backend.models.errors import (
    ChatBackendError,
    chat_backend_error_handler,
    generic_error_handler,
)
from backend.routers import auth_migration, chat, health, profile, sessions
from backend.services.rate_limiter import SlidingWindowRateLimiter

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage app startup and shutdown."""
    settings = load_backend_settings()

    # Configure logging
    logging.basicConfig(level=getattr(logging, settings.log_level.upper(), logging.INFO))

    # Create DB pool
    if settings.database_url:
        app.state.db_pool = await create_pool(settings.database_url)
        logger.info("Database pool created")
    else:
        app.state.db_pool = None
        logger.warning("DATABASE_URL not set — running without database")

    # Create Gemini client
    if settings.gemini_api_key:
        app.state.gemini_client = genai.Client(api_key=settings.gemini_api_key)
        logger.info("Gemini client initialized")
    else:
        app.state.gemini_client = None
        logger.warning("GEMINI_API_KEY not set — LLM features disabled")

    # Create rate limiter
    app.state.rate_limiter = SlidingWindowRateLimiter()
    app.state.settings = settings

    yield

    # Shutdown
    if app.state.db_pool is not None:
        await close_pool(app.state.db_pool)
        logger.info("Database pool closed")


class RequestLoggingMiddleware:
    """Pure ASGI middleware for request logging that doesn't buffer streams."""

    def __init__(self, app: ASGIApp) -> None:
        self.app = app

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        path = scope.get("path", "")
        method = scope.get("method", "")
        start = time.monotonic()
        status_code = 0

        async def send_wrapper(message):
            nonlocal status_code
            if message["type"] == "http.response.start":
                status_code = message.get("status", 0)
            await send(message)

        await self.app(scope, receive, send_wrapper)

        if path.startswith("/api/v1/"):
            duration_ms = int((time.monotonic() - start) * 1000)
            logger.info(
                "request path=%s method=%s status=%d latency_ms=%d",
                path,
                method,
                status_code,
                duration_ms,
            )


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    settings = load_backend_settings()

    app = FastAPI(
        title="RoboTutor Chat API",
        version="1.0.0",
        description="FastAPI backend for the Physical AI textbook chatbot",
        lifespan=lifespan,
    )

    # Request logging middleware — pure ASGI to avoid buffering SSE streams
    app.add_middleware(RequestLoggingMiddleware)

    # CORS — include Better-Auth service origin alongside configured origins
    cors_origins = list(settings.cors_origins)
    if settings.better_auth_url and settings.better_auth_url not in cors_origins:
        cors_origins.append(settings.better_auth_url)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PATCH", "OPTIONS"],
        allow_headers=["*"],
        expose_headers=["Retry-After"],
    )

    # Exception handlers
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.add_exception_handler(Exception, generic_error_handler)

    # Routers
    app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
    app.include_router(sessions.router, prefix="/api/v1", tags=["sessions"])
    app.include_router(profile.router, prefix="/api/v1", tags=["profile"])
    app.include_router(auth_migration.router, prefix="/api/v1", tags=["auth"])
    app.include_router(health.router, prefix="/api/v1", tags=["operations"])

    return app


app = create_app()
