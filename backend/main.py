"""FastAPI application factory for the RoboTutor chat backend."""

from __future__ import annotations

import logging
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.base import BaseHTTPMiddleware
from google import genai

from backend.config import load_backend_settings
from backend.db.pool import close_pool, create_pool
from backend.models.errors import (
    ChatBackendError,
    chat_backend_error_handler,
    generic_error_handler,
)
from backend.routers import chat, health, sessions
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


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    settings = load_backend_settings()

    app = FastAPI(
        title="RoboTutor Chat API",
        version="1.0.0",
        description="FastAPI backend for the Physical AI textbook chatbot",
        lifespan=lifespan,
    )

    # Request logging middleware (excludes message content for privacy)
    @app.middleware("http")
    async def log_requests(request: Request, call_next):
        start = time.monotonic()
        response = await call_next(request)
        duration_ms = int((time.monotonic() - start) * 1000)
        if request.url.path.startswith("/api/v1/"):
            logger.info(
                "request path=%s method=%s status=%d latency_ms=%d",
                request.url.path,
                request.method,
                response.status_code,
                duration_ms,
            )
        return response

    # CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins,
        allow_credentials=True,
        allow_methods=["GET", "POST", "OPTIONS"],
        allow_headers=["*"],
        expose_headers=["Retry-After"],
    )

    # Exception handlers
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.add_exception_handler(Exception, generic_error_handler)

    # Routers
    app.include_router(chat.router, prefix="/api/v1", tags=["chat"])
    app.include_router(sessions.router, prefix="/api/v1", tags=["sessions"])
    app.include_router(health.router, prefix="/api/v1", tags=["operations"])

    return app


app = create_app()
