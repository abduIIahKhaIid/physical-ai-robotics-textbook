"""Error response models and FastAPI exception handlers."""

from __future__ import annotations

import logging

from fastapi import Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

logger = logging.getLogger(__name__)


class ErrorDetail(BaseModel):
    """Error detail within an error response."""

    code: str
    message: str


class ErrorResponse(BaseModel):
    """Standard error response format."""

    error: ErrorDetail


class ChatBackendError(Exception):
    """Base exception for chat backend errors."""

    status_code: int = 500
    error_code: str = "internal_error"
    message: str = "An internal error occurred"

    def __init__(self, message: str | None = None):
        self.message = message or self.__class__.message
        super().__init__(self.message)


class ValidationError(ChatBackendError):
    status_code = 400
    error_code = "validation_error"
    message = "Invalid request"


class RateLimitError(ChatBackendError):
    status_code = 429
    error_code = "rate_limit_exceeded"
    message = "Too many requests"

    def __init__(self, retry_after: int, message: str | None = None):
        self.retry_after = retry_after
        super().__init__(
            message
            or f"You've sent too many messages. Please try again in {retry_after} seconds."
        )


class SessionNotFoundError(ChatBackendError):
    status_code = 404
    error_code = "session_not_found"
    message = "Session not found"


class RetrievalError(ChatBackendError):
    status_code = 503
    error_code = "retrieval_failed"
    message = "Search is temporarily unavailable"


class GenerationError(ChatBackendError):
    status_code = 503
    error_code = "generation_failed"
    message = "Response generation failed"


class ProfileNotFoundError(ChatBackendError):
    status_code = 404
    error_code = "profile_not_found"
    message = "Profile not found"


async def chat_backend_error_handler(
    request: Request, exc: ChatBackendError
) -> JSONResponse:
    """Handle ChatBackendError exceptions with consistent JSON format."""
    headers = {}
    if isinstance(exc, RateLimitError):
        headers["Retry-After"] = str(exc.retry_after)

    logger.warning(f"ChatBackendError: {exc.error_code} - {exc.message}")

    return JSONResponse(
        status_code=exc.status_code,
        content=ErrorResponse(
            error=ErrorDetail(code=exc.error_code, message=exc.message)
        ).model_dump(),
        headers=headers,
    )


async def generic_error_handler(request: Request, exc: Exception) -> JSONResponse:
    """Catch-all handler that never exposes stack traces."""
    logger.exception("Unhandled exception")
    return JSONResponse(
        status_code=500,
        content=ErrorResponse(
            error=ErrorDetail(
                code="internal_error",
                message="An internal error occurred. Please try again later.",
            )
        ).model_dump(),
    )
