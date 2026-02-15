"""Unit tests for error handling: all exception handlers return consistent JSON."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from backend.models.errors import (
    ChatBackendError,
    ValidationError,
    RateLimitError,
    SessionNotFoundError,
    RetrievalError,
    GenerationError,
    chat_backend_error_handler,
    generic_error_handler,
    ErrorResponse,
)


class TestExceptionHandlers:
    """Test that all exception handlers return consistent JSON without stack traces."""

    @pytest.mark.asyncio
    async def test_validation_error_returns_400(self):
        request = MagicMock()
        exc = ValidationError("Invalid input")

        response = await chat_backend_error_handler(request, exc)

        assert response.status_code == 400
        body = response.body.decode()
        assert '"code"' in body
        assert '"validation_error"' in body
        assert "Traceback" not in body

    @pytest.mark.asyncio
    async def test_rate_limit_error_returns_429_with_retry_after(self):
        request = MagicMock()
        exc = RateLimitError(retry_after=45)

        response = await chat_backend_error_handler(request, exc)

        assert response.status_code == 429
        assert response.headers.get("Retry-After") == "45"
        body = response.body.decode()
        assert '"rate_limit_exceeded"' in body
        assert "45 seconds" in body
        assert "Traceback" not in body

    @pytest.mark.asyncio
    async def test_session_not_found_returns_404(self):
        request = MagicMock()
        exc = SessionNotFoundError("Session abc not found")

        response = await chat_backend_error_handler(request, exc)

        assert response.status_code == 404
        body = response.body.decode()
        assert '"session_not_found"' in body
        assert "Traceback" not in body

    @pytest.mark.asyncio
    async def test_retrieval_error_returns_503(self):
        request = MagicMock()
        exc = RetrievalError()

        response = await chat_backend_error_handler(request, exc)

        assert response.status_code == 503
        body = response.body.decode()
        assert '"retrieval_failed"' in body
        assert "Traceback" not in body

    @pytest.mark.asyncio
    async def test_generation_error_returns_503(self):
        request = MagicMock()
        exc = GenerationError()

        response = await chat_backend_error_handler(request, exc)

        assert response.status_code == 503
        body = response.body.decode()
        assert '"generation_failed"' in body
        assert "Traceback" not in body

    @pytest.mark.asyncio
    async def test_generic_error_returns_500_no_stack_trace(self):
        request = MagicMock()
        exc = RuntimeError("Something unexpected happened")

        response = await generic_error_handler(request, exc)

        assert response.status_code == 500
        body = response.body.decode()
        assert '"internal_error"' in body
        assert "Something unexpected" not in body  # No details leaked
        assert "Traceback" not in body

    @pytest.mark.asyncio
    async def test_all_errors_have_consistent_format(self):
        """Every error response has the same {error: {code, message}} shape."""
        request = MagicMock()

        errors = [
            ValidationError("bad input"),
            RateLimitError(retry_after=10),
            SessionNotFoundError("not found"),
            RetrievalError(),
            GenerationError(),
        ]

        for exc in errors:
            response = await chat_backend_error_handler(request, exc)
            import json
            data = json.loads(response.body.decode())
            assert "error" in data
            assert "code" in data["error"]
            assert "message" in data["error"]
