"""Integration tests for the health check endpoint."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, patch

import pytest
import httpx

from backend.config import BackendSettings
from backend.services.rate_limiter import SlidingWindowRateLimiter


def _make_app(pool_healthy=True, qdrant_healthy=True):
    """Create test app with configurable health states."""
    settings = BackendSettings(
        database_url="postgresql://test",
        gemini_api_key="test-key",
    )

    mock_pool = AsyncMock()
    if pool_healthy:
        mock_pool.execute = AsyncMock(return_value=None)
    else:
        mock_pool.execute = AsyncMock(side_effect=Exception("DB connection failed"))

    from fastapi import FastAPI
    from backend.routers import health
    from backend.models.errors import ChatBackendError, chat_backend_error_handler

    app = FastAPI()
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.include_router(health.router, prefix="/api/v1")

    app.state.db_pool = mock_pool
    app.state.settings = settings
    app.state.rate_limiter = SlidingWindowRateLimiter()

    return app, qdrant_healthy


class TestHealthEndpoint:
    """Integration tests for GET /api/v1/health."""

    @pytest.mark.asyncio
    async def test_healthy_when_all_deps_ok(self):
        """Returns 200 with status 'healthy' when all dependencies are up."""
        app, _ = _make_app(pool_healthy=True, qdrant_healthy=True)

        with patch("backend.routers.health.QdrantClient") as MockQdrant:
            mock_qdrant_instance = MagicMock()
            mock_qdrant_instance.get_collections.return_value = MagicMock()
            MockQdrant.return_value = mock_qdrant_instance

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app),
                base_url="http://test",
            ) as client:
                response = await client.get("/api/v1/health")

            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "healthy"
            assert data["components"]["database"]["status"] == "ok"
            assert data["components"]["vector_store"]["status"] == "ok"
            assert "timestamp" in data

    @pytest.mark.asyncio
    async def test_degraded_when_db_down(self):
        """Returns 503 with status 'degraded' when database is unreachable."""
        app, _ = _make_app(pool_healthy=False, qdrant_healthy=True)

        with patch("backend.routers.health.QdrantClient") as MockQdrant:
            mock_qdrant_instance = MagicMock()
            mock_qdrant_instance.get_collections.return_value = MagicMock()
            MockQdrant.return_value = mock_qdrant_instance

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app),
                base_url="http://test",
            ) as client:
                response = await client.get("/api/v1/health")

            assert response.status_code == 503
            data = response.json()
            assert data["status"] == "degraded"
            assert data["components"]["database"]["status"] == "error"
            assert data["components"]["vector_store"]["status"] == "ok"

    @pytest.mark.asyncio
    async def test_degraded_when_qdrant_down(self):
        """Returns 503 with status 'degraded' when vector store is unreachable."""
        app, _ = _make_app(pool_healthy=True, qdrant_healthy=False)

        with patch("backend.routers.health.QdrantClient") as MockQdrant:
            MockQdrant.side_effect = Exception("Connection refused")

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app),
                base_url="http://test",
            ) as client:
                response = await client.get("/api/v1/health")

            assert response.status_code == 503
            data = response.json()
            assert data["status"] == "degraded"
            assert data["components"]["database"]["status"] == "ok"
            assert data["components"]["vector_store"]["status"] == "error"
