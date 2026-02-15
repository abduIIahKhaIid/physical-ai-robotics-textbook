"""Integration tests for session endpoints."""

from __future__ import annotations

import json
import uuid
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime, timezone

import pytest
import httpx

from backend.config import BackendSettings
from backend.services.rate_limiter import SlidingWindowRateLimiter


def _make_app():
    """Create test app with mocked DB."""
    settings = BackendSettings(
        database_url="postgresql://test",
        gemini_api_key="test-key",
    )

    mock_pool = AsyncMock()
    mock_gemini = MagicMock()

    from fastapi import FastAPI
    from backend.routers import chat, sessions, health
    from backend.models.errors import ChatBackendError, chat_backend_error_handler, generic_error_handler

    app = FastAPI()
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.add_exception_handler(Exception, generic_error_handler)
    app.include_router(chat.router, prefix="/api/v1")
    app.include_router(sessions.router, prefix="/api/v1")
    app.include_router(health.router, prefix="/api/v1")

    app.state.db_pool = mock_pool
    app.state.gemini_client = mock_gemini
    app.state.rate_limiter = SlidingWindowRateLimiter()
    app.state.settings = settings

    return app, mock_pool


class TestSessionEndpoints:
    """Integration tests for session list and message history."""

    @pytest.mark.asyncio
    async def test_list_sessions(self):
        """GET /api/v1/sessions returns session list."""
        app, mock_pool = _make_app()

        user_id = uuid.uuid4()
        session_id = uuid.uuid4()
        now = datetime.now(timezone.utc)

        # Mock user lookup
        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": user_id, "token": "anon-127.0.0.1", "tier": "anonymous", "ip_address": "127.0.0.1", "created_at": now, "last_seen_at": now},
        ])
        mock_pool.execute = AsyncMock()

        # Mock session listing and count
        mock_pool.fetch = AsyncMock(return_value=[
            {
                "id": session_id,
                "title": "Test Session",
                "status": "active",
                "message_count": 4,
                "created_at": now,
                "updated_at": now,
            },
        ])

        # The count query returns fetchrow
        original_fetchrow = mock_pool.fetchrow
        call_count = [0]
        async def multi_fetchrow(*args, **kwargs):
            call_count[0] += 1
            if call_count[0] == 1:
                return {"id": user_id, "token": "anon-127.0.0.1", "tier": "anonymous", "ip_address": "127.0.0.1", "created_at": now, "last_seen_at": now}
            elif call_count[0] == 2:
                return {"cnt": 1}
            return None
        mock_pool.fetchrow = AsyncMock(side_effect=multi_fetchrow)

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app),
            base_url="http://test",
        ) as client:
            response = await client.get("/api/v1/sessions")

        assert response.status_code == 200
        data = response.json()
        assert "sessions" in data
        assert data["total"] == 1
        assert data["sessions"][0]["title"] == "Test Session"
        assert data["sessions"][0]["message_count"] == 4

    @pytest.mark.asyncio
    async def test_get_session_messages(self):
        """GET /api/v1/sessions/{id}/messages returns message history."""
        app, mock_pool = _make_app()

        user_id = uuid.uuid4()
        session_id = uuid.uuid4()
        now = datetime.now(timezone.utc)

        call_count = [0]
        async def multi_fetchrow(*args, **kwargs):
            call_count[0] += 1
            if call_count[0] == 1:
                # get_user_by_token
                return {"id": user_id, "token": "anon-127.0.0.1", "tier": "anonymous", "ip_address": "127.0.0.1", "created_at": now, "last_seen_at": now}
            elif call_count[0] == 2:
                # get_session
                return {"id": session_id, "user_id": user_id, "title": "Test", "status": "active", "created_at": now, "updated_at": now}
            return None
        mock_pool.fetchrow = AsyncMock(side_effect=multi_fetchrow)
        mock_pool.execute = AsyncMock()

        msg1_id = uuid.uuid4()
        msg2_id = uuid.uuid4()
        mock_pool.fetch = AsyncMock(return_value=[
            {"id": msg1_id, "role": "user", "content": "Question 1", "retrieval_mode": "normal", "created_at": now},
            {"id": msg2_id, "role": "assistant", "content": "Answer 1", "retrieval_mode": "normal", "created_at": now},
        ])

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app),
            base_url="http://test",
        ) as client:
            response = await client.get(f"/api/v1/sessions/{session_id}/messages")

        assert response.status_code == 200
        data = response.json()
        assert data["session_id"] == str(session_id)
        assert len(data["messages"]) == 2
        assert data["messages"][0]["role"] == "user"
        assert data["messages"][1]["role"] == "assistant"

    @pytest.mark.asyncio
    async def test_get_messages_session_not_found(self):
        """GET /api/v1/sessions/{id}/messages returns 404 for missing session."""
        app, mock_pool = _make_app()

        user_id = uuid.uuid4()
        now = datetime.now(timezone.utc)

        call_count = [0]
        async def multi_fetchrow(*args, **kwargs):
            call_count[0] += 1
            if call_count[0] == 1:
                return {"id": user_id, "token": "anon-127.0.0.1", "tier": "anonymous", "ip_address": "127.0.0.1", "created_at": now, "last_seen_at": now}
            return None  # session not found
        mock_pool.fetchrow = AsyncMock(side_effect=multi_fetchrow)
        mock_pool.execute = AsyncMock()

        fake_session_id = uuid.uuid4()
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app),
            base_url="http://test",
        ) as client:
            response = await client.get(f"/api/v1/sessions/{fake_session_id}/messages")

        assert response.status_code == 404
        data = response.json()
        assert data["error"]["code"] == "session_not_found"
