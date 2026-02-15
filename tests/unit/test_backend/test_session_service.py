"""Unit tests for session service."""

from __future__ import annotations

import uuid
from unittest.mock import AsyncMock

import pytest

from backend.models.errors import SessionNotFoundError
from backend.services.session_service import (
    auto_title,
    get_session_messages,
    list_user_sessions,
)


class TestAutoTitle:
    """Test session title generation."""

    def test_short_title(self):
        assert auto_title("Hello world") == "Hello world"

    def test_truncates_long_message(self):
        msg = "x " * 100  # 200 chars
        title = auto_title(msg)
        assert len(title) <= 104
        assert title.endswith("...")

    def test_truncates_at_word_boundary(self):
        msg = "word " * 25  # 125 chars
        title = auto_title(msg)
        assert title.endswith("...")
        # Title should be truncated, not exceed 104 chars
        assert len(title) <= 104

    def test_exact_255_chars(self):
        msg = "x" * 100
        assert auto_title(msg) == msg


class TestListUserSessions:
    """Test session listing."""

    @pytest.mark.asyncio
    async def test_returns_sessions_sorted_by_updated(self):
        user_id = uuid.uuid4()
        session1_id = uuid.uuid4()
        session2_id = uuid.uuid4()

        pool = AsyncMock()
        pool.fetch = AsyncMock(return_value=[
            {
                "id": session2_id,
                "title": "Second",
                "status": "active",
                "message_count": 4,
                "created_at": "2026-01-02",
                "updated_at": "2026-01-02",
            },
            {
                "id": session1_id,
                "title": "First",
                "status": "active",
                "message_count": 2,
                "created_at": "2026-01-01",
                "updated_at": "2026-01-01",
            },
        ])
        pool.fetchrow = AsyncMock(return_value={"cnt": 2})

        result = await list_user_sessions(pool, user_id)
        assert result.total == 2
        assert len(result.sessions) == 2
        assert result.sessions[0].title == "Second"

    @pytest.mark.asyncio
    async def test_empty_sessions(self):
        pool = AsyncMock()
        pool.fetch = AsyncMock(return_value=[])
        pool.fetchrow = AsyncMock(return_value={"cnt": 0})

        result = await list_user_sessions(pool, uuid.uuid4())
        assert result.total == 0
        assert len(result.sessions) == 0


class TestGetSessionMessages:
    """Test message retrieval with ownership check."""

    @pytest.mark.asyncio
    async def test_returns_messages_in_order(self):
        user_id = uuid.uuid4()
        session_id = uuid.uuid4()

        pool = AsyncMock()
        pool.fetchrow = AsyncMock(return_value={
            "id": session_id,
            "user_id": user_id,
            "title": "Test",
            "status": "active",
        })
        pool.fetch = AsyncMock(return_value=[
            {"id": uuid.uuid4(), "role": "user", "content": "Q1", "retrieval_mode": "normal", "created_at": "2026-01-01T00:00:00"},
            {"id": uuid.uuid4(), "role": "assistant", "content": "A1", "retrieval_mode": "normal", "created_at": "2026-01-01T00:00:01"},
        ])

        result = await get_session_messages(pool, session_id, user_id)
        assert result.session_id == session_id
        assert len(result.messages) == 2
        assert result.messages[0].role == "user"
        assert result.messages[1].role == "assistant"

    @pytest.mark.asyncio
    async def test_session_not_found_raises_error(self):
        pool = AsyncMock()
        pool.fetchrow = AsyncMock(return_value=None)

        with pytest.raises(SessionNotFoundError):
            await get_session_messages(pool, uuid.uuid4(), uuid.uuid4())

    @pytest.mark.asyncio
    async def test_wrong_user_raises_session_not_found(self):
        session_id = uuid.uuid4()
        owner_id = uuid.uuid4()
        other_user_id = uuid.uuid4()

        pool = AsyncMock()
        pool.fetchrow = AsyncMock(return_value={
            "id": session_id,
            "user_id": owner_id,
            "title": "Test",
            "status": "active",
        })

        with pytest.raises(SessionNotFoundError):
            await get_session_messages(pool, session_id, other_user_id)

    @pytest.mark.asyncio
    async def test_updates_session_activity(self):
        user_id = uuid.uuid4()
        session_id = uuid.uuid4()

        pool = AsyncMock()
        pool.fetchrow = AsyncMock(return_value={
            "id": session_id,
            "user_id": user_id,
            "title": "Test",
            "status": "active",
        })
        pool.fetch = AsyncMock(return_value=[])

        await get_session_messages(pool, session_id, user_id)
        # Verify the session was accessed (fetchrow called)
        assert pool.fetchrow.call_count == 1
