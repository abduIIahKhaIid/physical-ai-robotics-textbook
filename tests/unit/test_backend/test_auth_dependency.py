"""Unit tests for authentication dependency functions."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone, timedelta
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from fastapi import HTTPException


@pytest.fixture
def mock_pool():
    return AsyncMock()


@pytest.fixture
def mock_request():
    req = MagicMock()
    req.client.host = "127.0.0.1"
    return req


def _make_ba_session(user_id="ba-user-123", expired=False):
    """Create a mock BA session record."""
    expires = datetime.now(timezone.utc) + (timedelta(hours=-1) if expired else timedelta(hours=24))
    record = MagicMock()
    record.__getitem__ = lambda self, key: {
        "userId": user_id,
        "expiresAt": expires,
        "token": "valid-ba-token",
    }[key]
    return record


class TestGetCurrentUser:
    """Tests for get_current_user dependency."""

    @pytest.mark.asyncio
    async def test_valid_ba_session_returns_identified_user(self, mock_request, mock_pool):
        """T018: Valid BA session token returns tier='identified' with correct userId."""
        mock_request.headers = {"Authorization": "Bearer valid-ba-token"}
        ba_session = _make_ba_session(user_id="ba-user-123")

        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=ba_session)

            from backend.dependencies import get_current_user
            user = await get_current_user(mock_request, mock_pool)

        assert user["id"] == "ba-user-123"
        assert user["tier"] == "identified"
        assert user["token"] == "valid-ba-token"

    @pytest.mark.asyncio
    async def test_expired_ba_session_falls_back_to_anonymous(self, mock_request, mock_pool):
        """T019: Expired BA session falls back to anonymous user creation."""
        mock_request.headers = {"Authorization": "Bearer expired-token"}

        anon_user_id = uuid.uuid4()
        with patch("backend.dependencies.queries") as mock_queries:
            # BA session not found (expired sessions are filtered by the query)
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=None)
            # No legacy user found either
            mock_queries.get_user_by_token = AsyncMock(return_value=None)
            # Create anonymous user
            mock_queries.create_user = AsyncMock(return_value=anon_user_id)

            from backend.dependencies import get_current_user
            user = await get_current_user(mock_request, mock_pool)

        assert user["tier"] == "anonymous"
        assert user["id"] == anon_user_id

    @pytest.mark.asyncio
    async def test_no_token_creates_anonymous_user(self, mock_request, mock_pool):
        """T020: No Authorization header creates anonymous user with anon- token."""
        mock_request.headers = {}

        anon_user_id = uuid.uuid4()
        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_user_by_token = AsyncMock(return_value=None)
            mock_queries.create_user = AsyncMock(return_value=anon_user_id)

            from backend.dependencies import get_current_user
            user = await get_current_user(mock_request, mock_pool)

        assert user["tier"] == "anonymous"
        assert user["token"].startswith("anon-")

    @pytest.mark.asyncio
    async def test_anon_token_skips_ba_lookup(self, mock_request, mock_pool):
        """Anon- prefixed tokens skip BA session lookup entirely."""
        mock_request.headers = {"Authorization": "Bearer anon-127.0.0.1"}

        user_data = {"id": uuid.uuid4(), "token": "anon-127.0.0.1", "tier": "anonymous", "ip_address": "127.0.0.1"}

        class FakeRecord:
            def __getitem__(self, key):
                return user_data[key]
            def keys(self):
                return user_data.keys()
            def values(self):
                return user_data.values()
            def items(self):
                return user_data.items()
            def __iter__(self):
                return iter(user_data)
            def __len__(self):
                return len(user_data)

        legacy_user = FakeRecord()

        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_user_by_token = AsyncMock(return_value=legacy_user)
            mock_queries.update_user_last_seen = AsyncMock()

            from backend.dependencies import get_current_user
            user = await get_current_user(mock_request, mock_pool)

        # Should NOT have called BA session lookup
        mock_queries.get_ba_session_by_token.assert_not_called()


class TestGetAuthenticatedUser:
    """Tests for get_authenticated_user dependency."""

    @pytest.mark.asyncio
    async def test_authenticated_user_required_returns_401_no_token(self, mock_request, mock_pool):
        """T021: get_authenticated_user with no token returns 401."""
        mock_request.headers = {}

        from backend.dependencies import get_authenticated_user
        with pytest.raises(HTTPException) as exc_info:
            await get_authenticated_user(mock_request, mock_pool)
        assert exc_info.value.status_code == 401

    @pytest.mark.asyncio
    async def test_authenticated_user_required_returns_401_anon_token(self, mock_request, mock_pool):
        """get_authenticated_user with anon- token returns 401."""
        mock_request.headers = {"Authorization": "Bearer anon-127.0.0.1"}

        from backend.dependencies import get_authenticated_user
        with pytest.raises(HTTPException) as exc_info:
            await get_authenticated_user(mock_request, mock_pool)
        assert exc_info.value.status_code == 401

    @pytest.mark.asyncio
    async def test_authenticated_user_required_returns_401_invalid_token(self, mock_request, mock_pool):
        """get_authenticated_user with invalid BA token returns 401."""
        mock_request.headers = {"Authorization": "Bearer invalid-token"}

        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=None)

            from backend.dependencies import get_authenticated_user
            with pytest.raises(HTTPException) as exc_info:
                await get_authenticated_user(mock_request, mock_pool)
            assert exc_info.value.status_code == 401

    @pytest.mark.asyncio
    async def test_authenticated_user_returns_valid_user(self, mock_request, mock_pool):
        """get_authenticated_user with valid BA token returns user dict."""
        mock_request.headers = {"Authorization": "Bearer valid-token"}
        ba_session = _make_ba_session(user_id="ba-user-456")

        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=ba_session)

            from backend.dependencies import get_authenticated_user
            user = await get_authenticated_user(mock_request, mock_pool)

        assert user["id"] == "ba-user-456"
        assert user["tier"] == "identified"

    @pytest.mark.asyncio
    async def test_invalidated_session_returns_anonymous(self, mock_request, mock_pool):
        """T054: Deleted BA session falls back to anonymous in get_current_user."""
        mock_request.headers = {"Authorization": "Bearer deleted-session-token"}

        anon_user_id = uuid.uuid4()
        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=None)
            mock_queries.get_user_by_token = AsyncMock(return_value=None)
            mock_queries.create_user = AsyncMock(return_value=anon_user_id)

            from backend.dependencies import get_current_user
            user = await get_current_user(mock_request, mock_pool)

        assert user["tier"] == "anonymous"
