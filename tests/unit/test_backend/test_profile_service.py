"""Unit tests for profile service business logic."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from unittest.mock import AsyncMock, MagicMock, patch

import pytest


def _make_profile_record(user_id="ba-user-123", onboarding_completed=True, **overrides):
    """Create a mock profile DB record."""
    now = datetime.now(timezone.utc)
    data = {
        "id": uuid.uuid4(),
        "user_id": user_id,
        "software_level": "intermediate",
        "programming_languages": "Python",
        "hardware_level": "hobbyist",
        "available_hardware": ["jetson_nano_orin"],
        "learning_goal": "Build robots",
        "preferred_pace": "self_paced",
        "onboarding_completed": onboarding_completed,
        "created_at": now,
        "updated_at": now,
    }
    data.update(overrides)

    class DictableRecord:
        def __getitem__(self, key):
            return data[key]
        def __contains__(self, key):
            return key in data
        def keys(self):
            return data.keys()
        def values(self):
            return data.values()
        def items(self):
            return data.items()
        def __iter__(self):
            return iter(data)
        def __len__(self):
            return len(data)

    return DictableRecord()


class TestGetUserProfile:
    @pytest.mark.asyncio
    async def test_get_user_profile_returns_defaults_when_no_profile(self):
        """T061: get_user_profile for user with no profile returns default values."""
        mock_pool = AsyncMock()

        with patch("backend.services.profile_service.queries") as mock_queries:
            mock_queries.get_profile_by_user_id = AsyncMock(return_value=None)

            from backend.services.profile_service import get_user_profile
            result = await get_user_profile(mock_pool, "nonexistent-user")

        assert result["onboarding_completed"] is False
        assert result["software_level"] == "beginner"
        assert result["hardware_level"] == "none"
        assert result["user_id"] == "nonexistent-user"
        assert result["created_at"] is None

    @pytest.mark.asyncio
    async def test_get_user_profile_returns_existing_profile(self):
        """get_user_profile returns stored profile when it exists."""
        mock_pool = AsyncMock()
        record = _make_profile_record()

        with patch("backend.services.profile_service.queries") as mock_queries:
            mock_queries.get_profile_by_user_id = AsyncMock(return_value=record)

            from backend.services.profile_service import get_user_profile
            result = await get_user_profile(mock_pool, "ba-user-123")

        assert result["onboarding_completed"] is True
        assert result["software_level"] == "intermediate"


class TestCreateOrUpdateProfile:
    @pytest.mark.asyncio
    async def test_create_profile_sets_onboarding_completed(self):
        """T062: create_or_update_profile sets onboarding_completed=true."""
        mock_pool = AsyncMock()
        record = _make_profile_record(onboarding_completed=True)

        with patch("backend.services.profile_service.queries") as mock_queries:
            mock_queries.upsert_profile = AsyncMock(return_value=record)

            from backend.services.profile_service import create_or_update_profile
            result = await create_or_update_profile(
                mock_pool,
                user_id="ba-user-123",
                software_level="intermediate",
                hardware_level="hobbyist",
                preferred_pace="self_paced",
            )

        assert result["onboarding_completed"] is True
        mock_queries.upsert_profile.assert_called_once()
