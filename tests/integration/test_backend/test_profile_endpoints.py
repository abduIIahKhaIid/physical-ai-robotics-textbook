"""Integration tests for profile CRUD endpoints."""

from __future__ import annotations

import uuid
from unittest.mock import AsyncMock, MagicMock, patch
from datetime import datetime, timezone

import pytest
import httpx

from backend.models.profile import ProfileCreate


def _make_test_app():
    """Create a FastAPI test app with mocked dependencies."""
    from fastapi import FastAPI
    from backend.routers import profile, auth_migration
    from backend.models.errors import ChatBackendError, chat_backend_error_handler, generic_error_handler

    app = FastAPI()
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.add_exception_handler(Exception, generic_error_handler)
    app.include_router(profile.router, prefix="/api/v1")
    app.include_router(auth_migration.router, prefix="/api/v1")

    mock_pool = AsyncMock()
    app.state.db_pool = mock_pool

    return app, mock_pool


def _make_profile_record(user_id="ba-user-123", **overrides):
    """Create a mock profile DB record."""
    now = datetime.now(timezone.utc)
    data = {
        "id": uuid.uuid4(),
        "user_id": user_id,
        "software_level": "intermediate",
        "programming_languages": "Python, C++",
        "hardware_level": "hobbyist",
        "available_hardware": ["jetson_nano_orin"],
        "learning_goal": "Build robots",
        "preferred_pace": "self_paced",
        "onboarding_completed": True,
        "created_at": now,
        "updated_at": now,
    }
    data.update(overrides)
    record = MagicMock()
    record.__getitem__ = lambda self, key: data[key]
    record.__contains__ = lambda self, key: key in data
    record.keys = lambda self: data.keys()
    record.__iter__ = lambda self: iter(data)
    # Make dict(record) work
    record.__len__ = lambda self: len(data)

    # For dict() conversion we need to support items()
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


def _override_auth(app, user_id="ba-user-123"):
    """Override get_authenticated_user to return a specific user."""
    from backend.dependencies import get_authenticated_user, get_db_pool

    async def mock_auth():
        return {"id": user_id, "token": "test-token", "tier": "identified", "ip_address": "127.0.0.1"}

    async def mock_pool():
        return app.state.db_pool

    app.dependency_overrides[get_authenticated_user] = mock_auth
    app.dependency_overrides[get_db_pool] = mock_pool


class TestPostProfile:
    @pytest.mark.asyncio
    async def test_post_profile_creates_and_returns_201(self):
        """T036: POST /api/v1/profile with valid data creates profile and returns 201."""
        app, mock_pool = _make_test_app()
        _override_auth(app)

        profile_record = _make_profile_record()
        with patch("backend.services.profile_service.queries") as mock_queries:
            mock_queries.upsert_profile = AsyncMock(return_value=profile_record)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.post(
                    "/api/v1/profile",
                    json={
                        "software_level": "intermediate",
                        "programming_languages": "Python, C++",
                        "hardware_level": "hobbyist",
                        "available_hardware": ["jetson_nano_orin"],
                        "learning_goal": "Build robots",
                        "preferred_pace": "self_paced",
                    },
                    headers={"Authorization": "Bearer test-token"},
                )

        assert resp.status_code == 201
        data = resp.json()
        assert data["onboarding_completed"] is True
        assert data["software_level"] == "intermediate"

    @pytest.mark.asyncio
    async def test_post_profile_rejects_invalid_software_level(self):
        """T038: POST with invalid software_level returns 422."""
        app, _ = _make_test_app()
        _override_auth(app)

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            resp = await client.post(
                "/api/v1/profile",
                json={
                    "software_level": "expert",
                    "hardware_level": "hobbyist",
                    "preferred_pace": "self_paced",
                },
                headers={"Authorization": "Bearer test-token"},
            )

        assert resp.status_code == 422

    @pytest.mark.asyncio
    async def test_post_profile_rejects_long_learning_goal(self):
        """T039: POST with learning_goal of 501 chars returns 422."""
        app, _ = _make_test_app()
        _override_auth(app)

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            resp = await client.post(
                "/api/v1/profile",
                json={
                    "software_level": "beginner",
                    "hardware_level": "none",
                    "preferred_pace": "self_paced",
                    "learning_goal": "x" * 501,
                },
                headers={"Authorization": "Bearer test-token"},
            )

        assert resp.status_code == 422


class TestGetProfile:
    @pytest.mark.asyncio
    async def test_get_profile_returns_existing_profile(self):
        """T037: GET /api/v1/profile returns saved profile data."""
        app, mock_pool = _make_test_app()
        _override_auth(app)

        profile_record = _make_profile_record()
        with patch("backend.services.profile_service.queries") as mock_queries:
            mock_queries.get_profile_by_user_id = AsyncMock(return_value=profile_record)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.get(
                    "/api/v1/profile",
                    headers={"Authorization": "Bearer test-token"},
                )

        assert resp.status_code == 200
        data = resp.json()
        assert data["software_level"] == "intermediate"
        assert data["onboarding_completed"] is True

    @pytest.mark.asyncio
    async def test_get_profile_returns_defaults_when_no_profile(self):
        """GET /api/v1/profile returns defaults when no profile exists."""
        app, mock_pool = _make_test_app()
        _override_auth(app)

        with patch("backend.services.profile_service.queries") as mock_queries:
            mock_queries.get_profile_by_user_id = AsyncMock(return_value=None)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.get(
                    "/api/v1/profile",
                    headers={"Authorization": "Bearer test-token"},
                )

        assert resp.status_code == 200
        data = resp.json()
        assert data["onboarding_completed"] is False
        assert data["software_level"] == "beginner"

    @pytest.mark.asyncio
    async def test_profile_requires_authentication(self):
        """T040: GET /api/v1/profile without token returns 401."""
        app, mock_pool = _make_test_app()
        # Do NOT override auth — let real dependency run

        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=None)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.get("/api/v1/profile")

        assert resp.status_code == 401


class TestPatchProfile:
    @pytest.mark.asyncio
    async def test_patch_profile_updates_partial_fields(self):
        """T049: PATCH /api/v1/profile updates only specified fields."""
        app, mock_pool = _make_test_app()
        _override_auth(app)

        existing = _make_profile_record()
        updated = _make_profile_record(software_level="advanced")

        with patch("backend.routers.profile.queries") as mock_queries:
            mock_queries.get_profile_by_user_id = AsyncMock(return_value=existing)
            mock_queries.update_profile = AsyncMock(return_value=updated)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.patch(
                    "/api/v1/profile",
                    json={"software_level": "advanced"},
                    headers={"Authorization": "Bearer test-token"},
                )

        assert resp.status_code == 200
        data = resp.json()
        assert data["software_level"] == "advanced"

    @pytest.mark.asyncio
    async def test_patch_profile_404_without_existing_profile(self):
        """T050: PATCH without prior POST returns 404."""
        app, mock_pool = _make_test_app()
        _override_auth(app)

        with patch("backend.routers.profile.queries") as mock_queries:
            mock_queries.get_profile_by_user_id = AsyncMock(return_value=None)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.patch(
                    "/api/v1/profile",
                    json={"software_level": "advanced"},
                    headers={"Authorization": "Bearer test-token"},
                )

        assert resp.status_code == 404


class TestMigrateSessions:
    @pytest.mark.asyncio
    async def test_migrate_endpoint_requires_auth(self):
        """T027: POST /api/v1/auth/migrate-sessions without token returns 401."""
        app, _ = _make_test_app()
        # Do NOT override auth

        with patch("backend.dependencies.queries") as mock_queries:
            mock_queries.get_ba_session_by_token = AsyncMock(return_value=None)

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://test"
            ) as client:
                resp = await client.post(
                    "/api/v1/auth/migrate-sessions",
                    json={"anonymous_token": "anon-127.0.0.1"},
                )

        assert resp.status_code == 401
