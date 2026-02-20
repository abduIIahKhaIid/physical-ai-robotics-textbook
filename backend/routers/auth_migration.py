"""Auth migration endpoint: POST /api/v1/auth/migrate-sessions."""

from __future__ import annotations

import asyncpg
from fastapi import APIRouter, Depends
from pydantic import BaseModel

from backend.db import queries
from backend.dependencies import get_authenticated_user, get_db_pool

router = APIRouter()


class MigrateSessionsRequest(BaseModel):
    """Request body for migrating anonymous sessions."""

    anonymous_token: str


class MigrateSessionsResponse(BaseModel):
    """Response body for session migration."""

    migrated_count: int


@router.post("/auth/migrate-sessions", response_model=MigrateSessionsResponse)
async def migrate_sessions(
    body: MigrateSessionsRequest,
    user: dict = Depends(get_authenticated_user),
    pool: asyncpg.Pool = Depends(get_db_pool),
) -> MigrateSessionsResponse:
    """Migrate anonymous chat sessions to the authenticated user."""
    count = await queries.migrate_anonymous_sessions(
        pool, body.anonymous_token, user["id"]
    )
    return MigrateSessionsResponse(migrated_count=count)
