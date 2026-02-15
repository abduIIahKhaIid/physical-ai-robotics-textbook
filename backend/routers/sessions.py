"""Session endpoints: list sessions, get message history."""

from __future__ import annotations

import uuid

import asyncpg
from fastapi import APIRouter, Depends, Query

from backend.dependencies import get_current_user, get_db_pool
from backend.models.api_models import MessageList, SessionList
from backend.services import session_service

router = APIRouter()


@router.get("/sessions", response_model=SessionList)
async def list_sessions(
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
    pool: asyncpg.Pool = Depends(get_db_pool),
    user: dict = Depends(get_current_user),
) -> SessionList:
    """List the current user's chat sessions."""
    return await session_service.list_user_sessions(
        pool, user["id"], limit, offset
    )


@router.get("/sessions/{session_id}/messages", response_model=MessageList)
async def get_session_messages(
    session_id: uuid.UUID,
    pool: asyncpg.Pool = Depends(get_db_pool),
    user: dict = Depends(get_current_user),
) -> MessageList:
    """Get all messages in a session."""
    return await session_service.get_session_messages(
        pool, session_id, user["id"]
    )
