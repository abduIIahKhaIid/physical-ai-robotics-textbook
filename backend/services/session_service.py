"""Session and message management service."""

from __future__ import annotations

import uuid
from typing import Any

from backend.db import queries
from backend.models.api_models import (
    MessageList,
    MessageResponse,
    SessionList,
    SessionSummary,
)
from backend.models.errors import SessionNotFoundError


async def list_user_sessions(
    pool: Any, user_id: uuid.UUID, limit: int = 20, offset: int = 0
) -> SessionList:
    """List sessions for a user with message counts."""
    rows = await queries.list_sessions(pool, user_id, limit, offset)
    total = await queries.count_sessions(pool, user_id)

    sessions = [
        SessionSummary(
            id=row["id"],
            title=row["title"],
            status=row["status"],
            message_count=row["message_count"],
            created_at=row["created_at"],
            updated_at=row["updated_at"],
        )
        for row in rows
    ]

    return SessionList(sessions=sessions, total=total)


async def get_session_messages(
    pool: Any, session_id: uuid.UUID, user_id: uuid.UUID
) -> MessageList:
    """Get messages for a session, verifying ownership."""
    session = await queries.get_session(pool, session_id)
    if session is None:
        raise SessionNotFoundError(f"Session {session_id} not found")
    if session["user_id"] != user_id:
        raise SessionNotFoundError(f"Session {session_id} not found")

    rows = await queries.get_messages(pool, session_id)

    messages = [
        MessageResponse(
            id=row["id"],
            role=row["role"],
            content=row["content"],
            retrieval_mode=row.get("retrieval_mode"),
            created_at=row["created_at"],
        )
        for row in rows
    ]

    return MessageList(messages=messages, session_id=session_id)


def auto_title(message: str) -> str:
    """Generate session title from first user message.

    Truncates at 100 chars on word boundary.
    """
    title = message[:100]
    if len(message) > 100:
        last_space = title.rfind(" ")
        if last_space > 20:
            title = title[:last_space]
        title += "..."
    return title
