"""Async SQL query functions for the chat backend."""

from __future__ import annotations

import uuid

import asyncpg


async def create_user(
    pool: asyncpg.Pool, token: str, tier: str = "anonymous", ip: str | None = None
) -> uuid.UUID:
    """Create a new user and return their ID."""
    row = await pool.fetchrow(
        """
        INSERT INTO users (token, tier, ip_address)
        VALUES ($1, $2, $3::inet)
        RETURNING id
        """,
        token,
        tier,
        ip,
    )
    return row["id"]


async def get_user_by_token(
    pool: asyncpg.Pool, token: str
) -> asyncpg.Record | None:
    """Look up a user by their bearer token."""
    return await pool.fetchrow(
        "SELECT id, token, tier, ip_address, created_at, last_seen_at FROM users WHERE token = $1",
        token,
    )


async def update_user_last_seen(pool: asyncpg.Pool, user_id: uuid.UUID) -> None:
    """Update a user's last_seen_at timestamp."""
    await pool.execute(
        "UPDATE users SET last_seen_at = now() WHERE id = $1", user_id
    )


async def create_session(
    pool: asyncpg.Pool, user_id: uuid.UUID, title: str = ""
) -> uuid.UUID:
    """Create a new chat session and return its ID."""
    row = await pool.fetchrow(
        """
        INSERT INTO sessions (user_id, title)
        VALUES ($1, $2)
        RETURNING id
        """,
        user_id,
        title,
    )
    return row["id"]


async def get_session(
    pool: asyncpg.Pool, session_id: uuid.UUID
) -> asyncpg.Record | None:
    """Get a session by ID."""
    return await pool.fetchrow(
        "SELECT id, user_id, title, status, created_at, updated_at FROM sessions WHERE id = $1",
        session_id,
    )


async def list_sessions(
    pool: asyncpg.Pool,
    user_id: uuid.UUID,
    limit: int = 20,
    offset: int = 0,
) -> list[asyncpg.Record]:
    """List sessions for a user, sorted by most recent activity."""
    return await pool.fetch(
        """
        SELECT s.id, s.title, s.status, s.created_at, s.updated_at,
               (SELECT COUNT(*) FROM messages m WHERE m.session_id = s.id) AS message_count
        FROM sessions s
        WHERE s.user_id = $1 AND s.status = 'active'
        ORDER BY s.updated_at DESC
        LIMIT $2 OFFSET $3
        """,
        user_id,
        limit,
        offset,
    )


async def count_sessions(pool: asyncpg.Pool, user_id: uuid.UUID) -> int:
    """Count total active sessions for a user."""
    row = await pool.fetchrow(
        "SELECT COUNT(*) AS cnt FROM sessions WHERE user_id = $1 AND status = 'active'",
        user_id,
    )
    return row["cnt"]


async def create_message(
    pool: asyncpg.Pool,
    session_id: uuid.UUID,
    role: str,
    content: str,
    retrieval_mode: str | None = None,
    selected_text: str | None = None,
    chunk_count: int | None = None,
    latency_ms: int | None = None,
) -> uuid.UUID:
    """Insert a message and return its ID."""
    row = await pool.fetchrow(
        """
        INSERT INTO messages (session_id, role, content, retrieval_mode, selected_text, chunk_count, latency_ms)
        VALUES ($1, $2, $3, $4, $5, $6, $7)
        RETURNING id
        """,
        session_id,
        role,
        content,
        retrieval_mode,
        selected_text,
        chunk_count,
        latency_ms,
    )
    return row["id"]


async def get_messages(
    pool: asyncpg.Pool, session_id: uuid.UUID
) -> list[asyncpg.Record]:
    """Get messages for a session, ordered chronologically."""
    return await pool.fetch(
        """
        SELECT id, session_id, role, content, retrieval_mode, created_at
        FROM messages
        WHERE session_id = $1
        ORDER BY created_at ASC
        """,
        session_id,
    )


async def update_session_activity(
    pool: asyncpg.Pool, session_id: uuid.UUID
) -> None:
    """Update a session's updated_at timestamp."""
    await pool.execute(
        "UPDATE sessions SET updated_at = now() WHERE id = $1", session_id
    )


async def update_session_title(
    pool: asyncpg.Pool, session_id: uuid.UUID, title: str
) -> None:
    """Update a session's title."""
    await pool.execute(
        "UPDATE sessions SET title = $1 WHERE id = $2", title, session_id
    )
