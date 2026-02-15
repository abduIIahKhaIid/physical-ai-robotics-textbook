"""FastAPI dependency injection functions."""

from __future__ import annotations

import logging
import uuid
from typing import Any

import asyncpg
from fastapi import Depends, Request

from backend.config import load_backend_settings
from backend.db import queries

logger = logging.getLogger(__name__)


async def get_db_pool(request: Request) -> asyncpg.Pool:
    """Return the app-level asyncpg connection pool."""
    return request.app.state.db_pool


async def get_gemini_client(request: Request) -> Any:
    """Return the app-level Gemini client."""
    return request.app.state.gemini_client


async def get_current_user(
    request: Request,
    pool: asyncpg.Pool = Depends(get_db_pool),
) -> dict:
    """Extract user from Authorization header or create anonymous user.

    Returns a dict with keys: id, token, tier, ip_address.
    """
    auth_header = request.headers.get("Authorization", "")

    if auth_header.startswith("Bearer "):
        token = auth_header[7:].strip()
        if token:
            user = await queries.get_user_by_token(pool, token)
            if user:
                await queries.update_user_last_seen(pool, user["id"])
                return dict(user)

    # Anonymous user: identify by IP
    client_ip = request.client.host if request.client else "0.0.0.0"
    token = f"anon-{client_ip}"

    user = await queries.get_user_by_token(pool, token)
    if user:
        await queries.update_user_last_seen(pool, user["id"])
        return dict(user)

    # Create new anonymous user
    user_id = await queries.create_user(pool, token=token, tier="anonymous", ip=client_ip)
    return {
        "id": user_id,
        "token": token,
        "tier": "anonymous",
        "ip_address": client_ip,
    }
