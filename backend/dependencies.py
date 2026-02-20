"""FastAPI dependency injection functions."""

from __future__ import annotations

import logging
import uuid
from typing import Any

import asyncpg
from fastapi import Depends, HTTPException, Request

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

    Flow:
    1. If Bearer token present and not anon- prefix → check Better-Auth session table
    2. If valid BA session → return identified user
    3. Else fall back to legacy anonymous user lookup/creation

    Returns a dict with keys: id, token, tier, ip_address.
    """
    auth_header = request.headers.get("Authorization", "")

    if auth_header.startswith("Bearer "):
        token = auth_header[7:].strip()
        if token and not token.startswith("anon-"):
            # Check Better-Auth session table
            ba_session = await queries.get_ba_session_by_token(pool, token)
            if ba_session:
                user_id = ba_session["userId"]
                logger.info("Authenticated BA user user_id=%s tier=identified", user_id)
                return {
                    "id": user_id,
                    "token": token,
                    "tier": "identified",
                    "ip_address": request.client.host if request.client else "0.0.0.0",
                }

        # Legacy token lookup (existing anonymous/identified users)
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


async def get_authenticated_user(
    request: Request,
    pool: asyncpg.Pool = Depends(get_db_pool),
) -> dict:
    """Require an authenticated Better-Auth user. Returns 401 if not authenticated.

    Unlike get_current_user, this does NOT fall back to anonymous users.
    """
    auth_header = request.headers.get("Authorization", "")

    if not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Authentication required")

    token = auth_header[7:].strip()
    if not token or token.startswith("anon-"):
        raise HTTPException(status_code=401, detail="Authentication required")

    ba_session = await queries.get_ba_session_by_token(pool, token)
    if not ba_session:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    user_id = ba_session["userId"]
    logger.info("Authenticated BA user user_id=%s tier=identified", user_id)
    return {
        "id": user_id,
        "token": token,
        "tier": "identified",
        "ip_address": request.client.host if request.client else "0.0.0.0",
    }
