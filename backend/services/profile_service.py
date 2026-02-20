"""Business logic for user onboarding profiles."""

from __future__ import annotations

import asyncpg

from backend.db import queries


async def create_or_update_profile(
    pool: asyncpg.Pool,
    user_id: str,
    software_level: str = "beginner",
    programming_languages: str = "",
    hardware_level: str = "none",
    available_hardware: list[str] | None = None,
    learning_goal: str = "",
    preferred_pace: str = "self_paced",
) -> dict:
    """Create or fully replace a user profile (sets onboarding_completed=true)."""
    row = await queries.upsert_profile(
        pool,
        user_id=user_id,
        software_level=software_level,
        programming_languages=programming_languages,
        hardware_level=hardware_level,
        available_hardware=available_hardware,
        learning_goal=learning_goal,
        preferred_pace=preferred_pace,
    )
    return _row_to_dict(row)


async def get_user_profile(pool: asyncpg.Pool, user_id: str) -> dict:
    """Get a user profile, returning defaults if no profile exists."""
    row = await queries.get_profile_by_user_id(pool, user_id)
    if row:
        return _row_to_dict(row)
    # Return default profile (not yet completed onboarding)
    return {
        "user_id": user_id,
        "software_level": "beginner",
        "programming_languages": "",
        "hardware_level": "none",
        "available_hardware": [],
        "learning_goal": "",
        "preferred_pace": "self_paced",
        "onboarding_completed": False,
        "created_at": None,
        "updated_at": None,
    }


def _row_to_dict(row: asyncpg.Record) -> dict:
    """Convert an asyncpg Record to a plain dict."""
    d = dict(row)
    # Convert available_hardware from PG array to Python list
    if "available_hardware" in d and d["available_hardware"] is None:
        d["available_hardware"] = []
    return d
