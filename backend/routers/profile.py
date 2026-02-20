"""Profile endpoints: GET/POST/PATCH /api/v1/profile."""

from __future__ import annotations

import asyncpg
from fastapi import APIRouter, Depends
from fastapi.responses import JSONResponse

from backend.db import queries
from backend.dependencies import get_authenticated_user, get_db_pool
from backend.models.errors import ProfileNotFoundError
from backend.models.profile import ProfileCreate, ProfileResponse, ProfileUpdate
from backend.services import profile_service

router = APIRouter()


@router.get("/profile", response_model=ProfileResponse)
async def get_profile(
    user: dict = Depends(get_authenticated_user),
    pool: asyncpg.Pool = Depends(get_db_pool),
) -> ProfileResponse:
    """Get the authenticated user's onboarding profile."""
    data = await profile_service.get_user_profile(pool, user["id"])
    return ProfileResponse(**data)


@router.post("/profile", response_model=ProfileResponse, status_code=201)
async def create_profile(
    body: ProfileCreate,
    user: dict = Depends(get_authenticated_user),
    pool: asyncpg.Pool = Depends(get_db_pool),
) -> JSONResponse:
    """Create or replace the user's onboarding profile."""
    data = await profile_service.create_or_update_profile(
        pool,
        user_id=user["id"],
        software_level=body.software_level,
        programming_languages=body.programming_languages,
        hardware_level=body.hardware_level,
        available_hardware=list(body.available_hardware),
        learning_goal=body.learning_goal,
        preferred_pace=body.preferred_pace,
    )
    return JSONResponse(
        status_code=201,
        content=ProfileResponse(**data).model_dump(mode="json"),
    )


@router.patch("/profile", response_model=ProfileResponse)
async def update_profile(
    body: ProfileUpdate,
    user: dict = Depends(get_authenticated_user),
    pool: asyncpg.Pool = Depends(get_db_pool),
) -> ProfileResponse:
    """Partially update the user's onboarding profile."""
    existing = await queries.get_profile_by_user_id(pool, user["id"])
    if not existing:
        raise ProfileNotFoundError()

    update_fields = body.model_dump(exclude_none=True)
    if not update_fields:
        return ProfileResponse(**dict(existing))

    row = await queries.update_profile(pool, user["id"], **update_fields)
    if not row:
        raise ProfileNotFoundError()
    return ProfileResponse(**dict(row))
