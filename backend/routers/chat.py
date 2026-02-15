"""Chat endpoint: POST /api/v1/chat with SSE streaming."""

from __future__ import annotations

import logging

import asyncpg
from fastapi import APIRouter, Depends, Request
from sse_starlette.sse import EventSourceResponse

from backend.config import BackendSettings
from backend.dependencies import get_current_user, get_db_pool, get_gemini_client
from backend.models.api_models import ChatRequest
from backend.models.errors import RateLimitError
from backend.services import chat_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/chat")
async def send_message(
    body: ChatRequest,
    request: Request,
    pool: asyncpg.Pool = Depends(get_db_pool),
    gemini_client=Depends(get_gemini_client),
    user: dict = Depends(get_current_user),
):
    """Send a chat message and receive a streamed SSE response."""
    settings: BackendSettings = request.app.state.settings

    # Rate limit check
    rate_limiter = request.app.state.rate_limiter
    identifier = str(user.get("id", user.get("ip_address", "unknown")))
    limit = (
        settings.rate_limit_identified
        if user.get("tier") == "identified"
        else settings.rate_limit_anon
    )

    allowed, retry_after = await rate_limiter.check(identifier, limit)
    if not allowed:
        raise RateLimitError(retry_after=retry_after)

    # Build filters dict from Pydantic model
    filters_dict = None
    if body.filters:
        filters_dict = body.filters.model_dump(exclude_none=True)

    generator = chat_service.stream_chat_response(
        message=body.message,
        session_id=body.session_id,
        mode=body.mode,
        selected_text=body.selected_text,
        source_doc_path=body.source_doc_path,
        source_section=body.source_section,
        filters=filters_dict,
        pool=pool,
        gemini_client=gemini_client,
        user=user,
        settings=settings,
    )

    return EventSourceResponse(generator, media_type="text/event-stream")
