"""Integration test for the chat endpoint (POST /api/v1/chat)."""

from __future__ import annotations

import json
import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
import httpx

from backend.config import BackendSettings


def _make_mock_app():
    """Create a FastAPI test app with mocked dependencies."""
    settings = BackendSettings(
        database_url="postgresql://test",
        gemini_api_key="test-key",
        llm_model="gemini-2.0-flash",
    )

    mock_pool = AsyncMock()
    session_id = uuid.uuid4()
    user_msg_id = uuid.uuid4()
    asst_msg_id = uuid.uuid4()
    user_id = uuid.uuid4()

    mock_pool.fetchrow = AsyncMock(side_effect=[
        None,  # get_user_by_token (anonymous lookup)
        {"id": user_id},  # create_user
        {"id": session_id},  # create_session
        {"id": user_msg_id},  # create_message (user)
        {"id": asst_msg_id},  # create_message (assistant)
    ])
    mock_pool.fetch = AsyncMock(return_value=[])
    mock_pool.execute = AsyncMock()

    mock_gemini = MagicMock()
    chunk1 = MagicMock()
    chunk1.text = "Physical AI "
    chunk2 = MagicMock()
    chunk2.text = "is amazing."
    mock_gemini.models.generate_content_stream.return_value = [chunk1, chunk2]

    from fastapi import FastAPI
    from backend.routers import chat, sessions, health
    from backend.models.errors import ChatBackendError, chat_backend_error_handler, generic_error_handler
    from backend.services.rate_limiter import SlidingWindowRateLimiter

    app = FastAPI()
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.add_exception_handler(Exception, generic_error_handler)
    app.include_router(chat.router, prefix="/api/v1")
    app.include_router(sessions.router, prefix="/api/v1")
    app.include_router(health.router, prefix="/api/v1")

    app.state.db_pool = mock_pool
    app.state.gemini_client = mock_gemini
    app.state.rate_limiter = SlidingWindowRateLimiter()
    app.state.settings = settings

    return app, mock_pool, mock_gemini, session_id


def _parse_sse_events(body: str) -> list[dict]:
    """Parse SSE events from response body text.

    Handles sse_starlette output format: event: name\\ndata: json\\n\\n
    """
    events = []
    current_event = None
    current_data = None

    for line in body.split("\n"):
        stripped = line.strip()
        if stripped.startswith("event:"):
            current_event = stripped[6:].strip()
        elif stripped.startswith("data:"):
            data_str = stripped[5:].strip()
            try:
                current_data = json.loads(data_str)
            except json.JSONDecodeError:
                current_data = data_str
        elif stripped == "" and current_event is not None:
            if current_data is not None:
                events.append({"event": current_event, "data": current_data})
            current_event = None
            current_data = None

    # Handle case where last event doesn't end with empty line
    if current_event is not None and current_data is not None:
        events.append({"event": current_event, "data": current_data})

    return events


class TestChatEndpoint:
    """Integration tests for POST /api/v1/chat."""

    @pytest.mark.asyncio
    async def test_chat_returns_sse_stream(self):
        """POST /api/v1/chat returns text/event-stream with token and done events."""
        app, mock_pool, mock_gemini, session_id = _make_mock_app()

        mock_retrieval = MagicMock()
        mock_retrieval.chunks = []
        mock_retrieval.mode = "normal"

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer."
        mock_grounded.context = ""
        mock_grounded.citations = []

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval), \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app),
                base_url="http://test",
            ) as client:
                response = await client.post(
                    "/api/v1/chat",
                    json={"message": "What is Physical AI?"},
                )

                assert response.status_code == 200
                assert "text/event-stream" in response.headers.get("content-type", "")

                body = response.text
                events = _parse_sse_events(body)

                token_events = [e for e in events if e["event"] == "token"]
                done_events = [e for e in events if e["event"] == "done"]

                assert len(token_events) >= 1
                assert len(done_events) == 1

                done_data = done_events[0]["data"]
                assert "message_id" in done_data
                assert "session_id" in done_data
                assert "citations" in done_data

    @pytest.mark.asyncio
    async def test_chat_rejects_empty_message(self):
        """POST with empty message returns 422."""
        app, _, _, _ = _make_mock_app()

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app),
            base_url="http://test",
        ) as client:
            response = await client.post(
                "/api/v1/chat",
                json={"message": ""},
            )
            assert response.status_code == 422

    @pytest.mark.asyncio
    async def test_chat_selected_text_mode(self):
        """POST with selected_text_only mode works correctly."""
        app, mock_pool, mock_gemini, session_id = _make_mock_app()

        user_id = uuid.uuid4()
        mock_pool.fetchrow = AsyncMock(side_effect=[
            None,
            {"id": user_id},
            {"id": session_id},
            {"id": uuid.uuid4()},
            {"id": uuid.uuid4()},
        ])

        mock_retrieval = MagicMock()
        mock_retrieval.chunks = []
        mock_retrieval.mode = "selected_text_only"

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer ONLY from selected text."
        mock_grounded.context = "Physical AI systems operate in real-time."
        mock_grounded.citations = []

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval), \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):

            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app),
                base_url="http://test",
            ) as client:
                response = await client.post(
                    "/api/v1/chat",
                    json={
                        "message": "Explain this",
                        "mode": "selected_text_only",
                        "selected_text": "Physical AI systems operate in real-time.",
                    },
                )

                assert response.status_code == 200
                events = _parse_sse_events(response.text)
                done_events = [e for e in events if e["event"] == "done"]
                assert len(done_events) == 1
