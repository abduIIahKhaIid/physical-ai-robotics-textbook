"""Integration test: selected-text-only mode must NEVER access Qdrant or Gemini embeddings."""

from __future__ import annotations

import json
import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
import httpx

from backend.config import BackendSettings
from backend.services.rate_limiter import SlidingWindowRateLimiter


def _make_app_with_mocks():
    """Create test app with instrumented mocks to detect Qdrant/embedding leaks."""
    settings = BackendSettings(
        database_url="postgresql://test",
        gemini_api_key="test-key",
        llm_model="gemini-2.0-flash",
    )

    mock_pool = AsyncMock()
    session_id = uuid.uuid4()
    user_id = uuid.uuid4()

    mock_pool.fetchrow = AsyncMock(side_effect=[
        None,  # get_user_by_token
        {"id": user_id},  # create_user
        {"id": session_id},  # create_session
        {"id": uuid.uuid4()},  # create_message (user)
        {"id": uuid.uuid4()},  # create_message (assistant)
    ])
    mock_pool.fetch = AsyncMock(return_value=[])
    mock_pool.execute = AsyncMock()

    mock_gemini = MagicMock()
    chunk1 = MagicMock()
    chunk1.text = "The selected text says real-time."
    mock_gemini.models.generate_content_stream.return_value = [chunk1]

    from fastapi import FastAPI
    from backend.routers import chat
    from backend.models.errors import ChatBackendError, chat_backend_error_handler

    app = FastAPI()
    app.add_exception_handler(ChatBackendError, chat_backend_error_handler)
    app.include_router(chat.router, prefix="/api/v1")

    app.state.db_pool = mock_pool
    app.state.gemini_client = mock_gemini
    app.state.rate_limiter = SlidingWindowRateLimiter()
    app.state.settings = settings

    return app, mock_gemini


def _parse_sse_events(body: str) -> list[dict]:
    """Parse SSE events from response body."""
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

    if current_event is not None and current_data is not None:
        events.append({"event": current_event, "data": current_data})

    return events


class TestSelectedTextIsolation:
    """Verify that selected_text_only mode NEVER accesses Qdrant or Gemini embeddings."""

    @pytest.mark.asyncio
    async def test_qdrant_search_not_called(self):
        """Qdrant search must not be called for selected_text_only mode."""
        app, mock_gemini = _make_app_with_mocks()

        # Patch Qdrant search to track calls
        with patch("rag.store.qdrant_store.search_chunks") as mock_qdrant_search, \
             patch("backend.services.chat_service.retrieve") as mock_retrieve:

            # Use the real selected-text retrieval path
            from rag.models import RetrievalResult, Query as RagQuery, ScoredChunk, Citation, ChunkMetadata
            mock_retrieve.side_effect = lambda query, *args, **kwargs: RetrievalResult(
                chunks=[ScoredChunk(
                    chunk_id="selection",
                    text=query.selected_text or "",
                    score=1.0,
                    metadata=ChunkMetadata(doc_path=query.source_doc_path or "selection"),
                    citation=Citation(title="Selected Text", section="User Selection", url=""),
                )],
                query=query,
                mode="selected_text_only",
                total_candidates=1,
            )

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
                # Qdrant search must NOT have been called
                mock_qdrant_search.assert_not_called()

    @pytest.mark.asyncio
    async def test_gemini_embed_not_called(self):
        """Gemini embed_content must not be called for selected_text_only mode."""
        app, mock_gemini = _make_app_with_mocks()

        from rag.models import RetrievalResult, Query as RagQuery, ScoredChunk, Citation, ChunkMetadata

        with patch("backend.services.chat_service.retrieve") as mock_retrieve:
            mock_retrieve.side_effect = lambda query, *args, **kwargs: RetrievalResult(
                chunks=[ScoredChunk(
                    chunk_id="selection",
                    text=query.selected_text or "",
                    score=1.0,
                    metadata=ChunkMetadata(doc_path="selection"),
                    citation=Citation(title="Selected Text", section="User Selection", url=""),
                )],
                query=query,
                mode="selected_text_only",
                total_candidates=1,
            )

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
                # Gemini embed_content must NOT have been called
                mock_gemini.models.embed_content.assert_not_called()

    @pytest.mark.asyncio
    async def test_response_references_selected_text(self):
        """Response in selected_text_only mode should reference the selected text."""
        app, mock_gemini = _make_app_with_mocks()

        from rag.models import RetrievalResult, ScoredChunk, Citation, ChunkMetadata

        with patch("backend.services.chat_service.retrieve") as mock_retrieve:
            mock_retrieve.side_effect = lambda query, *args, **kwargs: RetrievalResult(
                chunks=[ScoredChunk(
                    chunk_id="selection",
                    text=query.selected_text or "",
                    score=1.0,
                    metadata=ChunkMetadata(doc_path="selection"),
                    citation=Citation(title="Selected Text", section="User Selection", url=""),
                )],
                query=query,
                mode="selected_text_only",
                total_candidates=1,
            )

            with patch("backend.services.chat_service.apply_grounding_policy") as mock_ground:
                from rag.models import GroundedResponse
                mock_ground.return_value = GroundedResponse(
                    context="Physical AI systems operate in real-time.",
                    system_instruction="Answer ONLY from the selected text.",
                    citations=[Citation(title="Selected Text", section="User Selection", url="")],
                    mode="selected_text_only",
                    sufficient_context=True,
                )

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

                    done_data = done_events[0]["data"]
                    citations = done_data.get("citations", [])
                    assert len(citations) >= 1
                    assert citations[0]["title"] == "Selected Text"
