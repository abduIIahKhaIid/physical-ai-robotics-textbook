"""Chat orchestration service: retrieve -> ground -> stream LLM response."""

from __future__ import annotations

import asyncio
import json
import logging
import time
import uuid
from typing import Any, AsyncGenerator

from backend.config import BackendSettings
from backend.db import queries

from rag.models import Query as RagQuery, QueryFilters as RagQueryFilters
from rag.retriever.query_engine import retrieve
from rag.response.grounding import apply_grounding_policy

logger = logging.getLogger(__name__)

_SYSTEM_PROMPT = (
    "You are RoboTutor, an AI teaching assistant for a Physical AI & Humanoid Robotics textbook. "
    "You help students understand concepts from the textbook. "
    "Always base your answers on the provided textbook excerpts. "
    "Be clear, concise, and educational."
)


async def stream_chat_response(
    *,
    message: str,
    session_id: uuid.UUID | None,
    mode: str,
    selected_text: str | None,
    source_doc_path: str | None,
    source_section: str | None,
    filters: dict | None,
    pool: Any,
    gemini_client: Any,
    user: dict,
    settings: BackendSettings,
) -> AsyncGenerator[dict, None]:
    """Orchestrate chat: persist, retrieve, ground, stream, persist.

    Yields dicts with 'event' and 'data' keys for SSE formatting.
    """
    start_time = time.monotonic()
    db_warning = False
    user_message_id = None
    assistant_message_id = None

    # 1. Resolve or create session
    if session_id is None:
        try:
            title = _auto_title(message)
            session_id = await queries.create_session(pool, user["id"], title)
        except Exception:
            logger.exception("Failed to create session")
            db_warning = True
            session_id = uuid.uuid4()  # fallback ephemeral
    else:
        try:
            session = await queries.get_session(pool, session_id)
            if session is None:
                from backend.models.errors import SessionNotFoundError
                raise SessionNotFoundError(f"Session {session_id} not found")
            if session["user_id"] != user["id"]:
                from backend.models.errors import SessionNotFoundError
                raise SessionNotFoundError(f"Session {session_id} not found")
        except Exception as exc:
            from backend.models.errors import SessionNotFoundError
            if isinstance(exc, SessionNotFoundError):
                raise
            logger.exception("Failed to verify session")
            db_warning = True

    # 2. Persist user message
    try:
        user_message_id = await queries.create_message(
            pool, session_id, "user", message,
            retrieval_mode=mode,
            selected_text=selected_text,
        )
    except Exception:
        logger.exception("Failed to persist user message")
        db_warning = True

    # 3. Load conversation history
    history_messages = []
    try:
        rows = await queries.get_messages(pool, session_id)
        for row in rows:
            if user_message_id and row["id"] == user_message_id:
                continue
            history_messages.append({
                "role": row["role"],
                "content": row["content"],
            })
        max_history = settings.max_history_messages
        if len(history_messages) > max_history:
            history_messages = history_messages[-max_history:]
    except Exception:
        logger.exception("Failed to load conversation history")

    # 4. Construct RAG query and retrieve
    rag_filters = None
    if filters:
        rag_filters = RagQueryFilters(**filters)

    rag_query = RagQuery(
        question=message,
        mode=mode,
        selected_text=selected_text,
        source_doc_path=source_doc_path,
        source_section=source_section,
        filters=rag_filters,
    )

    try:
        retrieval_result = await asyncio.to_thread(
            retrieve, rag_query, gemini_client
        )
        grounded = apply_grounding_policy(retrieval_result, message)
    except Exception:
        logger.exception("Retrieval failed")
        from backend.models.errors import RetrievalError
        raise RetrievalError()

    # 5. Build LLM prompt
    llm_contents = _build_llm_contents(
        grounded.system_instruction,
        grounded.context,
        history_messages,
        message,
    )

    # 6. Stream LLM response
    full_response = []
    citations_data = []
    for c in grounded.citations:
        citations_data.append({
            "title": c.title,
            "section": c.section,
            "url": c.url,
            "module": c.module,
            "chapter": c.chapter,
        })

    try:
        if gemini_client is None:
            from backend.models.errors import GenerationError
            raise GenerationError("LLM client not configured")

        response = gemini_client.models.generate_content_stream(
            model=settings.llm_model,
            contents=llm_contents,
        )

        for chunk in response:
            if chunk.text:
                full_response.append(chunk.text)
                yield {
                    "event": "token",
                    "data": json.dumps({"content": chunk.text}),
                }

    except asyncio.CancelledError:
        logger.warning("Client disconnected mid-stream")
    except Exception as exc:
        from backend.models.errors import GenerationError
        if isinstance(exc, GenerationError):
            raise
        logger.exception("LLM generation failed")
        yield {
            "event": "error",
            "data": json.dumps({"code": "generation_failed", "message": "Response generation failed"}),
        }
        return

    # 7. Persist assistant message
    full_text = "".join(full_response)
    latency_ms = int((time.monotonic() - start_time) * 1000)
    chunk_count = len(grounded.citations)

    try:
        assistant_message_id = await queries.create_message(
            pool, session_id, "assistant", full_text,
            retrieval_mode=mode,
            chunk_count=chunk_count,
            latency_ms=latency_ms,
        )
        await queries.update_session_activity(pool, session_id)
    except Exception:
        logger.exception("Failed to persist assistant message")
        db_warning = True

    # 8. Yield done event
    done_data = {
        "message_id": str(assistant_message_id or uuid.uuid4()),
        "session_id": str(session_id),
        "citations": citations_data,
    }
    if db_warning:
        done_data["persistence_warning"] = True

    yield {
        "event": "done",
        "data": json.dumps(done_data),
    }


def _auto_title(message: str) -> str:
    """Generate session title from first user message."""
    title = message[:100]
    if len(message) > 100:
        last_space = title.rfind(" ")
        if last_space > 20:
            title = title[:last_space]
        title += "..."
    return title


def _build_llm_contents(
    system_instruction: str,
    context: str,
    history: list[dict],
    current_question: str,
) -> list[str]:
    """Build the content list for Gemini generate_content_stream."""
    parts = [f"{_SYSTEM_PROMPT}\n\n{system_instruction}"]

    if context:
        parts.append(f"\n\nTextbook Context:\n{context}")

    if history:
        parts.append("\n\nConversation History:")
        for msg in history:
            role_label = "Student" if msg["role"] == "user" else "Assistant"
            parts.append(f"\n{role_label}: {msg['content']}")

    parts.append(f"\n\nStudent: {current_question}\n\nAssistant:")

    return ["".join(parts)]
