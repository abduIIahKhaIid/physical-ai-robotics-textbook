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

from rag.models import GroundedResponse, Query as RagQuery, QueryFilters as RagQueryFilters
from rag.retriever.query_engine import retrieve
from rag.response.grounding import apply_grounding_policy

from backend.services.retrieval_router import should_retrieve

logger = logging.getLogger(__name__)

_SYSTEM_PROMPT = (
    "You are RoboTutor, an AI teaching assistant for a Physical AI & Humanoid Robotics textbook. "
    "You help students understand concepts from the textbook. "
    "When textbook excerpts are provided, base your answers on them. "
    "Be professional, warm, and concise. Keep responses short unless the student asks for detail."
)

_CONVERSATIONAL_INSTRUCTION = (
    "The student sent a conversational message (greeting, thanks, follow-up, etc.) "
    "that does not require a textbook lookup. Respond naturally and briefly. "
    "For greetings, reply with a short friendly greeting and let the student know "
    "you can help with topics from the textbook. Do not lecture or repeat yourself."
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

    # 3.5 Route: decide if retrieval is needed
    needs_retrieval = True
    if mode == "normal" and not filters:
        try:
            needs_retrieval = await should_retrieve(
                message, history_messages, gemini_client, settings.llm_model,
                timeout_s=settings.router_timeout_s,
            )
        except Exception:
            logger.warning("Router classification failed, defaulting to retrieve")
            needs_retrieval = True

    # 4. Construct RAG query and retrieve (conditionally)
    if needs_retrieval:
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
            retrieval_result = await asyncio.wait_for(
                asyncio.to_thread(retrieve, rag_query, gemini_client),
                timeout=settings.retrieval_timeout_s,
            )
            grounded = apply_grounding_policy(retrieval_result, message)
        except Exception:
            logger.exception("Retrieval failed")
            from backend.models.errors import RetrievalError
            raise RetrievalError()
    else:
        grounded = GroundedResponse(
            context="",
            system_instruction=_CONVERSATIONAL_INSTRUCTION,
            citations=[],
            mode="normal",
            sufficient_context=True,
        )

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

        queue: asyncio.Queue[str | None] = asyncio.Queue()
        loop = asyncio.get_running_loop()

        async def _produce():
            await asyncio.to_thread(
                _stream_llm_to_queue, gemini_client, settings.llm_model, llm_contents, queue, loop
            )

        producer = asyncio.create_task(_produce())

        deadline = asyncio.get_event_loop().time() + settings.llm_timeout_s
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                producer.cancel()
                raise asyncio.TimeoutError()
            try:
                text = await asyncio.wait_for(queue.get(), timeout=remaining)
            except asyncio.TimeoutError:
                producer.cancel()
                raise
            if text is None:
                break
            full_response.append(text)
            yield {
                "event": "token",
                "data": json.dumps({"content": text}),
            }

        # Re-raise any exception from the producer thread
        if producer.done() and producer.exception():
            raise producer.exception()

    except asyncio.TimeoutError:
        logger.warning("LLM generation timed out after %ds", settings.llm_timeout_s)
        yield {
            "event": "error",
            "data": json.dumps({"code": "generation_timeout", "message": "Response generation timed out"}),
        }
        return
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


def _stream_llm_to_queue(
    gemini_client: Any,
    model: str,
    contents: list[str],
    queue: asyncio.Queue,
    loop: asyncio.AbstractEventLoop,
) -> None:
    """Run synchronous LLM streaming in a thread, pushing chunks to a queue.

    Uses call_soon_threadsafe because asyncio.Queue is not thread-safe.
    """
    response = gemini_client.models.generate_content_stream(
        model=model, contents=contents,
    )
    for chunk in response:
        if chunk.text:
            loop.call_soon_threadsafe(queue.put_nowait, chunk.text)
    loop.call_soon_threadsafe(queue.put_nowait, None)  # sentinel: stream finished


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
