"""Unit tests for chat service orchestration."""

from __future__ import annotations

import json
import time
import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from backend.config import BackendSettings
from backend.services.chat_service import stream_chat_response, _auto_title, _build_llm_contents


class TestAutoTitle:
    """Test session title generation."""

    def test_short_message(self):
        assert _auto_title("What is ROS?") == "What is ROS?"

    def test_long_message_truncated_at_word(self):
        msg = "This is a very long question about robotics that exceeds one hundred characters and should be truncated properly at a reasonable boundary"
        title = _auto_title(msg)
        assert len(title) <= 104  # 100 + "..."
        assert title.endswith("...")

    def test_exact_100_chars(self):
        msg = "x" * 100
        assert _auto_title(msg) == msg


class TestBuildLLMContents:
    """Test LLM prompt construction."""

    def test_basic_prompt(self):
        result = _build_llm_contents(
            "Answer from excerpts.",
            "Some context here.",
            [],
            "What is Physical AI?",
        )
        assert len(result) == 1
        assert "Answer from excerpts." in result[0]
        assert "Some context here." in result[0]
        assert "What is Physical AI?" in result[0]

    def test_with_history(self):
        history = [
            {"role": "user", "content": "Hello"},
            {"role": "assistant", "content": "Hi there!"},
        ]
        result = _build_llm_contents(
            "System instruction",
            "Context",
            history,
            "Follow-up question",
        )
        assert "Student: Hello" in result[0]
        assert "Assistant: Hi there!" in result[0]
        assert "Student: Follow-up question" in result[0]

    def test_empty_context(self):
        result = _build_llm_contents(
            "System instruction", "", [], "Question"
        )
        assert "Textbook Context" not in result[0]


class TestStreamChatResponse:
    """Test chat response streaming with mocked dependencies."""

    @pytest.fixture
    def mock_pool(self):
        pool = AsyncMock()
        return pool

    @pytest.fixture
    def mock_gemini_client(self):
        client = MagicMock()
        chunk1 = MagicMock()
        chunk1.text = "Hello "
        chunk2 = MagicMock()
        chunk2.text = "world"
        client.models.generate_content_stream.return_value = [chunk1, chunk2]
        return client

    @pytest.fixture
    def mock_user(self):
        return {
            "id": uuid.uuid4(),
            "token": "test-token",
            "tier": "anonymous",
            "ip_address": "127.0.0.1",
        }

    @pytest.fixture
    def settings(self):
        return BackendSettings(
            database_url="postgresql://test",
            gemini_api_key="test-key",
            llm_model="gemini-2.0-flash",
        )

    @pytest.mark.asyncio
    async def test_normal_mode_calls_retrieve(
        self, mock_pool, mock_gemini_client, mock_user, settings
    ):
        """Test that normal mode calls retrieve with correct Query."""
        session_id = uuid.uuid4()

        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": session_id, "user_id": mock_user["id"], "status": "active"},
            {"id": uuid.uuid4()},
            {"id": uuid.uuid4()},
        ])
        mock_pool.fetch = AsyncMock(return_value=[])
        mock_pool.execute = AsyncMock()

        mock_retrieval_result = MagicMock()
        mock_retrieval_result.chunks = []
        mock_retrieval_result.mode = "normal"

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer from excerpts."
        mock_grounded.context = "Some context"
        mock_grounded.citations = []
        mock_grounded.sufficient_context = True

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval_result) as mock_retrieve, \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):
            events = []
            async for event in stream_chat_response(
                message="What is Physical AI?",
                session_id=session_id,
                mode="normal",
                selected_text=None,
                source_doc_path=None,
                source_section=None,
                filters=None,
                pool=mock_pool,
                gemini_client=mock_gemini_client,
                user=mock_user,
                settings=settings,
            ):
                events.append(event)

            mock_retrieve.assert_called_once()
            call_args = mock_retrieve.call_args
            query = call_args[0][0]
            assert query.mode == "normal"
            assert query.question == "What is Physical AI?"

    @pytest.mark.asyncio
    async def test_selected_text_mode_passes_selected_text(
        self, mock_pool, mock_gemini_client, mock_user, settings
    ):
        """Test selected-text-only mode passes selected_text to Query."""
        session_id = uuid.uuid4()

        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": session_id, "user_id": mock_user["id"], "status": "active"},
            {"id": uuid.uuid4()},
            {"id": uuid.uuid4()},
        ])
        mock_pool.fetch = AsyncMock(return_value=[])
        mock_pool.execute = AsyncMock()

        mock_retrieval_result = MagicMock()
        mock_retrieval_result.chunks = []
        mock_retrieval_result.mode = "selected_text_only"

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer from selection."
        mock_grounded.context = "Selected text context"
        mock_grounded.citations = []
        mock_grounded.sufficient_context = True

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval_result) as mock_retrieve, \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):
            events = []
            async for event in stream_chat_response(
                message="Explain this",
                session_id=session_id,
                mode="selected_text_only",
                selected_text="Physical AI operates in real-time.",
                source_doc_path="/docs/module-1",
                source_section="Intro",
                filters=None,
                pool=mock_pool,
                gemini_client=mock_gemini_client,
                user=mock_user,
                settings=settings,
            ):
                events.append(event)

            mock_retrieve.assert_called_once()
            query = mock_retrieve.call_args[0][0]
            assert query.mode == "selected_text_only"
            assert query.selected_text == "Physical AI operates in real-time."

    @pytest.mark.asyncio
    async def test_stream_yields_token_and_done_events(
        self, mock_pool, mock_gemini_client, mock_user, settings
    ):
        """Test that stream yields token events and a done event."""
        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": uuid.uuid4()},
            {"id": uuid.uuid4()},
            {"id": uuid.uuid4()},
        ])
        mock_pool.fetch = AsyncMock(return_value=[])
        mock_pool.execute = AsyncMock()

        mock_retrieval_result = MagicMock()
        mock_retrieval_result.chunks = []
        mock_retrieval_result.mode = "normal"

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer from excerpts."
        mock_grounded.context = ""
        mock_grounded.citations = []

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval_result), \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):
            events = []
            async for event in stream_chat_response(
                message="Hello",
                session_id=None,
                mode="normal",
                selected_text=None,
                source_doc_path=None,
                source_section=None,
                filters=None,
                pool=mock_pool,
                gemini_client=mock_gemini_client,
                user=mock_user,
                settings=settings,
            ):
                events.append(event)

            token_events = [e for e in events if e["event"] == "token"]
            done_events = [e for e in events if e["event"] == "done"]

            assert len(token_events) == 2  # "Hello " and "world"
            assert len(done_events) == 1

            done_data = json.loads(done_events[0]["data"])
            assert "session_id" in done_data
            assert "message_id" in done_data
            assert "citations" in done_data

    @pytest.mark.asyncio
    async def test_conversation_history_included(
        self, mock_pool, mock_gemini_client, mock_user, settings
    ):
        """Test that conversation history is loaded and passed to LLM."""
        session_id = uuid.uuid4()
        user_msg_id = uuid.uuid4()

        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": session_id, "user_id": mock_user["id"], "status": "active"},
            {"id": user_msg_id},
            {"id": uuid.uuid4()},
        ])
        mock_pool.fetch = AsyncMock(return_value=[
            {"id": uuid.uuid4(), "role": "user", "content": "Previous question", "retrieval_mode": "normal", "created_at": "2026-01-01"},
            {"id": uuid.uuid4(), "role": "assistant", "content": "Previous answer", "retrieval_mode": "normal", "created_at": "2026-01-01"},
            {"id": user_msg_id, "role": "user", "content": "Follow up", "retrieval_mode": "normal", "created_at": "2026-01-02"},
        ])
        mock_pool.execute = AsyncMock()

        mock_retrieval_result = MagicMock()
        mock_retrieval_result.chunks = []
        mock_retrieval_result.mode = "normal"

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer."
        mock_grounded.context = ""
        mock_grounded.citations = []

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval_result), \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):
            events = []
            async for event in stream_chat_response(
                message="Follow up",
                session_id=session_id,
                mode="normal",
                selected_text=None,
                source_doc_path=None,
                source_section=None,
                filters=None,
                pool=mock_pool,
                gemini_client=mock_gemini_client,
                user=mock_user,
                settings=settings,
            ):
                events.append(event)

            call_args = mock_gemini_client.models.generate_content_stream.call_args
            contents = call_args[1]["contents"] if "contents" in call_args[1] else call_args[0][0] if call_args[0] else call_args[1].get("contents")
            prompt_text = contents[0] if isinstance(contents, list) else contents
            assert "Previous question" in prompt_text
            assert "Previous answer" in prompt_text

    @pytest.mark.asyncio
    async def test_llm_timeout_yields_error_event(
        self, mock_pool, mock_user,
    ):
        """When LLM streaming times out, yield a timeout error event."""
        settings = BackendSettings(
            database_url="postgresql://test",
            gemini_api_key="test-key",
            llm_model="gemini-2.0-flash",
            llm_timeout_s=0,  # immediate timeout
        )

        client = MagicMock()

        def slow_stream(**kwargs):
            time.sleep(2)
            return []

        client.models.generate_content_stream.side_effect = slow_stream

        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": uuid.uuid4()},
            {"id": uuid.uuid4()},
        ])
        mock_pool.fetch = AsyncMock(return_value=[])
        mock_pool.execute = AsyncMock()

        mock_grounded = MagicMock()
        mock_grounded.system_instruction = "Answer."
        mock_grounded.context = ""
        mock_grounded.citations = []

        mock_retrieval_result = MagicMock()
        mock_retrieval_result.chunks = []
        mock_retrieval_result.mode = "normal"

        with patch("backend.services.chat_service.retrieve", return_value=mock_retrieval_result), \
             patch("backend.services.chat_service.apply_grounding_policy", return_value=mock_grounded):
            events = []
            async for event in stream_chat_response(
                message="Hello",
                session_id=None,
                mode="normal",
                selected_text=None,
                source_doc_path=None,
                source_section=None,
                filters=None,
                pool=mock_pool,
                gemini_client=client,
                user=mock_user,
                settings=settings,
            ):
                events.append(event)

            error_events = [e for e in events if e["event"] == "error"]
            assert len(error_events) == 1
            error_data = json.loads(error_events[0]["data"])
            assert error_data["code"] == "generation_timeout"

    @pytest.mark.asyncio
    async def test_retrieval_timeout_raises(
        self, mock_pool, mock_gemini_client, mock_user,
    ):
        """When retrieval times out, a RetrievalError is raised."""
        settings = BackendSettings(
            database_url="postgresql://test",
            gemini_api_key="test-key",
            llm_model="gemini-2.0-flash",
            retrieval_timeout_s=0,  # immediate timeout
        )

        session_id = uuid.uuid4()
        mock_pool.fetchrow = AsyncMock(side_effect=[
            {"id": session_id, "user_id": mock_user["id"], "status": "active"},
            {"id": uuid.uuid4()},
        ])
        mock_pool.fetch = AsyncMock(return_value=[])
        mock_pool.execute = AsyncMock()

        def slow_retrieve(*args, **kwargs):
            time.sleep(2)

        with patch("backend.services.chat_service.retrieve", side_effect=slow_retrieve), \
             patch("backend.services.chat_service.should_retrieve", return_value=True):
            from backend.models.errors import RetrievalError
            with pytest.raises(RetrievalError):
                events = []
                async for event in stream_chat_response(
                    message="What is ROS2?",
                    session_id=session_id,
                    mode="normal",
                    selected_text=None,
                    source_doc_path=None,
                    source_section=None,
                    filters=None,
                    pool=mock_pool,
                    gemini_client=mock_gemini_client,
                    user=mock_user,
                    settings=settings,
                ):
                    events.append(event)
