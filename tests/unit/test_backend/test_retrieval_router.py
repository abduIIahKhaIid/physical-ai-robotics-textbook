"""Unit tests for the retrieval router classifier."""

from __future__ import annotations

import asyncio
import time
from unittest.mock import MagicMock

import pytest

from backend.services.retrieval_router import should_retrieve, _format_history


class TestFormatHistory:
    """Test history formatting for the classification prompt."""

    def test_empty_history(self):
        assert _format_history([]) == "(no prior messages)"

    def test_single_turn(self):
        history = [
            {"role": "user", "content": "Hello"},
            {"role": "assistant", "content": "Hi there!"},
        ]
        result = _format_history(history)
        assert "Student: Hello" in result
        assert "Assistant: Hi there!" in result

    def test_truncates_to_last_two_turns(self):
        history = [
            {"role": "user", "content": "First"},
            {"role": "assistant", "content": "Reply 1"},
            {"role": "user", "content": "Second"},
            {"role": "assistant", "content": "Reply 2"},
            {"role": "user", "content": "Third"},
            {"role": "assistant", "content": "Reply 3"},
        ]
        # max_turns=2 keeps last 4 messages (2 turns x 2 messages each)
        result = _format_history(history, max_turns=2)
        assert "First" not in result
        assert "Reply 1" not in result
        assert "Second" in result
        assert "Reply 2" in result
        assert "Third" in result
        assert "Reply 3" in result


class TestShouldRetrieve:
    """Test the LLM-based classification function."""

    def _make_client(self, response_text: str) -> MagicMock:
        """Create a mock gemini_client returning the given text."""
        client = MagicMock()
        mock_response = MagicMock()
        mock_response.text = response_text
        client.models.generate_content.return_value = mock_response
        return client

    @pytest.mark.asyncio
    async def test_greeting_returns_false(self):
        """'SKIP' response means no retrieval needed."""
        client = self._make_client("SKIP")
        result = await should_retrieve("hi", [], client, "gemini-2.0-flash")
        assert result is False
        client.models.generate_content.assert_called_once()

    @pytest.mark.asyncio
    async def test_content_question_returns_true(self):
        """'RETRIEVE' response means retrieval is needed."""
        client = self._make_client("RETRIEVE")
        result = await should_retrieve(
            "What is ROS2?", [], client, "gemini-2.0-flash"
        )
        assert result is True

    @pytest.mark.asyncio
    async def test_skip_with_extra_text_still_skips(self):
        """Parser reads only the first word."""
        client = self._make_client("SKIP - this is a greeting")
        result = await should_retrieve("thanks!", [], client, "gemini-2.0-flash")
        assert result is False

    @pytest.mark.asyncio
    async def test_retrieve_with_extra_text(self):
        """Parser reads only the first word."""
        client = self._make_client("RETRIEVE - content question")
        result = await should_retrieve(
            "Explain kinematics", [], client, "gemini-2.0-flash"
        )
        assert result is True

    @pytest.mark.asyncio
    async def test_exception_defaults_to_retrieve(self):
        """On any error, default to True (safe fallback)."""
        client = MagicMock()
        client.models.generate_content.side_effect = RuntimeError("API down")
        result = await should_retrieve("hi", [], client, "gemini-2.0-flash")
        assert result is True

    @pytest.mark.asyncio
    async def test_empty_response_defaults_to_retrieve(self):
        """Empty LLM response defaults to True."""
        client = self._make_client("")
        result = await should_retrieve("hi", [], client, "gemini-2.0-flash")
        assert result is True

    @pytest.mark.asyncio
    async def test_none_response_defaults_to_retrieve(self):
        """None text in response defaults to True."""
        client = MagicMock()
        mock_response = MagicMock()
        mock_response.text = None
        client.models.generate_content.return_value = mock_response
        result = await should_retrieve("hi", [], client, "gemini-2.0-flash")
        assert result is True

    @pytest.mark.asyncio
    async def test_unexpected_word_defaults_to_retrieve(self):
        """Unexpected classification word defaults to retrieve."""
        client = self._make_client("MAYBE")
        result = await should_retrieve("hi", [], client, "gemini-2.0-flash")
        assert result is True

    @pytest.mark.asyncio
    async def test_case_insensitive_skip(self):
        """'skip' in lowercase is normalized to SKIP."""
        client = self._make_client("skip")
        result = await should_retrieve("hello", [], client, "gemini-2.0-flash")
        assert result is False

    @pytest.mark.asyncio
    async def test_history_passed_to_prompt(self):
        """Verify history is included in the classification prompt."""
        client = self._make_client("RETRIEVE")
        history = [
            {"role": "user", "content": "What is sim-to-real?"},
            {"role": "assistant", "content": "Sim-to-real is..."},
        ]
        await should_retrieve(
            "explain more", history, client, "gemini-2.0-flash"
        )
        call_args = client.models.generate_content.call_args
        prompt = call_args[1]["contents"][0] if "contents" in call_args[1] else call_args[0][1][0]
        assert "sim-to-real" in prompt.lower()

    @pytest.mark.asyncio
    async def test_timeout_defaults_to_retrieve(self):
        """When Gemini call times out, default to True (retrieve)."""
        client = MagicMock()

        def slow_generate(**kwargs):
            time.sleep(2)
            mock_resp = MagicMock()
            mock_resp.text = "SKIP"
            return mock_resp

        client.models.generate_content.side_effect = slow_generate
        result = await should_retrieve(
            "hi", [], client, "gemini-2.0-flash", timeout_s=0.1
        )
        assert result is True
