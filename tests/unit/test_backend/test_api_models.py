"""Unit tests for backend API request/response models."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from backend.models.api_models import ChatRequest, QueryFilters


class TestChatRequest:
    """Test ChatRequest validation."""

    def test_rejects_empty_message(self):
        with pytest.raises(ValidationError):
            ChatRequest(message="")

    def test_rejects_message_too_long(self):
        with pytest.raises(ValidationError):
            ChatRequest(message="x" * 4001)

    def test_accepts_valid_normal_mode(self):
        req = ChatRequest(message="What is Physical AI?")
        assert req.mode == "normal"
        assert req.selected_text is None

    def test_requires_selected_text_when_mode_selected_text_only(self):
        with pytest.raises(ValidationError, match="selected_text is required"):
            ChatRequest(message="Explain this", mode="selected_text_only")

    def test_accepts_valid_selected_text_request(self):
        req = ChatRequest(
            message="Explain this",
            mode="selected_text_only",
            selected_text="Physical AI systems operate in real-time.",
            source_doc_path="/docs/module-1/chapter-1",
            source_section="Introduction",
        )
        assert req.mode == "selected_text_only"
        assert req.selected_text is not None

    def test_rejects_selected_text_too_long(self):
        with pytest.raises(ValidationError):
            ChatRequest(
                message="Explain this",
                mode="selected_text_only",
                selected_text="x" * 10001,
            )

    def test_accepts_short_selected_text(self):
        req = ChatRequest(
            message="Explain this",
            mode="selected_text_only",
            selected_text="Short text.",
        )
        assert req.selected_text == "Short text."

    def test_rejects_invalid_mode(self):
        with pytest.raises(ValidationError):
            ChatRequest(message="Hi", mode="invalid_mode")

    def test_accepts_filters(self):
        req = ChatRequest(
            message="What is ROS?",
            filters=QueryFilters(modules=["module-1"], top_k=10),
        )
        assert req.filters.modules == ["module-1"]
        assert req.filters.top_k == 10

    def test_rejects_top_k_out_of_range(self):
        with pytest.raises(ValidationError):
            ChatRequest(
                message="Hi",
                filters=QueryFilters(top_k=0),
            )
        with pytest.raises(ValidationError):
            ChatRequest(
                message="Hi",
                filters=QueryFilters(top_k=21),
            )

    def test_accepts_max_length_message(self):
        req = ChatRequest(message="x" * 4000)
        assert len(req.message) == 4000
