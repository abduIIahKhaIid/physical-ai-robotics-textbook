"""Tests for rag.retriever.selected_text module.

Includes import isolation verification to ensure no Qdrant leakage.
"""

import importlib
import inspect

import pytest

from rag.models import Query
from rag.retriever.selected_text import retrieve_from_selection


class TestRetrieveFromSelection:
    def test_returns_synthetic_result(self):
        q = Query(
            question="Explain this",
            mode="selected_text_only",
            selected_text="Physical AI operates in real-time.",
            source_doc_path="module-1/1.1-intro/file.md",
            source_section="Real-time Operation",
        )
        result = retrieve_from_selection(q)
        assert len(result.chunks) == 1
        assert result.chunks[0].chunk_id == "selection"
        assert result.chunks[0].score == 1.0
        assert result.mode == "selected_text_only"

    def test_text_in_chunk(self):
        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text="The robot moves forward.",
        )
        result = retrieve_from_selection(q)
        assert result.chunks[0].text == "The robot moves forward."

    def test_citation_built(self):
        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text="Some text.",
            source_doc_path="module-1/ch1/file.md",
            source_section="Section A",
        )
        result = retrieve_from_selection(q)
        assert result.chunks[0].citation.section == "Section A"
        assert "/physical-ai-robotics-textbook/docs/" in result.chunks[0].citation.url

    def test_empty_selected_text_raises(self):
        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text="placeholder",  # pass validation
        )
        # Manually set to empty after construction
        q_dict = q.model_dump()
        q_dict["selected_text"] = ""
        with pytest.raises(ValueError, match="must not be empty"):
            retrieve_from_selection(Query.model_construct(**q_dict))

    def test_no_qdrant_import(self):
        """CRITICAL: selected_text module must NOT import qdrant_client."""
        module = importlib.import_module("rag.retriever.selected_text")
        source = inspect.getsource(module)
        assert "qdrant_client" not in source
        assert "rag.store" not in source

    def test_total_candidates_is_one(self):
        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text="Some text.",
        )
        result = retrieve_from_selection(q)
        assert result.total_candidates == 1
