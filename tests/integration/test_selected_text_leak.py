"""LEAK TESTS: Verify selected-text-only mode does not access Qdrant.

These tests ensure strict isolation of the selected-text-only mode
from the vector store, preventing any outside knowledge leakage.
"""

import importlib
import inspect
from unittest.mock import MagicMock, patch

import pytest

from rag.models import Query, QueryFilters
from rag.retriever.query_engine import retrieve
from rag.retriever.selected_text import retrieve_from_selection


class TestSelectedTextNoQdrantAccess:
    """Verify that selected-text queries never touch Qdrant."""

    def test_qdrant_search_not_called(self):
        """Mock qdrant search and verify 0 calls for selected-text query."""
        mock_qdrant = MagicMock()
        mock_qdrant.search.return_value = []

        q = Query(
            question="Explain this concept",
            mode="selected_text_only",
            selected_text="Physical AI systems must operate in real-time.",
            source_doc_path="module-1/ch1/file.md",
            source_section="Real-time Operation",
        )

        result = retrieve(q, qdrant_client=mock_qdrant)

        # Qdrant search must NOT have been called
        mock_qdrant.search.assert_not_called()
        assert result.mode == "selected_text_only"
        assert len(result.chunks) == 1
        assert result.chunks[0].chunk_id == "selection"

    def test_no_embedding_call_for_selection(self):
        """Verify no Gemini embedding call for selected-text mode."""
        mock_gemini = MagicMock()

        q = Query(
            question="Explain this",
            mode="selected_text_only",
            selected_text="Some selected text.",
        )

        result = retrieve(q, gemini_client=mock_gemini)
        mock_gemini.models.embed_content.assert_not_called()

    def test_cross_contamination_prevention(self):
        """Ensure selected-text result only contains user's selected text."""
        user_text = "Specific user-selected passage about robotics."

        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text=user_text,
        )

        result = retrieve_from_selection(q)

        # The only chunk text should be exactly the user's selection
        assert len(result.chunks) == 1
        assert result.chunks[0].text == user_text
        assert result.total_candidates == 1

    def test_empty_selection_raises_value_error(self):
        """Empty selected_text must raise ValueError, not silently proceed."""
        q = Query.model_construct(
            question="Explain",
            mode="selected_text_only",
            selected_text="",
        )
        with pytest.raises(ValueError, match="must not be empty"):
            retrieve_from_selection(q)

    def test_module_import_isolation(self):
        """The selected_text module must not import any store modules."""
        module = importlib.import_module("rag.retriever.selected_text")
        source = inspect.getsource(module)

        # Must not import qdrant_client or rag.store
        assert "import qdrant_client" not in source
        assert "from qdrant_client" not in source
        assert "from rag.store" not in source
        assert "import rag.store" not in source

    def test_query_engine_routes_without_store_import(self):
        """query_engine.retrieve() should NOT import store modules
        when handling selected_text_only queries."""
        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text="Test text here.",
        )

        # This should succeed without any Qdrant connection
        result = retrieve(q)
        assert result.mode == "selected_text_only"
        assert result.chunks[0].text == "Test text here."
