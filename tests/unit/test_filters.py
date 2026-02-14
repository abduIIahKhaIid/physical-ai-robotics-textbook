"""Tests for rag.retriever.filters."""

from rag.models import QueryFilters
from rag.retriever.filters import build_qdrant_filter


class TestBuildQdrantFilter:
    def test_none_returns_none(self):
        assert build_qdrant_filter(None) is None

    def test_empty_filters_returns_none(self):
        filters = QueryFilters()
        assert build_qdrant_filter(filters) is None

    def test_single_module_filter(self):
        filters = QueryFilters(modules=["module-1"])
        result = build_qdrant_filter(filters)
        assert result is not None
        assert len(result.must) == 1

    def test_multiple_filters(self):
        filters = QueryFilters(
            modules=["module-1"],
            content_types=["prose"],
            tags=["robotics"],
        )
        result = build_qdrant_filter(filters)
        assert result is not None
        assert len(result.must) == 3

    def test_chapters_filter(self):
        filters = QueryFilters(chapters=["1.1-intro"])
        result = build_qdrant_filter(filters)
        assert result is not None
        assert len(result.must) == 1

    def test_all_filters(self):
        filters = QueryFilters(
            modules=["module-1"],
            chapters=["ch1"],
            content_types=["prose"],
            tags=["ai"],
        )
        result = build_qdrant_filter(filters)
        assert len(result.must) == 4
