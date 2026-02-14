"""Tests for rag.models Pydantic data models."""

import pytest
from pydantic import ValidationError

from rag.models import (
    Chunk,
    ChunkMetadata,
    Citation,
    ContentType,
    Document,
    GroundedResponse,
    IngestionReport,
    Query,
    QueryFilters,
    RetrievalResult,
    ScoredChunk,
)


class TestContentType:
    def test_enum_values(self):
        assert ContentType.PROSE.value == "prose"
        assert ContentType.CODE.value == "code"
        assert ContentType.TABLE.value == "table"
        assert ContentType.LAB.value == "lab"
        assert ContentType.QUIZ.value == "quiz"
        assert ContentType.ASSESSMENT.value == "assessment"

    def test_string_enum(self):
        assert ContentType.PROSE == "prose"
        assert str(ContentType.PROSE) == "ContentType.PROSE"


class TestDocument:
    def test_defaults(self):
        doc = Document(doc_path="test.md")
        assert doc.absolute_path == ""
        assert doc.module == ""
        assert doc.tags == []
        assert doc.content_type == ContentType.PROSE

    def test_full_construction(self):
        doc = Document(
            doc_path="module-1/ch1/file.md",
            absolute_path="/abs/path.md",
            module="module-1",
            chapter="ch1",
            title="Test Title",
            tags=["tag1"],
            content_type=ContentType.LAB,
        )
        assert doc.module == "module-1"
        assert doc.content_type == ContentType.LAB


class TestQuery:
    def test_normal_mode(self):
        q = Query(question="What is AI?")
        assert q.mode == "normal"
        assert q.selected_text is None

    def test_selected_text_mode_requires_text(self):
        with pytest.raises(ValidationError, match="selected_text is required"):
            Query(question="Explain", mode="selected_text_only")

    def test_selected_text_mode_valid(self):
        q = Query(
            question="Explain",
            mode="selected_text_only",
            selected_text="Some text",
        )
        assert q.selected_text == "Some text"

    def test_empty_question_raises(self):
        with pytest.raises(ValidationError, match="question must not be empty"):
            Query(question="", mode="normal")


class TestChunkMetadata:
    def test_defaults(self):
        meta = ChunkMetadata(doc_path="test.md")
        assert meta.module == ""
        assert meta.chunk_index == 0
        assert meta.tags == []
        assert meta.content_type == "prose"

    def test_serialization_roundtrip(self):
        meta = ChunkMetadata(
            doc_path="test.md",
            module="module-1",
            tags=["a", "b"],
            chunk_index=3,
        )
        data = meta.model_dump()
        restored = ChunkMetadata(**data)
        assert restored == meta


class TestScoredChunk:
    def test_construction(self, sample_scored_chunk):
        assert sample_scored_chunk.score == 0.92
        assert sample_scored_chunk.citation.title == "1.1 Foundations of Physical AI"
