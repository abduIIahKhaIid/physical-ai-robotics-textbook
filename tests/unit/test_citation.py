"""Tests for rag.response.citation building and formatting."""

from rag.models import ChunkMetadata, Citation, ScoredChunk
from rag.response.citation import build_citation, format_context_with_citations


class TestBuildCitation:
    def test_basic(self):
        meta = ChunkMetadata(
            doc_path="module-1/ch1/file.md",
            title="Test Title",
            section_heading="Section A",
            module="module-1",
            chapter="ch1",
            url="/physical-ai-robotics-textbook/docs/module-1/ch1/file#section-a",
        )
        cit = build_citation(meta)
        assert cit.title == "Test Title"
        assert cit.section == "Section A"
        assert cit.module == "module-1"
        assert "/physical-ai-robotics-textbook/docs/" in cit.url

    def test_generates_url_when_missing(self):
        meta = ChunkMetadata(
            doc_path="module-1/ch1/file.md",
            title="Title",
            section_heading="Heading",
            url="",
        )
        cit = build_citation(meta)
        assert "/physical-ai-robotics-textbook/docs/" in cit.url
        assert "heading" in cit.url


class TestFormatContextWithCitations:
    def test_numbered_sources(self):
        chunks = [
            ScoredChunk(
                chunk_id="a",
                text="First chunk text.",
                score=0.9,
                metadata=ChunkMetadata(doc_path="a.md"),
                citation=Citation(title="Title A", section="Sec A", url="/a"),
            ),
            ScoredChunk(
                chunk_id="b",
                text="Second chunk text.",
                score=0.8,
                metadata=ChunkMetadata(doc_path="b.md"),
                citation=Citation(title="Title B", section="Sec B", url="/b"),
            ),
        ]
        formatted = format_context_with_citations(chunks)
        assert "[Source 1: Title A - Sec A]" in formatted
        assert "[Source 2: Title B - Sec B]" in formatted
        assert "First chunk text." in formatted
        assert "Second chunk text." in formatted

    def test_empty_list(self):
        assert format_context_with_citations([]) == ""

    def test_single_chunk(self):
        chunks = [
            ScoredChunk(
                chunk_id="a",
                text="Only text.",
                score=0.9,
                metadata=ChunkMetadata(doc_path="a.md"),
                citation=Citation(title="T", section="S", url="/u"),
            ),
        ]
        formatted = format_context_with_citations(chunks)
        assert "[Source 1:" in formatted
        assert "Only text." in formatted
