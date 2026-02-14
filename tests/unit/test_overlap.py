"""Tests for rag.chunker.overlap."""

from rag.chunker.overlap import add_overlaps
from rag.models import Chunk, ChunkMetadata


def _make_chunk(doc_path: str, text: str, index: int) -> Chunk:
    return Chunk(
        doc_path=doc_path,
        text=text,
        metadata=ChunkMetadata(
            doc_path=doc_path,
            chunk_index=index,
            word_count=len(text.split()),
        ),
    )


class TestAddOverlaps:
    def test_first_chunk_no_overlap(self):
        chunks = [_make_chunk("doc.md", "First chunk text", 0)]
        result = add_overlaps(chunks, overlap_tokens=10)
        assert result[0].text == "First chunk text"

    def test_subsequent_gets_overlap(self):
        chunks = [
            _make_chunk("doc.md", "First chunk with some words to overlap", 0),
            _make_chunk("doc.md", "Second chunk text", 1),
        ]
        result = add_overlaps(chunks, overlap_tokens=5)
        assert len(result) == 2
        # Second chunk should have overlap prepended
        assert "Second chunk text" in result[1].text
        # Should be longer than original
        assert len(result[1].text) > len("Second chunk text")

    def test_no_cross_doc_overlap(self):
        chunks = [
            _make_chunk("doc-a.md", "Doc A last chunk", 0),
            _make_chunk("doc-b.md", "Doc B first chunk", 0),
        ]
        result = add_overlaps(chunks, overlap_tokens=5)
        assert result[1].text == "Doc B first chunk"

    def test_empty_input(self):
        assert add_overlaps([], overlap_tokens=10) == []

    def test_zero_overlap(self):
        chunks = [
            _make_chunk("doc.md", "First", 0),
            _make_chunk("doc.md", "Second", 1),
        ]
        result = add_overlaps(chunks, overlap_tokens=0)
        assert result[1].text == "Second"
