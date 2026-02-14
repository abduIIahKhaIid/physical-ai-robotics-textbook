"""Tests for rag.chunker.id_generator."""

import re

from rag.chunker.id_generator import generate_ids
from rag.models import Chunk, ChunkMetadata


def _make_chunk(doc_path: str, text: str, index: int) -> Chunk:
    return Chunk(
        doc_path=doc_path,
        text=text,
        metadata=ChunkMetadata(doc_path=doc_path, chunk_index=index),
    )


class TestGenerateIds:
    def test_deterministic(self):
        chunks = [_make_chunk("doc.md", "hello", 0)]
        result1 = generate_ids(chunks)
        result2 = generate_ids(chunks)
        assert result1[0].id == result2[0].id

    def test_different_inputs_different_ids(self):
        chunks_a = generate_ids([_make_chunk("doc-a.md", "hello", 0)])
        chunks_b = generate_ids([_make_chunk("doc-b.md", "hello", 0)])
        assert chunks_a[0].id != chunks_b[0].id

    def test_different_indexes_different_ids(self):
        chunks = generate_ids([
            _make_chunk("doc.md", "hello", 0),
            _make_chunk("doc.md", "world", 1),
        ])
        assert chunks[0].id != chunks[1].id

    def test_hex_format_16_chars(self):
        chunks = generate_ids([_make_chunk("doc.md", "test", 0)])
        assert len(chunks[0].id) == 16
        assert re.match(r"^[0-9a-f]{16}$", chunks[0].id)

    def test_content_hash_set(self):
        chunks = generate_ids([_make_chunk("doc.md", "test content", 0)])
        assert chunks[0].metadata.content_hash != ""
        assert len(chunks[0].metadata.content_hash) == 64  # Full SHA-256

    def test_different_text_different_hash(self):
        chunks = generate_ids([
            _make_chunk("doc.md", "text A", 0),
            _make_chunk("doc.md", "text B", 1),
        ])
        assert chunks[0].metadata.content_hash != chunks[1].metadata.content_hash
