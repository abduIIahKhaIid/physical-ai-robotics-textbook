"""Tests for rag.embedder.gemini_embedder with mocked Gemini client."""

import pytest
from unittest.mock import MagicMock

from rag.embedder.gemini_embedder import batch_embed
from rag.models import Chunk, ChunkMetadata


def _make_chunk(text: str, index: int) -> Chunk:
    return Chunk(
        doc_path="test.md",
        text=text,
        metadata=ChunkMetadata(doc_path="test.md", chunk_index=index),
    )


def _make_mock_gemini(dim: int = 768):
    mock = MagicMock()

    def embed_content(model, contents, config=None):
        response = MagicMock()
        embeddings = []
        items = contents if isinstance(contents, list) else [contents]
        for _ in items:
            emb = MagicMock()
            emb.values = [0.1] * dim
            embeddings.append(emb)
        response.embeddings = embeddings
        return response

    mock.models.embed_content.side_effect = embed_content
    return mock


class TestBatchEmbed:
    @pytest.mark.asyncio
    async def test_basic_embedding(self):
        chunks = [_make_chunk("hello world", 0)]
        mock_client = _make_mock_gemini()
        result = await batch_embed(
            chunks, client=mock_client, model="test-model", batch_size=10
        )
        assert len(result) == 1
        assert len(result[0].embedding) == 768

    @pytest.mark.asyncio
    async def test_batching(self):
        chunks = [_make_chunk(f"text {i}", i) for i in range(5)]
        mock_client = _make_mock_gemini()
        result = await batch_embed(
            chunks, client=mock_client, model="test", batch_size=2
        )
        assert len(result) == 5
        # Should have called embed_content 3 times (2+2+1)
        assert mock_client.models.embed_content.call_count == 3

    @pytest.mark.asyncio
    async def test_empty_input(self):
        result = await batch_embed([], client=MagicMock(), model="test")
        assert result == []

    @pytest.mark.asyncio
    async def test_embeddings_aligned(self):
        chunks = [_make_chunk(f"text {i}", i) for i in range(3)]
        mock_client = _make_mock_gemini()
        result = await batch_embed(
            chunks, client=mock_client, model="test", batch_size=10
        )
        for i, chunk in enumerate(result):
            assert chunk.text == f"text {i}"
            assert len(chunk.embedding) == 768

    @pytest.mark.asyncio
    async def test_retry_on_rate_limit(self):
        mock_client = MagicMock()
        call_count = 0

        def embed_with_retry(model, contents, config=None):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise Exception("429 Rate limit exceeded")
            response = MagicMock()
            embeddings = []
            items = contents if isinstance(contents, list) else [contents]
            for _ in items:
                emb = MagicMock()
                emb.values = [0.1] * 768
                embeddings.append(emb)
            response.embeddings = embeddings
            return response

        mock_client.models.embed_content.side_effect = embed_with_retry
        chunks = [_make_chunk("test", 0)]
        result = await batch_embed(
            chunks, client=mock_client, model="test", max_retries=3
        )
        assert len(result) == 1
        assert call_count == 2
