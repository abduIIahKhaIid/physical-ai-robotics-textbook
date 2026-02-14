"""Tests for rag.embedder.openai_embedder with mocked OpenAI."""

import pytest
from unittest.mock import MagicMock, patch

from rag.embedder.openai_embedder import batch_embed
from rag.models import Chunk, ChunkMetadata


def _make_chunk(text: str, index: int) -> Chunk:
    return Chunk(
        doc_path="test.md",
        text=text,
        metadata=ChunkMetadata(doc_path="test.md", chunk_index=index),
    )


def _make_mock_openai(dim: int = 1536):
    mock = MagicMock()

    def create_embeddings(input, model):
        response = MagicMock()
        items = []
        for _ in input:
            item = MagicMock()
            item.embedding = [0.1] * dim
            items.append(item)
        response.data = items
        return response

    mock.embeddings.create.side_effect = create_embeddings
    return mock


class TestBatchEmbed:
    @pytest.mark.asyncio
    async def test_basic_embedding(self):
        chunks = [_make_chunk("hello world", 0)]
        mock_client = _make_mock_openai()
        result = await batch_embed(chunks, client=mock_client, model="test-model", batch_size=10)
        assert len(result) == 1
        assert len(result[0].embedding) == 1536

    @pytest.mark.asyncio
    async def test_batching(self):
        chunks = [_make_chunk(f"text {i}", i) for i in range(5)]
        mock_client = _make_mock_openai()
        result = await batch_embed(chunks, client=mock_client, model="test", batch_size=2)
        assert len(result) == 5
        # Should have called create 3 times (2+2+1)
        assert mock_client.embeddings.create.call_count == 3

    @pytest.mark.asyncio
    async def test_empty_input(self):
        result = await batch_embed([], client=MagicMock(), model="test")
        assert result == []

    @pytest.mark.asyncio
    async def test_embeddings_aligned(self):
        chunks = [_make_chunk(f"text {i}", i) for i in range(3)]
        mock_client = _make_mock_openai()
        result = await batch_embed(chunks, client=mock_client, model="test", batch_size=10)
        for i, chunk in enumerate(result):
            assert chunk.text == f"text {i}"
            assert len(chunk.embedding) == 1536

    @pytest.mark.asyncio
    async def test_retry_on_rate_limit(self):
        mock_client = MagicMock()
        call_count = 0

        def create_with_retry(input, model):
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise Exception("429 Rate limit exceeded")
            response = MagicMock()
            items = []
            for _ in input:
                item = MagicMock()
                item.embedding = [0.1] * 1536
                items.append(item)
            response.data = items
            return response

        mock_client.embeddings.create.side_effect = create_with_retry
        chunks = [_make_chunk("test", 0)]
        result = await batch_embed(chunks, client=mock_client, model="test", max_retries=3)
        assert len(result) == 1
        assert call_count == 2
