"""Batch embedding via OpenAI API with retry logic."""

import asyncio
import logging

from openai import OpenAI

from rag.config import load_settings
from rag.models import Chunk

logger = logging.getLogger(__name__)


async def batch_embed(
    chunks: list[Chunk],
    client: OpenAI | None = None,
    model: str | None = None,
    batch_size: int = 100,
    max_retries: int = 3,
) -> list[Chunk]:
    """Embed chunks in batches using OpenAI embeddings API.

    Args:
        chunks: Chunks to embed (text used as input).
        client: Optional pre-configured OpenAI client.
        model: Embedding model name. Defaults to settings.
        batch_size: Number of texts per API call.
        max_retries: Max retry attempts on rate limit errors.

    Returns:
        Chunks with embedding field populated.
    """
    if not chunks:
        return chunks

    settings = load_settings()
    if client is None:
        client = OpenAI(api_key=settings.openai_api_key)
    if model is None:
        model = settings.embedding_model

    result: list[Chunk] = []

    for batch_start in range(0, len(chunks), batch_size):
        batch = chunks[batch_start : batch_start + batch_size]
        texts = [c.text for c in batch]

        embeddings = await _embed_with_retry(client, texts, model, max_retries)

        for chunk, embedding in zip(batch, embeddings):
            updated = chunk.model_copy(update={"embedding": embedding})
            result.append(updated)

        logger.info(
            f"Embedded batch {batch_start // batch_size + 1} "
            f"({len(batch)} chunks)"
        )

    return result


async def _embed_with_retry(
    client: OpenAI,
    texts: list[str],
    model: str,
    max_retries: int,
) -> list[list[float]]:
    """Call OpenAI embeddings with exponential backoff retry."""
    for attempt in range(max_retries):
        try:
            response = client.embeddings.create(input=texts, model=model)
            return [item.embedding for item in response.data]
        except Exception as e:
            if attempt < max_retries - 1 and _is_retryable(e):
                wait = 2**attempt
                logger.warning(f"Retry {attempt + 1}/{max_retries} after {wait}s: {e}")
                await asyncio.sleep(wait)
            else:
                raise


def _is_retryable(error: Exception) -> bool:
    """Check if the error is retryable (rate limit or server error)."""
    error_str = str(error).lower()
    return "429" in error_str or "rate" in error_str or "500" in error_str
