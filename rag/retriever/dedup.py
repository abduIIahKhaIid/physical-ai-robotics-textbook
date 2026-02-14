"""Near-duplicate removal using token-level Jaccard similarity."""

import tiktoken

from rag.models import ScoredChunk

_encoder = tiktoken.get_encoding("cl100k_base")


def _token_set(text: str) -> set[int]:
    """Get the set of token IDs for a text."""
    return set(_encoder.encode(text))


def _jaccard_similarity(set_a: set[int], set_b: set[int]) -> float:
    """Compute Jaccard similarity between two token sets."""
    if not set_a and not set_b:
        return 1.0
    intersection = len(set_a & set_b)
    union = len(set_a | set_b)
    return intersection / union if union > 0 else 0.0


def deduplicate(
    chunks: list[ScoredChunk],
    threshold: float = 0.95,
) -> list[ScoredChunk]:
    """Remove near-duplicate chunks by token-level Jaccard similarity.

    Chunks are processed in score order (highest first).
    A chunk is dropped if its Jaccard similarity with any kept chunk
    exceeds the threshold.
    """
    if not chunks:
        return chunks

    sorted_chunks = sorted(chunks, key=lambda c: c.score, reverse=True)
    kept: list[ScoredChunk] = []
    kept_token_sets: list[set[int]] = []

    for chunk in sorted_chunks:
        chunk_tokens = _token_set(chunk.text)
        is_dup = False
        for existing_tokens in kept_token_sets:
            if _jaccard_similarity(chunk_tokens, existing_tokens) >= threshold:
                is_dup = True
                break
        if not is_dup:
            kept.append(chunk)
            kept_token_sets.append(chunk_tokens)

    return kept
