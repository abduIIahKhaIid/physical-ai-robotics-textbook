"""Add overlap tokens between consecutive chunks for context continuity."""

import tiktoken

from rag.models import Chunk

_encoder = tiktoken.get_encoding("cl100k_base")


def _last_n_tokens_text(text: str, n: int) -> str:
    """Extract roughly the last n tokens of text."""
    tokens = _encoder.encode(text)
    if len(tokens) <= n:
        return text
    overlap_tokens = tokens[-n:]
    return _encoder.decode(overlap_tokens)


def add_overlaps(chunks: list[Chunk], overlap_tokens: int = 50) -> list[Chunk]:
    """Prepend overlap from previous chunk to each chunk.

    Rules:
    - First chunk gets no overlap.
    - Skip overlap across document boundaries.
    - Prepend ~overlap_tokens from end of previous chunk's text.
    """
    if not chunks or overlap_tokens <= 0:
        return chunks

    result: list[Chunk] = []
    for i, chunk in enumerate(chunks):
        if i == 0 or chunk.doc_path != chunks[i - 1].doc_path:
            result.append(chunk)
            continue

        prev_text = chunks[i - 1].text
        overlap_text = _last_n_tokens_text(prev_text, overlap_tokens)
        new_text = overlap_text + "\n\n" + chunk.text

        updated = chunk.model_copy(update={"text": new_text})
        updated.metadata.word_count = len(new_text.split())
        result.append(updated)

    return result
