"""Generate deterministic chunk IDs and content hashes."""

import hashlib

from rag.models import Chunk


def _sha256_hex(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def generate_ids(chunks: list[Chunk]) -> list[Chunk]:
    """Set deterministic id and content_hash for each chunk.

    id = SHA-256(doc_path::chunk_index)[:16]
    content_hash = SHA-256(text)
    """
    result: list[Chunk] = []
    for chunk in chunks:
        key = f"{chunk.doc_path}::{chunk.metadata.chunk_index}"
        chunk_id = _sha256_hex(key)[:16]
        content_hash = _sha256_hex(chunk.text)

        updated = chunk.model_copy(update={"id": chunk_id})
        updated.metadata.content_hash = content_hash
        result.append(updated)

    return result
