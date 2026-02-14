"""Qdrant vector store operations for chunk storage and retrieval."""

import logging

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Filter,
    PointStruct,
)

from rag.config import load_settings
from rag.models import Chunk, ChunkMetadata, ScoredChunk, Citation
from rag.response.citation import build_citation
from rag.store.collection_config import (
    COLLECTION_NAME,
    PAYLOAD_INDEXES,
    VECTOR_CONFIG,
)

logger = logging.getLogger(__name__)


def _get_client(client: QdrantClient | None = None) -> QdrantClient:
    if client is not None:
        return client
    settings = load_settings()
    return QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key or None)


def ensure_collection(client: QdrantClient | None = None) -> None:
    """Create collection and payload indexes if they don't exist."""
    qdrant = _get_client(client)
    collections = [c.name for c in qdrant.get_collections().collections]

    if COLLECTION_NAME not in collections:
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VECTOR_CONFIG,
        )
        logger.info(f"Created collection '{COLLECTION_NAME}'")

    for idx in PAYLOAD_INDEXES:
        qdrant.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name=idx["field_name"],
            field_schema=idx["field_schema"],
        )
    logger.info("Payload indexes ensured")


def upsert_chunks(chunks: list[Chunk], client: QdrantClient | None = None) -> int:
    """Upsert chunks into Qdrant. Returns count of upserted points."""
    if not chunks:
        return 0

    qdrant = _get_client(client)
    points = [
        PointStruct(
            id=chunk.id,
            vector=chunk.embedding,
            payload=chunk.metadata.model_dump(),
        )
        for chunk in chunks
    ]

    settings = load_settings()
    batch_size = settings.batch_size
    total = 0

    for i in range(0, len(points), batch_size):
        batch = points[i : i + batch_size]
        qdrant.upsert(collection_name=COLLECTION_NAME, points=batch)
        total += len(batch)
        logger.info(f"Upserted batch of {len(batch)} points")

    return total


def delete_stale_chunks(
    doc_path: str,
    current_ids: set[str],
    client: QdrantClient | None = None,
) -> int:
    """Delete chunks for a doc_path that are not in current_ids.

    Returns count of deleted points.
    """
    qdrant = _get_client(client)
    from qdrant_client.models import FieldCondition, MatchValue

    # Scroll all existing chunks for this doc
    doc_filter = Filter(
        must=[FieldCondition(key="doc_path", match=MatchValue(value=doc_path))]
    )
    existing_ids: list[str] = []
    offset = None
    while True:
        points, offset = qdrant.scroll(
            collection_name=COLLECTION_NAME,
            scroll_filter=doc_filter,
            limit=100,
            offset=offset,
        )
        existing_ids.extend(str(p.id) for p in points)
        if offset is None:
            break

    stale_ids = [pid for pid in existing_ids if pid not in current_ids]
    if stale_ids:
        qdrant.delete(
            collection_name=COLLECTION_NAME,
            points_selector=stale_ids,
        )
        logger.info(f"Deleted {len(stale_ids)} stale chunks for {doc_path}")
    return len(stale_ids)


def search_chunks(
    vector: list[float],
    query_filter: Filter | None = None,
    top_k: int = 5,
    client: QdrantClient | None = None,
) -> list[ScoredChunk]:
    """Search Qdrant for similar chunks.

    Returns ScoredChunk objects with citations built.
    """
    qdrant = _get_client(client)
    hits = qdrant.search(
        collection_name=COLLECTION_NAME,
        query_vector=vector,
        query_filter=query_filter,
        limit=top_k,
    )

    scored: list[ScoredChunk] = []
    for hit in hits:
        payload = hit.payload or {}
        metadata = ChunkMetadata(**payload)
        citation = build_citation(metadata)
        scored.append(
            ScoredChunk(
                chunk_id=str(hit.id),
                text=payload.get("text", ""),
                score=hit.score,
                metadata=metadata,
                citation=citation,
            )
        )
    return scored
