"""Query engine that routes between normal and selected-text modes."""

from openai import OpenAI

from rag.config import load_settings
from rag.models import Query, RetrievalResult
from rag.retriever.selected_text import retrieve_from_selection


def retrieve(
    query: Query,
    openai_client: OpenAI | None = None,
    qdrant_client=None,
) -> RetrievalResult:
    """Route query to appropriate retrieval mode.

    IMPORTANT: selected_text_only mode is checked FIRST, before any
    Qdrant imports or operations, ensuring complete isolation.
    """
    if query.mode == "selected_text_only":
        return retrieve_from_selection(query)

    return _retrieve_normal(query, openai_client, qdrant_client)


def _retrieve_normal(
    query: Query,
    openai_client: OpenAI | None = None,
    qdrant_client=None,
) -> RetrievalResult:
    """Full-book retrieval via Qdrant vector search."""
    # Lazy imports to keep selected_text mode isolated from store
    from rag.retriever.filters import build_qdrant_filter
    from rag.retriever.dedup import deduplicate
    from rag.store.qdrant_store import search_chunks

    settings = load_settings()

    # Embed the question
    if openai_client is None:
        openai_client = OpenAI(api_key=settings.openai_api_key)

    response = openai_client.embeddings.create(
        input=[query.question],
        model=settings.embedding_model,
    )
    query_vector = response.data[0].embedding

    # Build filters
    top_k = query.filters.top_k if query.filters else 5
    qdrant_filter = build_qdrant_filter(query.filters)

    # Search
    scored_chunks = search_chunks(
        vector=query_vector,
        query_filter=qdrant_filter,
        top_k=top_k,
        client=qdrant_client,
    )

    # Deduplicate
    deduped = deduplicate(scored_chunks, threshold=settings.dedup_threshold)

    return RetrievalResult(
        chunks=deduped,
        query=query,
        mode="normal",
        total_candidates=len(scored_chunks),
    )
