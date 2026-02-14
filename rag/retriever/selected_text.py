"""Selected-text-only retrieval mode.

CRITICAL: This module MUST NOT import the vector store client or any store module.
It bypasses vector search entirely, answering only from user-provided text.
"""

from rag.models import (
    ChunkMetadata,
    Citation,
    Query,
    RetrievalResult,
    ScoredChunk,
)
from rag.url_builder import build_url


def retrieve_from_selection(query: Query) -> RetrievalResult:
    """Create a retrieval result from the user's selected text.

    This completely bypasses Qdrant search. The selected text is wrapped
    in a synthetic ScoredChunk with score=1.0.

    Args:
        query: Must have mode="selected_text_only" and non-empty selected_text.

    Returns:
        RetrievalResult with a single synthetic chunk.

    Raises:
        ValueError: If selected_text is empty or None.
    """
    if not query.selected_text:
        raise ValueError("selected_text must not be empty for selected-text-only mode")

    doc_path = query.source_doc_path or ""
    section = query.source_section or ""

    url = build_url(doc_path, section or None) if doc_path else ""

    metadata = ChunkMetadata(
        doc_path=doc_path,
        section_heading=section,
        heading_breadcrumb=[section] if section else [],
        chunk_index=0,
        content_type="prose",
        word_count=len(query.selected_text.split()),
        url=url,
    )

    citation = Citation(
        title=section or "Selected Text",
        section=section,
        url=url,
    )

    chunk = ScoredChunk(
        chunk_id="selection",
        text=query.selected_text,
        score=1.0,
        metadata=metadata,
        citation=citation,
    )

    return RetrievalResult(
        chunks=[chunk],
        query=query,
        mode="selected_text_only",
        total_candidates=1,
    )
