"""Citation building and context formatting for retrieval results."""

from rag.models import ChunkMetadata, Citation, ScoredChunk
from rag.url_builder import build_url


def build_citation(metadata: ChunkMetadata) -> Citation:
    """Build a Citation from chunk metadata."""
    url = metadata.url or build_url(metadata.doc_path, metadata.section_heading or None)
    return Citation(
        title=metadata.title,
        section=metadata.section_heading,
        url=url,
        module=metadata.module,
        chapter=metadata.chapter,
    )


def format_context_with_citations(chunks: list[ScoredChunk]) -> str:
    """Format scored chunks as numbered context with source citations.

    Output format:
    [Source 1: Title - Section]
    chunk text...

    [Source 2: Title - Section]
    chunk text...
    """
    parts: list[str] = []
    for i, chunk in enumerate(chunks, 1):
        c = chunk.citation
        header = f"[Source {i}: {c.title} - {c.section}]"
        parts.append(f"{header}\n{chunk.text}")
    return "\n\n".join(parts)
