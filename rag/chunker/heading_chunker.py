"""Heading-aware chunking of structural blocks into indexed chunks."""

from datetime import datetime, timezone

import tiktoken

from rag.models import Chunk, ChunkMetadata, Document
from rag.parser.markdown_splitter import BlockType, StructuralBlock
from rag.url_builder import build_url

_encoder = tiktoken.get_encoding("cl100k_base")


def _count_tokens(text: str) -> int:
    return len(_encoder.encode(text))


def chunk_blocks(
    blocks: list[StructuralBlock],
    doc: Document,
    min_tokens: int = 200,
    max_tokens: int = 800,
    target_tokens: int = 500,
) -> list[Chunk]:
    """Group structural blocks under headings into chunks.

    Algorithm:
    1. Track current heading breadcrumb for context.
    2. Accumulate blocks until reaching target token count.
    3. Emit chunk at heading boundaries or when max exceeded.
    4. Oversized code/table blocks become solo chunks.

    Returns list of Chunks (without embeddings or IDs set).
    """
    chunks: list[Chunk] = []
    current_text_parts: list[str] = []
    current_tokens = 0
    heading_breadcrumb: list[str] = []
    current_section = ""
    chunk_index = 0
    now_iso = datetime.now(timezone.utc).isoformat()

    def _emit():
        nonlocal chunk_index, current_text_parts, current_tokens
        if not current_text_parts:
            return
        text = "\n\n".join(current_text_parts)
        word_count = len(text.split())
        url = build_url(doc.doc_path, current_section or None)
        chunks.append(
            Chunk(
                doc_path=doc.doc_path,
                text=text,
                metadata=ChunkMetadata(
                    doc_path=doc.doc_path,
                    module=doc.module,
                    chapter=doc.chapter,
                    section_heading=current_section,
                    heading_breadcrumb=list(heading_breadcrumb),
                    chunk_index=chunk_index,
                    content_type=doc.content_type.value,
                    tags=list(doc.tags),
                    title=doc.title,
                    word_count=word_count,
                    ingested_at=now_iso,
                    url=url,
                ),
            )
        )
        chunk_index += 1
        current_text_parts = []
        current_tokens = 0

    for block in blocks:
        # Update heading breadcrumb
        if block.block_type == BlockType.HEADING:
            level = block.heading_level
            # Trim breadcrumb to parent level
            heading_breadcrumb = heading_breadcrumb[: level - 1]
            heading_breadcrumb.append(block.heading_text)
            current_section = block.heading_text

            # Emit accumulated content at heading boundary
            if current_tokens >= min_tokens:
                _emit()
            continue

        block_tokens = _count_tokens(block.text)

        # Oversized atomic block (code or table) → solo chunk
        if block_tokens > max_tokens and block.block_type in (
            BlockType.CODE,
            BlockType.TABLE,
        ):
            _emit()  # flush pending
            current_text_parts = [block.text]
            current_tokens = block_tokens
            _emit()
            continue

        # Would exceed max → emit then start new
        if current_tokens + block_tokens > max_tokens and current_text_parts:
            _emit()

        current_text_parts.append(block.text)
        current_tokens += block_tokens

        # At or above target → emit
        if current_tokens >= target_tokens:
            _emit()

    # Emit remaining content
    _emit()

    return chunks
