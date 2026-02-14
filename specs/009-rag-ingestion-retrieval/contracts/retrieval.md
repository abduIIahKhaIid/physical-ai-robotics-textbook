# Retrieval Pipeline Contract

**Feature**: 009-rag-ingestion-retrieval
**Date**: 2026-02-14

## Module Interfaces

### `rag.retriever.query_engine`

```python
def retrieve(query: Query) -> RetrievalResult:
    """
    Main entry point for retrieval. Routes to appropriate handler.

    Routing:
    - mode == "normal" → _retrieve_normal(query)
    - mode == "selected_text_only" → selected_text.retrieve_from_selection(query)

    CRITICAL: Mode routing happens BEFORE any Qdrant imports or calls.

    Raises:
        ValueError: If question is empty, or if mode is selected_text_only
                    and selected_text is empty/None
    """

def _retrieve_normal(query: Query) -> RetrievalResult:
    """
    Normal mode: full vector search.

    Steps:
    1. Embed query.question via openai_embedder
    2. Build Qdrant filter from query.filters
    3. Search Qdrant with vector + filter + top_k
    4. Deduplicate results (threshold=0.95)
    5. Build citations for each chunk
    6. Return RetrievalResult
    """
```

### `rag.retriever.selected_text`

```python
def retrieve_from_selection(query: Query) -> RetrievalResult:
    """
    Selected-text-only mode. NO VECTOR SEARCH.

    This module MUST NOT import qdrant_client or any store module.

    Steps:
    1. Validate query.selected_text is non-empty
    2. Create synthetic ScoredChunk:
       - chunk_id: "selection"
       - text: query.selected_text
       - score: 1.0
       - metadata: minimal (from query.source_doc_path, query.source_section)
    3. Build citation from source info
    4. Return RetrievalResult with mode="selected_text_only"

    Raises:
        ValueError: If selected_text is empty or None
    """
```

### `rag.retriever.filters`

```python
def build_qdrant_filter(filters: QueryFilters | None) -> QdrantFilter | None:
    """
    Convert QueryFilters to Qdrant filter object.

    Mapping:
    - filters.modules → FieldCondition(key="module", match=MatchAny(any=...))
    - filters.chapters → FieldCondition(key="chapter", match=MatchAny(any=...))
    - filters.content_types → FieldCondition(key="content_type", match=MatchAny(any=...))
    - filters.tags → FieldCondition(key="tags", match=MatchAny(any=...))
    - Multiple conditions combined with Filter(must=[...])
    - None/empty filters return None (no filtering)
    """
```

### `rag.retriever.dedup`

```python
def deduplicate(
    chunks: list[ScoredChunk],
    threshold: float = 0.95
) -> list[ScoredChunk]:
    """
    Remove near-duplicate chunks from results.

    Algorithm:
    - Iterate chunks in score order (highest first)
    - For each chunk, compare its text against all already-accepted chunks
    - If text similarity (by content_hash or jaccard of tokens) exceeds threshold, skip
    - Otherwise, accept the chunk

    Returns: Filtered list maintaining original score order
    """
```

### `rag.response.grounding`

```python
def apply_grounding_policy(
    result: RetrievalResult,
    question: str
) -> GroundedResponse:
    """
    Enforce groundedness constraints.

    Policy:
    1. If result.chunks is empty:
       - sufficient_context = False
       - system_instruction: refusal message
       - context: empty
    2. If mode == "selected_text_only":
       - system_instruction: "Answer ONLY from the following selected text.
         Do not use any other knowledge. If the text does not contain
         enough information to answer, say so."
       - context: the selected text
       - sufficient_context = True
    3. If mode == "normal":
       - system_instruction: "Answer based on the following textbook excerpts.
         Cite the source for each fact. If the excerpts don't contain enough
         information, explicitly state that."
       - context: formatted chunks with citation markers
       - sufficient_context = True

    Returns: GroundedResponse ready for LLM prompt construction
    """
```

### `rag.response.citation`

```python
def build_citation(metadata: ChunkMetadata) -> Citation:
    """
    Create Citation from chunk metadata.
    Uses url_builder for GitHub Pages URL generation.
    """

def format_context_with_citations(chunks: list[ScoredChunk]) -> str:
    """
    Format chunk texts with inline citation markers.

    Format:
    [Source 1: {title} - {section}]
    {chunk text}

    [Source 2: {title} - {section}]
    {chunk text}
    ...
    """
```

### `rag.url_builder`

```python
BASE_URL = "/physical-ai-robotics-textbook/docs/"

def build_url(doc_path: str, section_heading: str | None = None) -> str:
    """
    Generate GitHub Pages URL from doc_path and optional section heading.

    Steps:
    1. Strip "website/docs/" prefix if present
    2. Strip ".md" extension
    3. If filename is "index", use parent directory path
    4. Prepend BASE_URL
    5. If section_heading provided, append #{slugify(section_heading)}

    Slugify: lowercase, replace spaces/special chars with hyphens,
    strip leading/trailing hyphens, collapse multiple hyphens.

    Examples:
    - "module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md", "Embodiment"
      → "/physical-ai-robotics-textbook/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations#embodiment"
    - "module-1/1.1-introduction-to-physical-ai/index.md", None
      → "/physical-ai-robotics-textbook/docs/module-1/1.1-introduction-to-physical-ai/"
    """
```
