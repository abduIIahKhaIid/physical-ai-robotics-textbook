# Ingestion Pipeline Contract

**Feature**: 009-rag-ingestion-retrieval
**Date**: 2026-02-14

## Module Interfaces

### `rag.parser.discovery`

```python
def discover_documents(docs_dir: str = "website/docs/") -> list[Document]:
    """
    Recursively find all .md files under docs_dir.

    Returns Document objects with doc_path (relative), absolute_path,
    module, chapter, and content_type populated from path analysis.
    Content and frontmatter fields are NOT populated (see parse_frontmatter).

    Raises:
        FileNotFoundError: If docs_dir does not exist
    """
```

### `rag.parser.frontmatter`

```python
def parse_frontmatter(doc: Document) -> Document:
    """
    Read the file at doc.absolute_path, extract YAML frontmatter,
    and populate title, description, tags, learning_objectives,
    sidebar_label, sidebar_position, and content fields.

    Fallback behavior when frontmatter is missing:
    - title: First H1 heading in content, or filename without extension
    - description: Empty string
    - tags: Empty list
    - learning_objectives: Empty list
    - sidebar_label: Same as title
    - sidebar_position: 0

    Returns: Updated Document with all fields populated
    """
```

### `rag.parser.markdown_splitter`

```python
class BlockType(Enum):
    HEADING = "heading"
    PROSE = "prose"
    CODE = "code"
    TABLE = "table"
    ADMONITION = "admonition"

@dataclass
class StructuralBlock:
    type: BlockType
    content: str
    heading_level: int | None  # 1-6 for headings, None for others
    heading_text: str | None   # Raw heading text

def split_markdown(content: str) -> list[StructuralBlock]:
    """
    Parse markdown content into structural blocks.

    Rules:
    - Headings (# through ######) become HEADING blocks
    - Fenced code blocks (``` or ~~~) become CODE blocks (kept atomic)
    - Pipe tables become TABLE blocks (kept atomic)
    - Docusaurus admonitions (:::) become ADMONITION blocks
    - Everything else becomes PROSE blocks
    - LaTeX math ($$..$$) is preserved in prose
    - Consecutive prose lines are merged into one block

    Returns: Ordered list of StructuralBlocks
    """
```

### `rag.chunker.heading_chunker`

```python
def chunk_blocks(
    blocks: list[StructuralBlock],
    doc: Document,
    min_tokens: int = 200,
    max_tokens: int = 800,
    target_tokens: int = 500
) -> list[Chunk]:
    """
    Group structural blocks into chunks respecting heading boundaries.

    Algorithm:
    1. Track current heading breadcrumb as headings are encountered
    2. Accumulate non-heading blocks under current heading
    3. When accumulated tokens reach target, emit chunk
    4. NEVER split across heading boundaries — emit partial chunk at heading
    5. NEVER split code or table blocks — if one exceeds max_tokens, emit as solo chunk
    6. Chunks include heading_breadcrumb and section_heading from current state

    Returns: List of Chunk objects (without embeddings or IDs)
    """
```

### `rag.chunker.overlap`

```python
def add_overlaps(chunks: list[Chunk], overlap_tokens: int = 50) -> list[Chunk]:
    """
    Prepend overlap text from previous chunk to each chunk.

    Rules:
    - First chunk in document: no overlap
    - Overlap taken from the END of the previous chunk
    - Overlap only between chunks from the same document
    - Overlap text is prepended to chunk.text
    - chunk.word_count and content_hash reflect the final text (with overlap)
    """
```

### `rag.chunker.id_generator`

```python
def generate_ids(chunks: list[Chunk]) -> list[Chunk]:
    """
    Assign deterministic IDs and content hashes.

    ID = SHA-256(f"{chunk.doc_path}::{chunk.chunk_index}")[:16]
    content_hash = SHA-256(chunk.text)

    Returns: Chunks with id and metadata.content_hash populated
    """
```

### `rag.embedder.openai_embedder`

```python
async def batch_embed(
    chunks: list[Chunk],
    model: str = "text-embedding-3-small",
    batch_size: int = 100,
    max_retries: int = 3
) -> list[Chunk]:
    """
    Generate embeddings for all chunks using OpenAI API.

    - Batches texts in groups of batch_size
    - Retries failed batches with exponential backoff
    - Returns chunks with embedding field populated

    Raises:
        OpenAIError: After max_retries exhausted for a batch
    """
```

### `rag.store.qdrant_client`

```python
def ensure_collection(collection_name: str = "textbook_chunks") -> None:
    """Create collection if not exists, with correct vector config and indexes."""

def upsert_chunks(chunks: list[Chunk], collection_name: str = "textbook_chunks") -> int:
    """Upsert chunks (id, vector, payload). Returns count upserted."""

def delete_stale_chunks(doc_path: str, current_ids: set[str], collection_name: str = "textbook_chunks") -> int:
    """Delete chunks for doc_path whose IDs are not in current_ids. Returns count deleted."""
```

### `rag.cli`

```python
# CLI commands (click or argparse):

# ingest: Run full ingestion pipeline
#   rag ingest [--docs-dir website/docs/] [--collection textbook_chunks]

# ingest-doc: Ingest a single document
#   rag ingest-doc <doc_path>

# query: Run a test query (for development)
#   rag query "What is embodied cognition?" [--top-k 5] [--module module-1]

# evaluate: Run evaluation harness
#   rag evaluate [--test-set tests/evaluation/test_set.json]
```
