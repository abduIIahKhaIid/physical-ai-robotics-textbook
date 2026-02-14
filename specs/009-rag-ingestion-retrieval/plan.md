# Implementation Plan: RAG Ingestion & Retrieval

**Branch**: `009-rag-ingestion-retrieval` | **Date**: 2026-02-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-rag-ingestion-retrieval/spec.md`

## Summary

Build an end-to-end RAG pipeline that ingests 89 Docusaurus markdown files from `website/docs/` into Qdrant Cloud, supporting two retrieval modes: full-book semantic search (normal mode) and selected-text-only mode (bypasses vector search entirely). The pipeline consists of three layers: (1) an offline ingestion CLI that discovers, parses, chunks, embeds, and upserts documents; (2) a retrieval module that accepts queries, embeds them, searches Qdrant with optional metadata filters, and returns ranked chunks with citations; (3) a response policy module that enforces groundedness constraints and selected-text-only isolation. Scope is limited to ingestion and retrieval logic — no FastAPI endpoints or ChatKit UI integration in this feature.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: `qdrant-client` (Qdrant Cloud SDK), `openai` (embeddings via OpenAI API), `python-frontmatter` (YAML frontmatter parsing), `tiktoken` (token counting for chunking), `hashlib` (content hashing, stdlib), `pydantic` (data models and validation)
**Storage**: Qdrant Cloud (vector store), local filesystem (source markdown)
**Testing**: `pytest` with `pytest-asyncio` for async Qdrant operations
**Target Platform**: Linux/macOS CLI, Python package consumable by future FastAPI backend
**Project Type**: Single Python package (library + CLI)
**Performance Goals**: Full ingestion of 89 docs in <10 minutes; retrieval query <2 seconds; embedding batch throughput ≥50 chunks/second
**Constraints**: No secrets in repo (`.env` for API keys); deterministic chunk IDs for idempotent upserts; selected-text-only mode must NEVER touch vector store
**Scale/Scope**: 89 markdown files → ~1,500–3,000 chunks; single Qdrant collection; single embedding model

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
| --------- | ------ | ----- |
| I. Documentation-First | PASS | Spec and plan written before implementation |
| II. Selected Text Only Mode | PASS | FR-010/FR-011 enforce strict isolation; plan includes leak tests |
| III. Test-First (NON-NEGOTIABLE) | PASS | TDD enforced: test harness and contract tests precede implementation |
| IV. Secure Architecture | PASS | API keys via `.env`; no secrets in repo; `.gitignore` updated |
| V. Scalable Cloud Infrastructure | PASS | Qdrant Cloud for vector storage; batch embedding for throughput |
| VI. Modular Component Design | PASS | Clear separation: parser → chunker → embedder → upserter → retriever |

## Project Structure

### Documentation (this feature)

```text
specs/009-rag-ingestion-retrieval/
├── plan.md              # This file
├── research.md          # Phase 0: technology decisions
├── data-model.md        # Phase 1: entity schemas
├── quickstart.md        # Phase 1: how to run ingestion + test retrieval
├── contracts/           # Phase 1: module interfaces
│   ├── ingestion.md     # Ingestion pipeline contract
│   └── retrieval.md     # Retrieval pipeline contract
└── tasks.md             # Phase 2: implementation tasks
```

### Source Code (repository root)

```text
rag/
├── __init__.py
├── config.py                  # Environment config (Qdrant URL, API keys, model name)
├── models.py                  # Pydantic models: Document, Chunk, Query, RetrievalResult
├── parser/
│   ├── __init__.py
│   ├── discovery.py           # Discover markdown files in website/docs/
│   ├── frontmatter.py         # Parse YAML frontmatter + fallback to path-derived metadata
│   └── markdown_splitter.py   # Split markdown body into structural blocks (headings, code, tables, prose)
├── chunker/
│   ├── __init__.py
│   ├── heading_chunker.py     # Heading-aware chunking with token limits
│   ├── overlap.py             # Overlap generation between adjacent chunks
│   └── id_generator.py        # Deterministic chunk ID from doc_path + chunk_index + content_hash
├── embedder/
│   ├── __init__.py
│   └── openai_embedder.py     # Batch embedding via OpenAI text-embedding-3-small
├── store/
│   ├── __init__.py
│   ├── qdrant_client.py       # Qdrant collection setup, upsert, delete, search
│   └── collection_config.py   # Collection schema, vector config, payload indexes
├── retriever/
│   ├── __init__.py
│   ├── query_engine.py        # Normal mode: embed query → search Qdrant → rank → cite
│   ├── filters.py             # Metadata filter builder (module, chapter, content_type, tags)
│   ├── dedup.py               # Near-duplicate chunk filtering at retrieval time
│   └── selected_text.py       # Selected-text-only mode: bypass vector store, use provided text
├── response/
│   ├── __init__.py
│   ├── grounding.py           # Groundedness policy: refuse if context insufficient
│   └── citation.py            # Citation formatter: doc title, section, URL with baseUrl
├── cli.py                     # CLI entry point: ingest, query, evaluate commands
└── url_builder.py             # Generate GitHub Pages URLs with baseUrl /physical-ai-robotics-textbook/

tests/
├── conftest.py                # Shared fixtures (sample docs, mock Qdrant, mock OpenAI)
├── unit/
│   ├── test_discovery.py
│   ├── test_frontmatter.py
│   ├── test_markdown_splitter.py
│   ├── test_heading_chunker.py
│   ├── test_overlap.py
│   ├── test_id_generator.py
│   ├── test_openai_embedder.py
│   ├── test_filters.py
│   ├── test_dedup.py
│   ├── test_selected_text.py
│   ├── test_grounding.py
│   ├── test_citation.py
│   └── test_url_builder.py
├── integration/
│   ├── test_ingestion_pipeline.py    # End-to-end: parse → chunk → embed → upsert
│   ├── test_retrieval_pipeline.py    # End-to-end: query → search → filter → cite
│   └── test_selected_text_leak.py    # LEAK TEST: verify selected-text mode never queries Qdrant
├── evaluation/
│   ├── eval_harness.py               # Evaluation runner: precision@5, recall@5, MRR
│   ├── test_set.json                 # 20 curated question-answer-expected_chunks triples
│   └── test_eval_harness.py          # Tests for the evaluation harness itself
└── fixtures/
    ├── sample_chapter.md             # Representative chapter with all content types
    ├── sample_lab.md                 # Lab exercise with code blocks
    ├── sample_quiz.md                # Quiz with Q&A format
    └── no_frontmatter.md             # File without YAML frontmatter
```

**Structure Decision**: Single Python package at repo root (`rag/`) with CLI entry point. This is a library-first design so the future FastAPI backend can import `rag.retriever` and `rag.response` directly without running the CLI. Tests are separate at `tests/` following pytest conventions.

---

## Phase 0: Research & Decisions

### Decision 1: Embedding Model

**Decision**: OpenAI `text-embedding-3-small` (1536 dimensions)
**Rationale**: Constitution mandates OpenAI; `text-embedding-3-small` offers best cost/quality ratio for educational content. 1536 dims is the default and matches Qdrant's optimized HNSW index. Cost: ~$0.02 per 1M tokens — full textbook ingestion costs <$0.01.
**Alternatives considered**:
- `text-embedding-3-large` (3072 dims): Higher quality but 2x storage/cost. Overkill for ~3K chunks of English educational text.
- `text-embedding-ada-002`: Legacy model, superseded by v3 family.
- Open-source (e.g., `all-MiniLM-L6-v2`): Lower quality, no constitution alignment, requires self-hosting.

### Decision 2: Chunking Strategy

**Decision**: Heading-aware hierarchical chunking with token-based size control
**Rationale**: The textbook has consistent heading structure (H1 = chapter, H2 = section, H3 = subsection). Splitting on headings preserves semantic boundaries. Token-based sizing (200–800 tokens, target 500) ensures chunks fit within embedding model context and LLM retrieval windows.
**Algorithm**:
1. Parse markdown into structural blocks: headings, prose paragraphs, code blocks (fenced), tables, admonitions (Docusaurus `:::` syntax)
2. Group consecutive blocks under their nearest heading ancestor
3. Accumulate blocks into a chunk until token count reaches target (500 tokens)
4. If a single block exceeds 800 tokens (e.g., large code block), emit it as its own chunk
5. Never split across heading boundaries, code fences, or table delimiters
6. Generate 50-token overlap by prepending the last ~50 tokens of the previous chunk to the next

**Alternatives considered**:
- Fixed-size character splitting (LangChain `RecursiveCharacterTextSplitter`): Ignores semantic boundaries, splits code blocks.
- Sentence-level splitting: Too granular for educational content; loses section context.
- Full-section chunks (one chunk per H2): Some sections are 2000+ tokens — too large for embedding quality.

### Decision 3: Qdrant Collection Configuration

**Decision**: Single collection `textbook_chunks` with HNSW index, cosine distance, 1536 dimensions
**Configuration**:
- Vector name: `default`
- Dimensions: 1536 (matches `text-embedding-3-small`)
- Distance: Cosine
- HNSW config: `m=16, ef_construct=100` (default, suitable for <10K vectors)
- On-disk storage: disabled (dataset fits in memory at ~3K vectors × 1536 dims ≈ 18MB)
- Payload indexes (keyword type): `module`, `chapter`, `content_type`, `tags`

**Rationale**: Single collection simplifies operations. Payload indexes enable efficient filtered search per FR-014/FR-016. Cosine distance is standard for OpenAI embeddings. Dataset is small enough that in-memory HNSW provides <50ms query latency.

### Decision 4: Chunk ID Generation

**Decision**: Deterministic ID = SHA-256(`{doc_path}::{chunk_index}`)[:16] (hex string, 16 chars)
**Rationale**: Deterministic IDs enable idempotent upserts (FR-004). Using doc_path + chunk_index means the same document always produces the same chunk IDs. Content hash is stored in payload (not in the ID) so that changed content is detected and the chunk is overwritten. The 16-char hex prefix provides 64 bits of entropy — collision probability is negligible for <10K chunks.
**Stale chunk cleanup**: During re-ingestion of a document, compute the set of new chunk IDs and delete any existing chunks for that `doc_path` whose IDs are not in the new set.

### Decision 5: Selected-Text-Only Mode Architecture

**Decision**: Complete vector store bypass — selected text is passed directly to the response generator as context
**Architecture**:
1. Client sends `{ question: "...", selected_text: "...", mode: "selected_text_only" }`
2. Retriever module detects mode and returns a synthetic `RetrievalResult` containing only the selected text as a single "chunk"
3. Response generator receives this result identically to normal mode — same grounding policy applies
4. **No Qdrant query is ever made** in this mode
5. Citation in this mode references the document/section from which the text was selected (passed by client)

**Leak prevention**:
- The `selected_text.py` module is the only code path for this mode
- It constructs a `RetrievalResult` without importing or calling any Qdrant client
- Integration test `test_selected_text_leak.py` mocks Qdrant client and asserts it receives zero calls during selected-text-only queries
- The retriever interface enforces mode routing before any search logic runs

### Decision 6: URL/Anchor Generation for Citations

**Decision**: Generate URLs compatible with GitHub Pages deployment at `https://abduIIahKhaIid.github.io/physical-ai-robotics-textbook/`
**Algorithm**:
1. Base URL: `/physical-ai-robotics-textbook/docs/` (from `docusaurus.config.js` baseUrl + docs routeBasePath)
2. Doc path: Strip `website/docs/` prefix and `.md` extension
3. For index.md files: Use parent directory path (Docusaurus convention)
4. Anchor: Slugify the section heading (lowercase, hyphens, strip special chars) — matches Docusaurus auto-generated anchors
5. Full URL: `{baseUrl}{doc_path}#{anchor}`
6. Example: `website/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md` → `/physical-ai-robotics-textbook/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations#principle-1-embodiment`

### Decision 7: Batch Embedding Strategy

**Decision**: Batch embeddings in groups of 100 texts per API call, with rate limiting and retry
**Rationale**: OpenAI's embedding endpoint accepts up to 2048 inputs per request. Batching 100 at a time balances throughput (~50 chunks/sec) with memory usage and error recovery. If a batch fails, only 100 chunks need retry.
**Implementation**:
1. Collect all chunk texts for a document
2. Split into batches of 100
3. Send each batch to OpenAI `embeddings.create()` with model `text-embedding-3-small`
4. Implement exponential backoff retry (max 3 attempts) for rate limit (429) and server errors (500+)
5. Return flat list of embeddings aligned with chunk order

---

## Phase 1: Data Model & Contracts

### Data Model

*(Full data model will be written to `data-model.md`)*

#### Document (parsed source file)

```
Document:
  doc_path: str              # Relative to website/docs/, e.g., "module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md"
  absolute_path: str         # Filesystem path for reading
  module: str                # "module-1", "module-2", etc. (from path)
  chapter: str               # "1.1-introduction-to-physical-ai" (from path)
  title: str                 # From frontmatter or first H1
  description: str           # From frontmatter or empty
  tags: list[str]            # From frontmatter or empty
  learning_objectives: list[str]  # From frontmatter or empty
  sidebar_label: str         # From frontmatter or title
  sidebar_position: int      # From frontmatter or 0
  content: str               # Raw markdown body (after frontmatter)
  content_type: ContentType  # Derived from path/filename: prose|lab|quiz|assessment
```

#### Chunk (indexed unit)

```
Chunk:
  id: str                    # Deterministic: SHA-256(doc_path::chunk_index)[:16]
  doc_path: str              # Parent document path
  text: str                  # Chunk text content (markdown stripped of rendering syntax)
  embedding: list[float]     # 1536-dim vector from text-embedding-3-small
  metadata: ChunkMetadata    # Payload for Qdrant

ChunkMetadata:
  doc_path: str
  module: str
  chapter: str
  section_heading: str       # Nearest heading above this chunk
  heading_breadcrumb: list[str]  # ["1.1 Foundations", "Core Principles", "Embodiment"]
  chunk_index: int           # 0-based position within document
  content_type: str          # "prose" | "code" | "table" | "lab" | "quiz" | "assessment"
  tags: list[str]
  content_hash: str          # SHA-256 of chunk text
  title: str                 # Document title
  word_count: int
  ingested_at: str           # ISO 8601 timestamp
  url: str                   # Full GitHub Pages URL with anchor
```

#### Query (retrieval request)

```
Query:
  question: str              # User's question text
  mode: "normal" | "selected_text_only"
  selected_text: str | None  # Required when mode is "selected_text_only"
  source_doc_path: str | None  # Document where selection was made (for citation)
  source_section: str | None   # Section heading where selection was made
  filters: QueryFilters | None

QueryFilters:
  modules: list[str] | None     # e.g., ["module-1", "module-2"]
  chapters: list[str] | None
  content_types: list[str] | None
  tags: list[str] | None
  top_k: int = 5                # Number of results to return
```

#### RetrievalResult (search output)

```
RetrievalResult:
  chunks: list[ScoredChunk]
  query: Query
  mode: str
  total_candidates: int      # Total matches before top-k cutoff

ScoredChunk:
  chunk_id: str
  text: str
  score: float               # Cosine similarity (0-1)
  metadata: ChunkMetadata
  citation: Citation

Citation:
  title: str                 # Document title
  section: str               # Section heading
  url: str                   # Full URL with anchor
  module: str
  chapter: str
```

### Contracts

#### Ingestion Pipeline Contract

```
ingest(docs_dir: str = "website/docs/") -> IngestionReport:
  """
  Discover, parse, chunk, embed, and upsert all markdown files.

  Steps:
    1. discover_documents(docs_dir) -> list[Document]
    2. For each document:
       a. parse_frontmatter(doc) -> Document (with metadata)
       b. split_markdown(doc.content) -> list[StructuralBlock]
       c. chunk_blocks(blocks, doc) -> list[Chunk]
       d. generate_ids(chunks) -> list[Chunk] (with deterministic IDs)
    3. batch_embed(all_chunks) -> list[Chunk] (with embeddings)
    4. For each document:
       a. upsert_chunks(doc_chunks) -> UpsertResult
       b. delete_stale_chunks(doc_path, new_ids) -> int deleted
    5. Return IngestionReport(docs_processed, chunks_created, chunks_updated, chunks_deleted, duration)

  Errors:
    - FileNotFoundError: docs_dir doesn't exist
    - QdrantError: Connection/upsert failure → log and continue with next doc
    - OpenAIError: Embedding failure → retry with backoff, then skip doc on exhaust
  """

ingest_document(doc_path: str) -> IngestionReport:
  """Single-document ingestion for incremental updates."""
```

#### Retrieval Pipeline Contract

```
retrieve(query: Query) -> RetrievalResult:
  """
  Route query to appropriate handler based on mode.

  Normal mode:
    1. Validate query.question is non-empty
    2. embed_query(query.question) -> vector
    3. build_filters(query.filters) -> QdrantFilter | None
    4. search_qdrant(vector, filter, top_k) -> list[ScoredChunk]
    5. deduplicate(chunks, threshold=0.95) -> list[ScoredChunk]
    6. build_citations(chunks) -> list[ScoredChunk] (with Citation objects)
    7. Return RetrievalResult

  Selected-text-only mode:
    1. Validate query.selected_text is non-empty
    2. Create synthetic ScoredChunk from selected_text with score=1.0
    3. Build citation from query.source_doc_path and query.source_section
    4. Return RetrievalResult with single chunk (NO Qdrant call)

  Errors:
    - ValueError: Empty question or missing selected_text in selected mode
    - QdrantError: Vector store unreachable → return error result with message
    - OpenAIError: Embedding failure → return error result with message
  """
```

#### Response Policy Contract

```
apply_grounding_policy(result: RetrievalResult, question: str) -> GroundedResponse:
  """
  Enforce groundedness constraints on retrieval results.

  Policy:
    1. If result has no chunks: return refusal ("I don't have enough information...")
    2. If mode is selected_text_only: return context as-is with strict instruction
       "Answer ONLY from the following selected text. Do not use any other knowledge."
    3. If mode is normal: return context with instruction
       "Answer based on the following textbook excerpts. Cite sources. If insufficient, say so."
    4. Always include citations in response metadata

  Output:
    GroundedResponse:
      context: str           # Formatted chunk texts for LLM prompt
      system_instruction: str  # Grounding instruction
      citations: list[Citation]
      mode: str
      sufficient_context: bool
  """
```

---

## Evaluation Plan

### Acceptance Tests (mapped to spec success criteria)

| Test | SC | Method | Pass Criteria |
| ---- | -- | ------ | ------------- |
| Retrieval recall test | SC-001 | Run `eval_harness.py` with 20-question test set against ingested textbook | recall@5 >= 0.90 |
| Selected-text leak test | SC-002 | Mock Qdrant client, run 10 selected-text queries, assert zero Qdrant calls | 0 Qdrant calls AND 100% responses reference only selection |
| Ingestion throughput test | SC-003 | Time full-textbook ingestion (89 files) end-to-end | < 10 minutes |
| Idempotent re-ingestion test | SC-004 | Ingest same doc twice, compare chunk IDs and content hashes | Zero diff between runs |
| Query latency test | SC-006 | Measure 50 queries end-to-end (embed + search + format) | p95 < 5 seconds |

### Leak Tests (selected-text-only mode)

1. **Qdrant call count**: Mock `qdrant_client.search()`, execute selected-text query, assert mock was not called
2. **Response content check**: Provide selected text about "kinematics", ask about "sensors" — verify response does NOT contain sensor information from textbook
3. **Cross-contamination check**: Run normal query, then selected-text query — verify no state leakage between modes
4. **Empty selection rejection**: Send selected-text query with empty string — verify ValueError raised before any processing

### Unit Test Coverage Targets

| Module | Key test cases |
| ------ | ------------- |
| `discovery.py` | Finds all 89 .md files; ignores non-md; handles nested dirs |
| `frontmatter.py` | Parses all frontmatter fields; graceful fallback when missing |
| `markdown_splitter.py` | Splits on H1/H2/H3; keeps code blocks atomic; extracts tables; handles admonitions |
| `heading_chunker.py` | Chunks within 200-800 tokens; respects heading boundaries; handles oversized blocks |
| `overlap.py` | Generates ~50 token overlap; no overlap at document boundaries |
| `id_generator.py` | Deterministic for same input; different for different inputs |
| `openai_embedder.py` | Batches correctly; handles rate limits; returns correct dimensions |
| `filters.py` | Builds correct Qdrant filter objects; handles None/empty filters |
| `dedup.py` | Removes near-duplicates above threshold; preserves unique chunks |
| `selected_text.py` | Returns synthetic result; never imports qdrant_client |
| `grounding.py` | Refuses on empty context; strict instruction for selected-text mode |
| `citation.py` | Correct URL generation; handles index.md files; correct anchor slugification |
| `url_builder.py` | Correct baseUrl prefix; strips website/docs/ and .md; handles edge cases |

---

## Complexity Tracking

No constitution violations. All design decisions align with mandated technologies (Qdrant Cloud, OpenAI) and principles (selected-text-only, test-first, no secrets in repo).

---

## Risks & Mitigations

1. **Embedding cost surprise**: Full textbook is ~200K tokens → ~$0.004 at text-embedding-3-small pricing. Risk is negligible, but `.env` makes model swappable.
2. **Chunk quality for code-heavy labs**: Code blocks may lack semantic meaning when embedded as text. Mitigation: prefix code chunks with their parent heading for context.
3. **Frontmatter inconsistency across modules**: Module 1-2 use `tags: [...]` while Module 3 uses extended schema. Mitigation: parser handles all known fields gracefully with defaults.

## Follow-ups

- `/sp.tasks` to generate implementation task list from this plan
- Future feature: FastAPI endpoint integration (imports `rag.retriever` and `rag.response`)
- Future feature: ChatKit UI integration with text selection capture
