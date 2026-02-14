# Research: RAG Ingestion & Retrieval

**Feature**: 009-rag-ingestion-retrieval
**Date**: 2026-02-14

## Decision Log

### D1: Embedding Model → OpenAI text-embedding-3-small (1536 dims)

**Decision**: OpenAI `text-embedding-3-small`
**Rationale**: Constitution mandates OpenAI. Best cost/quality for English educational text at $0.02/1M tokens. 1536 dims matches Qdrant HNSW defaults.
**Alternatives considered**:
- `text-embedding-3-large` (3072 dims): 2x cost/storage, marginal quality gain for <10K vectors
- `text-embedding-ada-002`: Legacy, superseded
- Open-source (SBERT): No constitution alignment, requires self-hosting

### D2: Chunking → Heading-aware hierarchical, 200–800 tokens

**Decision**: Split on H1/H2/H3 boundaries, accumulate blocks to ~500 token target, 50-token overlap
**Rationale**: Textbook has consistent heading structure across 89 files. Heading-based splitting preserves semantic coherence. Token-based limits ensure embedding quality.
**Alternatives considered**:
- `RecursiveCharacterTextSplitter`: Ignores semantic boundaries
- Sentence-level: Too granular, loses section context
- Full-section (one chunk per H2): Some sections exceed 2000 tokens

### D3: Qdrant Collection → Single collection, cosine, HNSW, 4 payload indexes

**Decision**: `textbook_chunks` collection with cosine distance, keyword indexes on module/chapter/content_type/tags
**Rationale**: ~3K vectors fits in memory. Payload indexes enable FR-014 filtered search. Single collection simplifies ops.
**Alternatives considered**:
- Per-module collections: Adds complexity, no performance benefit at this scale
- Dot product distance: Requires normalized vectors, cosine handles this automatically

### D4: Chunk IDs → SHA-256(doc_path::chunk_index)[:16]

**Decision**: Deterministic hex ID from path + position
**Rationale**: Enables idempotent upserts (FR-004). Content hash stored in payload for change detection.
**Alternatives considered**:
- UUID: Not deterministic, breaks idempotency
- Content-hash-based ID: Chunk reordering changes all downstream IDs

### D5: Selected-Text Mode → Complete Qdrant bypass

**Decision**: No vector search in selected-text mode. Synthetic RetrievalResult from provided text.
**Rationale**: Constitution Principle II requires strict isolation. Bypassing Qdrant entirely eliminates any leakage vector. Leak tests verify zero Qdrant calls.
**Alternatives considered**:
- Embed selected text and search for similar chunks: Violates spec FR-010 ("MUST NOT retrieve additional chunks")
- Filter by doc_path only: Still returns non-selected content

### D6: URL Generation → baseUrl-aware with anchor slugification

**Decision**: `/physical-ai-robotics-textbook/docs/{doc_path}#{anchor}` matching Docusaurus conventions
**Rationale**: GitHub Pages serves at this baseUrl. Docusaurus auto-generates anchors by slugifying headings.
**Alternatives considered**:
- Relative URLs: Break when chatbot is embedded on different pages
- Full absolute URLs: Correct but the domain may change; use path-based with configurable base

### D7: Batch Embedding → Groups of 100 with exponential backoff

**Decision**: 100 texts per API call, 3 retries with backoff
**Rationale**: Balances throughput (~50 chunks/sec) with error granularity. OpenAI supports up to 2048 per call.
**Alternatives considered**:
- Single text per call: Too slow (~3 calls/sec rate limit)
- 2048 per call: One failure retries everything
