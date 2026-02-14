# Tasks: RAG Ingestion & Retrieval

**Input**: Design documents from `/specs/009-rag-ingestion-retrieval/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Tests**: Included — constitution mandates TDD (Principle III: Test-First NON-NEGOTIABLE).

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1–US4)
- Exact file paths included in all descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and configuration

- [X] T001 Create Python package directory structure: `rag/__init__.py`, `rag/parser/__init__.py`, `rag/chunker/__init__.py`, `rag/embedder/__init__.py`, `rag/store/__init__.py`, `rag/retriever/__init__.py`, `rag/response/__init__.py`, `tests/unit/`, `tests/integration/`, `tests/evaluation/`, `tests/fixtures/`
- [X] T002 Create `pyproject.toml` at repo root with dependencies: `qdrant-client`, `openai`, `python-frontmatter`, `tiktoken`, `pydantic>=2.0`; dev deps: `pytest`, `pytest-asyncio`, `pytest-mock`; entry point `rag.cli:main`
- [X] T003 [P] Create `.env.example` at repo root with placeholder keys: `QDRANT_URL`, `QDRANT_API_KEY`, `OPENAI_API_KEY`, `EMBEDDING_MODEL=text-embedding-3-small`, `QDRANT_COLLECTION=textbook_chunks`; update `.gitignore` to exclude `.env`
- [X] T004 [P] Create `rag/config.py` — load environment variables via `os.environ` with defaults; export `Settings` pydantic model with fields: `qdrant_url`, `qdrant_api_key`, `openai_api_key`, `embedding_model`, `qdrant_collection`, `embedding_dimensions=1536`, `chunk_min_tokens=200`, `chunk_max_tokens=800`, `chunk_target_tokens=500`, `chunk_overlap_tokens=50`, `batch_size=100`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Pydantic models, test fixtures, and shared utilities that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Implement all Pydantic data models in `rag/models.py` per data-model.md: `ContentType` enum, `Document`, `ChunkMetadata`, `Chunk`, `QueryFilters`, `Query`, `Citation`, `ScoredChunk`, `RetrievalResult`, `GroundedResponse`, `IngestionReport`; include validators (e.g., `selected_text` required when `mode="selected_text_only"`)
- [X] T006 [P] Write unit tests for all models in `tests/unit/test_models.py`: test enum values, validator enforcement (selected_text required in selected mode, question non-empty), serialization round-trip, default values
- [X] T007 [P] Implement URL builder in `rag/url_builder.py`: `build_url(doc_path, section_heading)` using `BASE_URL="/physical-ai-robotics-textbook/docs/"`, strip `website/docs/` prefix and `.md` extension, handle `index.md` → parent directory, slugify anchors (lowercase, hyphens, strip specials) per plan Decision 6
- [X] T008 [P] Write unit tests for URL builder in `tests/unit/test_url_builder.py`: test regular doc path, index.md handling, anchor slugification, special characters, empty heading, baseUrl prefix correctness
- [X] T009 [P] Create test fixtures in `tests/fixtures/`: `sample_chapter.md` (prose + code + headings + table), `sample_lab.md` (lab exercise with code blocks), `sample_quiz.md` (quiz with Q&A), `no_frontmatter.md` (file without YAML frontmatter) — modeled after actual textbook content patterns
- [X] T010 [P] Create shared test configuration in `tests/conftest.py`: fixtures for sample documents, mock Qdrant client, mock OpenAI embeddings API, temp directory with sample docs

**Checkpoint**: Foundation ready — models, URL builder, fixtures all tested

---

## Phase 3: User Story 3 — Content Ingestion Pipeline (Priority: P2) 🎯 MVP

**Goal**: Implement the full ingestion pipeline: discover → parse → chunk → embed → upsert into Qdrant

**Note**: US3 is implemented first because US1 (full-book retrieval) and US2 (selected-text) depend on ingested content. US3 is the pipeline that produces the data.

**Independent Test**: Add a new markdown file to `website/docs/`, run `python -m rag.cli ingest`, query Qdrant to confirm content appears with correct metadata.

### Tests for User Story 3

- [X] T011 [P] [US3] Write unit tests for document discovery in `tests/unit/test_discovery.py`: finds all .md files recursively, ignores non-.md, handles nested dirs, raises FileNotFoundError for missing dir, correctly extracts module/chapter/content_type from path
- [X] T012 [P] [US3] Write unit tests for frontmatter parser in `tests/unit/test_frontmatter.py`: parses all frontmatter fields (title, description, tags, learning-objectives, sidebar_label, sidebar_position), fallback when frontmatter missing (title from H1, defaults), handles malformed YAML gracefully
- [X] T013 [P] [US3] Write unit tests for markdown splitter in `tests/unit/test_markdown_splitter.py`: splits on H1/H2/H3 into HEADING blocks, keeps fenced code blocks (``` and ~~~) as atomic CODE blocks, detects pipe tables as TABLE blocks, detects Docusaurus admonitions (:::) as ADMONITION blocks, merges consecutive prose lines, preserves LaTeX math in prose
- [X] T014 [P] [US3] Write unit tests for heading chunker in `tests/unit/test_heading_chunker.py`: chunks within 200–800 token range, never splits across heading boundaries, code blocks kept atomic (oversized → solo chunk), heading_breadcrumb tracks hierarchy correctly, chunk_index is 0-based sequential per doc
- [X] T015 [P] [US3] Write unit tests for overlap generator in `tests/unit/test_overlap.py`: first chunk has no overlap, subsequent chunks get ~50 tokens prepended from previous, no overlap across document boundaries, word_count and content_hash reflect final text
- [X] T016 [P] [US3] Write unit tests for ID generator in `tests/unit/test_id_generator.py`: deterministic (same input → same ID), different inputs → different IDs, ID format is 16-char hex, content_hash is SHA-256 of chunk text
- [X] T017 [P] [US3] Write unit tests for OpenAI embedder in `tests/unit/test_openai_embedder.py`: batches into groups of 100, mock OpenAI API returns correct dimensions (1536), handles rate limit (429) with retry, returns embeddings aligned with chunk order
- [X] T018 [P] [US3] Write unit tests for Qdrant store in `tests/unit/test_qdrant_store.py`: `ensure_collection` creates with correct vector config (1536 dims, cosine), `upsert_chunks` sends correct payloads, `delete_stale_chunks` removes old IDs not in current set, payload indexes created on module/chapter/content_type/tags

### Implementation for User Story 3

- [X] T019 [US3] Implement document discovery in `rag/parser/discovery.py`: `discover_documents(docs_dir)` recursively finds .md files, creates `Document` objects with `doc_path` (relative), `absolute_path`, `module` (from path), `chapter` (from path), `content_type` (from filename pattern: lab/quiz/assessment/prose)
- [X] T020 [US3] Implement frontmatter parser in `rag/parser/frontmatter.py`: `parse_frontmatter(doc)` reads file, extracts YAML via `python-frontmatter`, populates title/description/tags/learning_objectives/sidebar_label/sidebar_position/content; fallback: title from first H1 or filename, empty defaults for optional fields
- [X] T021 [US3] Implement markdown splitter in `rag/parser/markdown_splitter.py`: `split_markdown(content)` returns `list[StructuralBlock]` with `BlockType` enum (HEADING, PROSE, CODE, TABLE, ADMONITION); fenced code blocks kept atomic, pipe tables kept atomic, `:::` admonitions detected, consecutive prose merged, LaTeX preserved
- [X] T022 [US3] Implement heading-aware chunker in `rag/chunker/heading_chunker.py`: `chunk_blocks(blocks, doc, min_tokens, max_tokens, target_tokens)` groups blocks under headings, accumulates to target, emits at heading boundaries, oversized code/table blocks become solo chunks; uses `tiktoken` for token counting with `cl100k_base` encoding
- [X] T023 [US3] Implement overlap generator in `rag/chunker/overlap.py`: `add_overlaps(chunks, overlap_tokens=50)` prepends ~50 tokens from end of previous chunk; skip for first chunk and across document boundaries; recalculate word_count after overlap
- [X] T024 [US3] Implement ID generator in `rag/chunker/id_generator.py`: `generate_ids(chunks)` sets `chunk.id = SHA-256(f"{doc_path}::{chunk_index}")[:16]` and `chunk.metadata.content_hash = SHA-256(chunk.text)`
- [X] T025 [US3] Implement OpenAI embedder in `rag/embedder/openai_embedder.py`: `async batch_embed(chunks, model, batch_size=100, max_retries=3)` calls `openai.embeddings.create()` in batches, exponential backoff retry on 429/5xx, returns chunks with `embedding` field populated (1536-dim vectors)
- [X] T026 [US3] Implement Qdrant collection config in `rag/store/collection_config.py`: define `COLLECTION_CONFIG` with vectors (size=1536, distance=Cosine), HNSW defaults (m=16, ef_construct=100); define `PAYLOAD_INDEXES` list: keyword indexes on `module`, `chapter`, `content_type`, `tags`
- [X] T027 [US3] Implement Qdrant client wrapper in `rag/store/qdrant_client.py`: `ensure_collection()` creates collection if not exists with config from `collection_config.py` and creates payload indexes; `upsert_chunks(chunks)` converts Chunk to Qdrant PointStruct (id, vector, payload); `delete_stale_chunks(doc_path, current_ids)` scrolls for doc_path, deletes IDs not in current set
- [X] T028 [US3] Implement CLI ingestion commands in `rag/cli.py`: `ingest` command (discover → parse → chunk → overlap → IDs → embed → upsert → delete stale → report); `ingest-doc` command for single document; use `click` for CLI framework; print `IngestionReport` summary on completion
- [X] T029 [US3] Write integration test for full ingestion pipeline in `tests/integration/test_ingestion_pipeline.py`: parse → chunk → embed (mocked) → upsert (mocked) end-to-end with fixture files; verify chunk count, metadata correctness, ID determinism, stale deletion
- [X] T030 [US3] Write idempotent re-ingestion test in `tests/integration/test_ingestion_pipeline.py`: ingest same doc twice with mocked services, assert identical chunk IDs and content hashes on both runs (SC-004 acceptance)

**Checkpoint**: `python -m rag.cli ingest --docs-dir website/docs/` runs end-to-end. Verify: `pytest tests/unit/test_discovery.py tests/unit/test_frontmatter.py tests/unit/test_markdown_splitter.py tests/unit/test_heading_chunker.py tests/unit/test_overlap.py tests/unit/test_id_generator.py tests/unit/test_openai_embedder.py tests/unit/test_qdrant_store.py tests/integration/test_ingestion_pipeline.py -v`

---

## Phase 4: User Story 1 — Full-Book Question Answering (Priority: P1)

**Goal**: Implement normal-mode retrieval: embed query → search Qdrant → filter → deduplicate → cite → grounded response

**Independent Test**: Submit 10 questions covering each module, verify retrieved chunks are relevant and answers cite correct chapters.

### Tests for User Story 1

- [X] T031 [P] [US1] Write unit tests for metadata filter builder in `tests/unit/test_filters.py`: single filter (module only), multiple filters (module + content_type), empty/None filters return None, all filter types (modules, chapters, content_types, tags) produce correct Qdrant FieldConditions
- [X] T032 [P] [US1] Write unit tests for deduplication in `tests/unit/test_dedup.py`: removes near-duplicates above 0.95 threshold, preserves unique chunks, maintains score order, handles empty input, handles single chunk
- [X] T033 [P] [US1] Write unit tests for citation builder in `tests/unit/test_citation.py`: `build_citation` produces correct title/section/url/module/chapter, `format_context_with_citations` outputs numbered source format, uses url_builder correctly for index.md files
- [X] T034 [P] [US1] Write unit tests for grounding policy in `tests/unit/test_grounding.py`: empty chunks → refusal (sufficient_context=False), normal mode → citation instruction, selected-text mode → strict instruction "Answer ONLY from selected text", always includes citations list
- [X] T035 [P] [US1] Write integration test for retrieval pipeline in `tests/integration/test_retrieval_pipeline.py`: mock Qdrant search returns fixture chunks, verify embed → search → dedup → cite → grounded response end-to-end, verify citations have correct URLs

### Implementation for User Story 1

- [X] T036 [US1] Implement metadata filter builder in `rag/retriever/filters.py`: `build_qdrant_filter(filters)` converts `QueryFilters` to Qdrant `Filter(must=[...])` with `FieldCondition` + `MatchAny` for each non-None filter field; returns None for empty/None filters
- [X] T037 [US1] Implement near-duplicate filter in `rag/retriever/dedup.py`: `deduplicate(chunks, threshold=0.95)` iterates by score, compares text via token-level Jaccard similarity, skips chunks above threshold
- [X] T038 [US1] Implement citation builder in `rag/response/citation.py`: `build_citation(metadata)` creates `Citation` using `url_builder.build_url()`; `format_context_with_citations(chunks)` returns formatted string with `[Source N: title - section]` headers
- [X] T039 [US1] Implement grounding policy in `rag/response/grounding.py`: `apply_grounding_policy(result, question)` returns `GroundedResponse` with mode-specific `system_instruction`, formatted context, citations list, and `sufficient_context` flag
- [X] T040 [US1] Implement normal-mode query engine in `rag/retriever/query_engine.py`: `retrieve(query)` routes by mode; `_retrieve_normal(query)` embeds question → builds filter → searches Qdrant → deduplicates → builds citations → returns `RetrievalResult`; add Qdrant search call using `qdrant_client.search()` with vector, filter, top_k
- [X] T041 [US1] Implement CLI query command in `rag/cli.py`: `query` subcommand accepts question string, `--top-k`, `--module`, `--chapter`, `--content-type` filter flags; prints ranked results with scores and citations

**Checkpoint**: `python -m rag.cli query "What are the core principles of Physical AI?" --top-k 5` returns relevant chunks with citations. Verify: `pytest tests/unit/test_filters.py tests/unit/test_dedup.py tests/unit/test_citation.py tests/unit/test_grounding.py tests/integration/test_retrieval_pipeline.py -v`

---

## Phase 5: User Story 2 — Selected Text Only Mode (Priority: P1)

**Goal**: Implement selected-text-only retrieval that completely bypasses Qdrant and uses only the provided selection as context

**Independent Test**: Select a paragraph, ask a question, verify response references ONLY the selected content. Run leak tests to confirm zero Qdrant calls.

### Tests for User Story 2

- [X] T042 [P] [US2] Write unit tests for selected-text retriever in `tests/unit/test_selected_text.py`: returns synthetic `RetrievalResult` with score=1.0, validates non-empty selected_text (raises ValueError on empty/None), MUST NOT import `qdrant_client` or any `rag.store` module (verify via `inspect.getmodule` or import check), citation uses source_doc_path/source_section from query
- [X] T043 [P] [US2] Write leak test in `tests/integration/test_selected_text_leak.py`: (1) mock `qdrant_client.QdrantClient` globally, run 10 selected-text queries, assert mock `.search()` called 0 times; (2) provide selected text about "kinematics", ask about "sensors", verify response has NO sensor content; (3) run normal query then selected-text query, verify no state leakage; (4) empty selection → ValueError before any processing

### Implementation for User Story 2

- [X] T044 [US2] Implement selected-text retriever in `rag/retriever/selected_text.py`: `retrieve_from_selection(query)` validates `selected_text` non-empty, creates synthetic `ScoredChunk` (id="selection", text=selected_text, score=1.0, metadata from query source fields), builds citation, returns `RetrievalResult(mode="selected_text_only")`; MUST NOT import any `rag.store` or `rag.embedder` module
- [X] T045 [US2] Wire selected-text mode routing in `rag/retriever/query_engine.py`: update `retrieve(query)` to check `query.mode == "selected_text_only"` BEFORE any Qdrant imports; route to `selected_text.retrieve_from_selection(query)`; add validation that `selected_text` is non-empty when mode is selected
- [X] T046 [US2] Update grounding policy in `rag/response/grounding.py` for selected-text mode: when `mode == "selected_text_only"`, set `system_instruction` to: "Answer ONLY from the following selected text. Do not use any other knowledge. If the text does not contain enough information to answer, say so."
- [X] T047 [US2] Update CLI query command in `rag/cli.py`: add `--selected-text` and `--mode` flags; when `--mode selected_text_only`, pass selected_text and source metadata to `Query` object

**Checkpoint**: `python -m rag.cli query "Explain this" --mode selected_text_only --selected-text "Physical AI systems must operate in real-time."` returns answer from selection only. Verify: `pytest tests/unit/test_selected_text.py tests/integration/test_selected_text_leak.py -v` — all pass, zero Qdrant calls confirmed.

---

## Phase 6: User Story 4 — Retrieval Quality Diagnostics (Priority: P3)

**Goal**: Build an evaluation harness with curated test set to measure retrieval quality

**Independent Test**: Run `python -m rag.cli evaluate` and verify structured report with per-question precision/recall/MRR scores.

### Tests for User Story 4

- [X] T048 [P] [US4] Write unit tests for evaluation harness in `tests/evaluation/test_eval_harness.py`: computes precision@5 correctly, computes recall@5 correctly, computes MRR correctly, handles zero-result queries, produces structured JSON report, flags low-scoring questions

### Implementation for User Story 4

- [X] T049 [US4] Create curated test set in `tests/evaluation/test_set.json`: 20 question-answer-expected_chunks triples covering all 4 modules; include single-module questions, cross-module questions, and one out-of-scope question; each entry has `question`, `expected_doc_paths` (list), `expected_sections` (list)
- [X] T050 [US4] Implement evaluation harness in `tests/evaluation/eval_harness.py`: `run_evaluation(test_set_path, retriever)` loads test set JSON, runs each query through retriever, computes precision@5, recall@5, MRR per question; outputs structured report with per-question scores and aggregates; flags questions with recall@5 < 0.5
- [X] T051 [US4] Add `evaluate` CLI command in `rag/cli.py`: `evaluate` subcommand accepts `--test-set` path (default `tests/evaluation/test_set.json`), runs harness, prints summary table and overall metrics, exits with code 1 if aggregate recall@5 < 0.90 (SC-001)

**Checkpoint**: `python -m rag.cli evaluate --test-set tests/evaluation/test_set.json` prints report. Verify: `pytest tests/evaluation/test_eval_harness.py -v`

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Validation, documentation, and cleanup

- [X] T052 Run full unit test suite: `pytest tests/unit/ -v` — all tests pass
- [X] T053 Run full integration test suite: `pytest tests/integration/ -v` — all pass, including leak tests
- [X] T054 [P] Validate ingestion on real textbook: run `python -m rag.cli ingest --docs-dir website/docs/` against Qdrant (or mock), verify 89 docs processed, ~1500–3000 chunks created, no errors
- [X] T055 [P] Validate retrieval on real data: run 5 sample queries via CLI, verify relevant chunks returned with correct citations and valid GitHub Pages URLs
- [X] T056 [P] Validate idempotent re-ingestion: run ingestion twice on same content, verify zero new/modified chunks (SC-004)
- [X] T057 [P] Validate selected-text-only leak prevention: run `pytest tests/integration/test_selected_text_leak.py -v`, confirm all 4 leak tests pass (SC-002)
- [X] T058 Run `quickstart.md` end-to-end validation: follow all steps in `specs/009-rag-ingestion-retrieval/quickstart.md`, verify commands work as documented

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 completion — BLOCKS all user stories
- **US3 Ingestion (Phase 3)**: Depends on Phase 2 — must complete before US1 (retrieval needs data)
- **US1 Retrieval (Phase 4)**: Depends on Phase 2 + Phase 3 (needs ingested data to test against)
- **US2 Selected-Text (Phase 5)**: Depends on Phase 2 only (no Qdrant dependency by design) — CAN run in parallel with Phase 3/4
- **US4 Evaluation (Phase 6)**: Depends on Phase 3 + Phase 4 (needs ingestion + retrieval working)
- **Polish (Phase 7)**: Depends on all user story phases

### User Story Dependencies

- **US3 (P2 — implemented first)**: Foundational only. Produces data for US1/US4.
- **US1 (P1)**: Foundational + US3 (needs ingested chunks in Qdrant)
- **US2 (P1)**: Foundational only. Zero dependency on Qdrant — can be built in parallel with US3.
- **US4 (P3)**: Foundational + US3 + US1 (evaluation harness tests retrieval quality)

### Parallel Opportunities

Within Phase 2: T005–T010 all parallelizable (different files)
Within Phase 3 tests: T011–T018 all parallelizable
Within Phase 4 tests: T031–T035 all parallelizable
Phase 5 (US2) can run entirely in parallel with Phase 3 (US3) since US2 never touches Qdrant

---

## Parallel Example: Phase 3 (Ingestion)

```bash
# Step 1: Write all tests in parallel (T011–T018) — all marked [P]
# Step 2: Verify tests fail (TDD red phase)
pytest tests/unit/test_discovery.py tests/unit/test_frontmatter.py tests/unit/test_markdown_splitter.py tests/unit/test_heading_chunker.py -v  # expect FAIL

# Step 3: Implement modules sequentially (T019–T028)
# T019–T021 (parser modules) can be parallelized
# T022–T024 (chunker modules) can be parallelized after parser
# T025–T027 (embedder + store) can be parallelized
# T028 (CLI) last — orchestrates all modules

# Step 4: Verify tests pass (TDD green phase)
pytest tests/unit/ tests/integration/test_ingestion_pipeline.py -v  # expect PASS
```

---

## Implementation Strategy

1. **MVP**: Phase 1 + Phase 2 + Phase 3 (ingestion pipeline) = data in Qdrant
2. **Core retrieval**: Phase 4 (normal query) = end-to-end Q&A working
3. **Critical differentiator**: Phase 5 (selected-text mode) = isolation enforced
4. **Quality assurance**: Phase 6 (evaluation harness) = measurable recall
5. **Release validation**: Phase 7 (polish) = all acceptance criteria verified

**Total tasks**: 58
**Per-story**: US3=20, US1=11, US2=6, US4=4, Setup=4, Foundation=6, Polish=7
**Parallel opportunities**: 30 tasks marked [P]
**Suggested MVP scope**: Phases 1–3 (US3 ingestion) — delivers indexed textbook
