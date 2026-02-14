---
id: "0004"
title: "Implement RAG Ingestion Retrieval Pipeline"
stage: green
date: 2026-02-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 009-rag-ingestion-retrieval
branch: 009-rag-ingestion-retrieval
user: abdulIIahKhaIid
command: /sp.implement
labels: ["rag", "ingestion", "retrieval", "qdrant", "selected-text", "evaluation"]
links:
  spec: specs/009-rag-ingestion-retrieval/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - rag/parser/discovery.py
  - rag/parser/frontmatter.py
  - rag/parser/markdown_splitter.py
  - rag/chunker/heading_chunker.py
  - rag/chunker/overlap.py
  - rag/chunker/id_generator.py
  - rag/embedder/openai_embedder.py
  - rag/store/collection_config.py
  - rag/store/qdrant_store.py
  - rag/retriever/filters.py
  - rag/retriever/dedup.py
  - rag/retriever/selected_text.py
  - rag/retriever/query_engine.py
  - rag/response/citation.py
  - rag/response/grounding.py
  - rag/eval/eval_harness.py
  - rag/eval/test_set.json
  - rag/cli.py
  - pyproject.toml
tests:
  - tests/unit/test_models.py
  - tests/unit/test_url_builder.py
  - tests/unit/test_discovery.py
  - tests/unit/test_frontmatter.py
  - tests/unit/test_markdown_splitter.py
  - tests/unit/test_heading_chunker.py
  - tests/unit/test_overlap.py
  - tests/unit/test_id_generator.py
  - tests/unit/test_filters.py
  - tests/unit/test_dedup.py
  - tests/unit/test_selected_text.py
  - tests/unit/test_grounding.py
  - tests/unit/test_citation.py
  - tests/unit/test_openai_embedder.py
  - tests/unit/test_eval_harness.py
  - tests/integration/test_selected_text_leak.py
---

## Prompt

Implement Spec 009 end-to-end: build the ingestion and retrieval system to index Docusaurus markdown/MDX into Qdrant by implementing chunking (heading-based with safe handling for code/tables), metadata attachment (doc_id/path/title/heading_path/section/url/lang/module/week/content_type), embedding + batched upsert into a Qdrant collection with payload indexes for common filters, and retrieval functions that return grounded answers with traceability; implement strict "selected text only mode" that bypasses Qdrant retrieval and answers only using the provided selection or refuses when insufficient; and add an evaluation harness with acceptance tests and leak tests that demonstrate correct indexing, correct retrieval, and no outside-text leakage in selection-only mode. Acceptance: indexing runs successfully, retrieval returns relevant grounded chunks, selection-only mode is strictly enforced, evaluation tests pass, and the module is ready for FastAPI integration in Spec 010.

## Response snapshot

Implemented all 58 tasks across 7 phases of Spec 009. Created 18 implementation files and 16 test files. The RAG pipeline supports: document discovery with content type detection, YAML frontmatter parsing with fallbacks, heading-aware structural markdown splitting (atomic code/table/admonition blocks), configurable chunking (200-800 tokens with tiktoken cl100k_base), overlap generation, deterministic SHA-256 chunk IDs, async batch OpenAI embedding with retry, Qdrant upsert/search/stale-cleanup, metadata filter building, token-level Jaccard deduplication, strict selected-text-only mode (zero Qdrant imports), grounding policy enforcement, citation formatting, and an offline evaluation harness with 20 test queries including 4 leak tests. All 110 tests pass with 70% code coverage. Fixed pyproject.toml build backend and source-level import isolation for leak tests.

## Outcome

- Impact: Complete RAG ingestion and retrieval system ready for FastAPI integration in Spec 010
- Tests: 110 passed, 0 failed, 70% coverage (uncovered: CLI entry points, live Qdrant/OpenAI calls)
- Files: 18 implementation files, 16 test files, 4 test fixtures, eval test set (20 queries)
- Next prompts: /sp.implement for Spec 010 (FastAPI chat endpoint integration)
- Reflection: Subagents failed with 400 errors; direct implementation was faster and more reliable

## Evaluation notes (flywheel)

- Failure modes observed: 3 initial test failures (docstring containing "qdrant_client" tripped import isolation test; index.md URL trailing slash mismatch) — all fixed
- Graders run and results (PASS/FAIL): pytest 110/110 PASS, checklist 16/16 PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Add integration tests for CLI commands using click.testing.CliRunner
