---
id: "0002"
title: "RAG Pipeline Architecture Plan"
stage: plan
date: 2026-02-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 009-rag-ingestion-retrieval
branch: 009-rag-ingestion-retrieval
user: architect
command: /sp.plan
labels: [rag, architecture, qdrant, chunking, retrieval, selected-text-mode, ingestion]
links:
  spec: specs/009-rag-ingestion-retrieval/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/009-rag-ingestion-retrieval/plan.md
  - specs/009-rag-ingestion-retrieval/research.md
  - specs/009-rag-ingestion-retrieval/data-model.md
  - specs/009-rag-ingestion-retrieval/contracts/ingestion.md
  - specs/009-rag-ingestion-retrieval/contracts/retrieval.md
  - specs/009-rag-ingestion-retrieval/quickstart.md
tests:
  - Constitution check: all 6 principles PASS
---

## Prompt

Plan the implementation for Spec 009: produce an end-to-end architecture for ingestion and retrieval including (1) document discovery and parsing approach for Docusaurus docs, (2) chunking algorithm and edge-case handling (code blocks, tables, admonitions), (3) metadata schema and URL/anchor generation compatible with GitHub Pages baseUrl, (4) Qdrant collection configuration and payload indexes for common filters, (5) embedding workflow and batching strategy, (6) retrieval pipeline design (query → embed → Qdrant search → optional filters/rerank → grounded response policy), and (7) selected-text-only mode design and enforcement. Acceptance: the plan is actionable at file level (what scripts/modules to create), isolates scope to ingestion/retrieval policy (no FastAPI/ChatKit integration yet), and includes a concrete evaluation plan with acceptance tests and leak tests. use context7

## Response snapshot

Generated complete implementation plan with 7 research decisions, file-level project structure (rag/ package with 6 submodules + tests/), data model (9 entities), ingestion and retrieval contracts, evaluation plan with 5 acceptance tests and 4 leak tests, and quickstart guide. Key architectural choices: heading-aware chunking (200-800 tokens), OpenAI text-embedding-3-small (1536 dims), single Qdrant collection with 4 payload indexes, complete vector store bypass for selected-text-only mode with import-level isolation.

## Outcome

- ✅ Impact: Full architecture plan covering all 7 requested areas, actionable at file level
- 🧪 Tests: Constitution check passed all 6 principles; evaluation plan covers SC-001 through SC-006
- 📁 Files: plan.md, research.md, data-model.md, contracts/ingestion.md, contracts/retrieval.md, quickstart.md
- 🔁 Next prompts: /sp.tasks to generate implementation task list
- 🧠 Reflection: Codebase analysis of 89 docs revealed consistent frontmatter schema and heading structure, validating heading-aware chunking strategy; baseUrl from docusaurus.config.js critical for citation URL generation

## Evaluation notes (flywheel)

- Failure modes observed: None — plan generated from spec + codebase analysis
- Graders run and results (PASS/FAIL): Constitution check PASS (6/6 principles)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.tasks to produce implementation checklist
