---
id: "0003"
title: "Switch OpenAI to Gemini Plan Update"
stage: plan
date: 2026-02-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 010-fastapi-chat-backend
branch: 010-fastapi-chat-backend
user: codespace
command: user-request
labels: ["gemini", "openai-migration", "embeddings", "llm-provider"]
links:
  spec: specs/010-fastapi-chat-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-fastapi-chat-backend/plan.md
  - specs/010-fastapi-chat-backend/research.md
  - specs/010-fastapi-chat-backend/quickstart.md
  - specs/010-fastapi-chat-backend/spec.md
tests:
  - none
---

## Prompt

in 010-fastapi-chat-backend we want to use opensource llm like gemini for embedding and text also update the plan accordingly

## Response snapshot

Updated all plan artifacts to replace OpenAI with Google Gemini throughout the stack:
- **research.md R3**: Rewrote LLM/embedding decision — Gemini `gemini-2.0-flash` for text generation, `gemini-embedding-001` for embeddings (768d), `google-genai` SDK. Documented impact on spec 009 RAG pipeline (5 files need refactoring).
- **plan.md**: Updated Technical Context (dependencies), runtime flow diagram, DI pattern, configuration section (new `BackendSettings` with Gemini fields), spec 009 integration section (detailed refactoring table for 5 rag/ files), added Gemini migration tests, added complexity tracking entry for constitution deviation, added risk #4 for Qdrant re-ingestion.
- **quickstart.md**: Updated env vars from `OPENAI_API_KEY` to `GEMINI_API_KEY`, model defaults to `gemini-embedding-001` and `gemini-2.0-flash`.
- **spec.md**: Updated assumptions and dependencies to reference Gemini instead of OpenAI.
- Key breaking change: Qdrant collection must be recreated with 768-dim vectors (was 1536) and all content re-ingested.

## Outcome

- ✅ Impact: Full provider switch from OpenAI to Gemini across all plan artifacts, with detailed migration plan for spec 009
- 🧪 Tests: Added 3 Gemini migration test entries to validation plan
- 📁 Files: 4 artifacts updated (plan.md, research.md, quickstart.md, spec.md)
- 🔁 Next prompts: /sp.tasks to generate implementation tasks including Gemini migration
- 🧠 Reflection: Gemini switch is bigger than just the backend — it requires refactoring the existing RAG pipeline (spec 009) since both ingestion and query-time embedding must use the same model

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution deviation documented in Complexity Tracking
- Prompt variant (if applicable): ad-hoc user request during plan phase
- Next experiment (smallest change to try): Proceed to /sp.tasks
