---
id: "0004"
title: "FastAPI Chat Backend Tasks Generation"
stage: tasks
date: 2026-02-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: 010-fastapi-chat-backend
branch: 010-fastapi-chat-backend
user: codespace
command: /sp.tasks
labels: ["tasks", "fastapi", "tdd", "gemini-migration", "selected-text-isolation"]
links:
  spec: specs/010-fastapi-chat-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-fastapi-chat-backend/tasks.md
tests:
  - none
---

## Prompt

Create an atomic tasks checklist to implement Spec 010: add tasks to scaffold the FastAPI project structure (app entry, routers, settings), implement core endpoints (/health, /chat, optional /sessions and /sessions/{id}), define Pydantic schemas including selected_text and mode fields, implement Neon schema migrations for chat_sessions and chat_messages (and optional users/profile tables if in scope), implement DB access layer and persistence in chat flow, integrate Spec 009 retrieval functions for normal mode and implement server-enforced selection-only mode (bypass Qdrant and refuse when selection lacks answer), implement streaming if chosen (SSE or streaming response) with client-friendly framing, add security basics (CORS config, env var validation, request body limits, logging redaction) and rate limiting (simple in-memory/dev + documented production option), and add tests (API contract tests, DB persistence tests, selection-only leak tests). Acceptance: tasks list exact file paths and commands (run server, run migrations, run tests) and end with validation proving endpoints work, messages persist, and selection-only mode is strictly enforced. use context7

## Response snapshot

Generated 48 tasks across 8 phases organized by user story:
- Phase 1 (Setup): 5 tasks — scaffold backend/, test dirs, pyproject.toml, .env.example, config
- Phase 2 (Foundational): 15 tasks — Gemini migration (T006-T012: refactor rag/ from OpenAI to google-genai, 768d vectors), DB schema (T013-T015: migration SQL, pool, runner), core models (T016-T017: Pydantic + errors), queries (T018), dependencies (T019), app factory (T020)
- Phase 3 (US1 Chat): 6 tasks — TDD with 3 test tasks + chat_service + chat router + verify
- Phase 4 (US2 Selected-Text): 5 tasks — isolation tests + verify zero Qdrant/Gemini access
- Phase 5 (US3 Sessions): 5 tasks — persistence tests + session service + session router
- Phase 6 (US4 Rate Limiting): 4 tasks — rate limiter tests + implementation + integration
- Phase 7 (US5 Health): 3 tasks — health tests + implementation
- Phase 8 (Polish): 5 tasks — error handling, logging, CORS, full coverage run, quickstart validation

## Outcome

- ✅ Impact: 48 atomic, file-level tasks with TDD approach, exact paths, and validation commands
- 🧪 Tests: 12 test tasks distributed across user stories (written first per TDD)
- 📁 Files: tasks.md created at specs/010-fastapi-chat-backend/tasks.md
- 🔁 Next prompts: /sp.analyze for cross-artifact consistency, or /sp.implement to begin execution
- 🧠 Reflection: Gemini migration (Phase 2a) is the highest-risk foundational task since it modifies the existing passing test suite. T012 gate ensures all 110+ tests still pass before proceeding.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation — all 48 tasks follow checkbox + ID + [P]/[Story] + file path format
- Prompt variant (if applicable): standard /sp.tasks flow
- Next experiment (smallest change to try): Run /sp.implement to begin Phase 1 execution
