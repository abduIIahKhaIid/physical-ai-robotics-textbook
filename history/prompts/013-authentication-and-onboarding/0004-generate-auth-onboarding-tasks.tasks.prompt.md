---
id: "0004"
title: "Generate Auth Onboarding Tasks"
stage: tasks
date: 2026-02-19
surface: agent
model: claude-sonnet-4-6
feature: 013-authentication-and-onboarding
branch: 013-authentication-and-onboarding
user: abdulIIahKhaIid
command: /sp.tasks
labels: ["authentication", "onboarding", "tasks", "better-auth"]
links:
  spec: specs/013-authentication-and-onboarding/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/013-authentication-and-onboarding/tasks.md
tests:
  - N/A (task generation stage)
---

## Prompt

Create an atomic tasks checklist to implement auth + onboarding: add tasks to set up Better-Auth configuration (providers, env vars, callbacks) and add UI pages/components for signup/signin, add tasks to create onboarding questionnaire UI and client-side validation, add tasks to create Neon migrations for users and user_profiles tables, add tasks to implement backend profile endpoints (create/read/update) and auth enforcement (middleware/dependency that extracts user_id), add tasks to wire authenticated identity into chat requests (store user_id on sessions/messages if in scope), add tasks to implement logout and session expiry handling, add tasks for security (CORS, cookie flags, CSRF if needed, secrets management, rate limits), and add tasks for tests (signup/signin flow, onboarding save, profile fetch, unauthenticated access behavior). Acceptance: tasks list exact file paths and commands to run (migrations, tests, run dev), and end with an end-to-end checklist confirming login to onboarding to profile stored in Neon works.

## Response snapshot

Generated 66 tasks across 8 phases organized by user story priority. 22 test tasks (TDD per constitution), 44 implementation tasks, 33 parallelizable. Phases: Setup (6) → Foundation (11) → US1 Register (7) → US2 Login (8) → US3 Onboarding (15) → US4 Profile Update (6) → US5 Logout (4) → Polish/E2E (9). All tasks include exact file paths, commands, and curl-based smoke tests. E2E validation task (T065) covers full flow: register → onboarding → profile stored → chat with auth → anonymous backward compat.

## Outcome

- Impact: Complete task breakdown ready for implementation via /sp.implement
- Tests: 22 test tasks defined spanning auth dependency, profile queries, profile endpoints, session migration, and migrations
- Files: tasks.md created
- Next prompts: /sp.analyze then /sp.implement
- Reflection: Organizing by user story makes dependencies explicit and MVP scope clear (Phases 1-3)

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS (all 66 tasks follow checkbox+ID+path format)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
