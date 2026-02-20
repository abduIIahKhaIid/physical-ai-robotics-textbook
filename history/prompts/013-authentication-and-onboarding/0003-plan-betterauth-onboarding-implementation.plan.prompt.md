---
id: "0003"
title: "Plan Better-Auth Onboarding Implementation"
stage: plan
date: 2026-02-19
surface: agent
model: claude-sonnet-4-6
feature: 013-authentication-and-onboarding
branch: 013-authentication-and-onboarding
user: abdulIIahKhaIid
command: /sp.plan
labels: ["authentication", "better-auth", "onboarding", "architecture", "planning"]
links:
  spec: specs/013-authentication-and-onboarding/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/013-authentication-and-onboarding/plan.md
  - specs/013-authentication-and-onboarding/research.md
  - specs/013-authentication-and-onboarding/data-model.md
  - specs/013-authentication-and-onboarding/quickstart.md
  - specs/013-authentication-and-onboarding/contracts/auth-service-api.md
  - specs/013-authentication-and-onboarding/contracts/fastapi-profile-api.md
tests:
  - N/A (plan stage — test plan defined, not executed)
---

## Prompt

Plan the implementation for Better-Auth + onboarding: define the end-to-end auth architecture (frontend pages/components, backend verification/middleware, session/token handling), define the onboarding UX flow (first login → questionnaire → saved profile → redirect), define the Neon schema and migrations (users + user_profiles with typed fields and timestamps), define API endpoints needed (auth callbacks/verification if applicable, profile GET/POST/PATCH), define how authenticated requests reach the FastAPI chat endpoints (dependency that validates identity and attaches user_id), define rate-limits and security controls, and define validation (local dev runbook, DB migration test, auth smoke tests).

## Response snapshot

Created comprehensive implementation plan with 6 artifacts:
- **plan.md**: 4-phase implementation (Auth Service Setup → FastAPI Integration → Frontend Integration → Testing), 5 architecture decisions (AD-1 through AD-5), security controls matrix, future extensibility table
- **research.md**: 6 research topics resolved via Context7 (Better-Auth schema, bearer plugin, React client, additional fields, CORS, session validation)
- **data-model.md**: Entity schema for Better-Auth managed tables (user, session, account, verification) + application table (user_profiles), migration SQL, state transitions, validation rules
- **contracts/auth-service-api.md**: Better-Auth endpoint reference + server/client configuration
- **contracts/fastapi-profile-api.md**: 4 new endpoints (GET/POST/PATCH profile + migrate-sessions), updated get_current_user dependency, full sequence diagram
- **quickstart.md**: Local dev setup, smoke test curl commands, DB verification SQL

Key architectural decisions: Better-Auth as separate Node.js microservice (AD-1), bearer tokens not cookies (AD-2), shared DB with direct session validation (AD-3), separate user_profiles table (AD-4), backward-compatible anonymous flow (AD-5).

## Outcome

- Impact: Complete implementation plan ready for task decomposition
- Tests: Test plan defined (migration, auth dependency, profile endpoints, e2e smoke)
- Files: 6 plan artifacts created
- Next prompts: /sp.tasks to generate task breakdown
- Reflection: Context7 research on Better-Auth schema was critical for data model accuracy — camelCase column naming affects FastAPI query design

## Evaluation notes (flywheel)

- Failure modes observed: Research subagent did not write research.md (had to create manually)
- Graders run and results (PASS/FAIL): Constitution check PASS (all 6 principles)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Give research subagent explicit write instructions with exact path
