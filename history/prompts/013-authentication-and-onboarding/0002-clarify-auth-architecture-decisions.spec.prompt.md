---
id: "0002"
title: "Clarify Auth Architecture Decisions"
stage: spec
date: 2026-02-19
surface: agent
model: claude-sonnet-4-6
feature: 013-authentication-and-onboarding
branch: 013-authentication-and-onboarding
user: abdulIIahKhaIid
command: /sp.clarify
labels: ["authentication", "better-auth", "architecture", "clarification"]
links:
  spec: specs/013-authentication-and-onboarding/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/013-authentication-and-onboarding/spec.md
tests:
  - N/A (clarification stage)
---

## Prompt

Clarify the auth + onboarding requirements using Better-Auth: confirm where auth lives in this architecture (Docusaurus client + FastAPI backend integration), confirm the authentication method (email/password, OAuth, magic link) and session style (cookie vs bearer token), confirm the minimal onboarding questionnaire fields, confirm the Neon storage model, confirm protected routes/behaviors, confirm token/identity propagation, confirm security basics, and confirm acceptance criteria.

## Response snapshot

Completed 2 of 5 clarification questions before user advanced to /sp.plan:
- Q1: Where does Better-Auth live? → A: Separate Node.js microservice alongside FastAPI, both connecting to shared Neon DB.
- Q2: Session strategy? → A: Bearer tokens via Better-Auth Bearer plugin. Client stores in localStorage, sends via Authorization header.
- Q3 (deferred): ChatKit guest access — assumed guest access with limits per FR-014.

Spec updated with Clarifications section (Session 2026-02-19). FR-003, FR-004 updated to reference Better-Auth. Assumptions updated to reflect Better-Auth's managed tables.

## Outcome

- Impact: 2 critical architecture decisions locked (service topology + session strategy)
- Tests: N/A (clarification stage)
- Files: spec.md updated with clarifications section and inline updates
- Next prompts: /sp.plan (user already invoked)
- Reflection: User moved quickly to planning — 2 high-impact questions were sufficient to unblock

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
