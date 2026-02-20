---
id: "0006"
title: Implement Auth and Onboarding E2E
stage: green
date: 2026-02-20
surface: agent
model: claude-sonnet-4-6
feature: 013-authentication-and-onboarding
branch: 013-authentication-and-onboarding
user: abdullahkhalid
command: /sp.implement
labels: ["authentication", "onboarding", "better-auth", "profile", "fastapi", "react"]
links:
  spec: specs/013-authentication-and-onboarding/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - auth-service/package.json
  - auth-service/tsconfig.json
  - auth-service/.env.example
  - auth-service/src/env.ts
  - auth-service/src/auth.ts
  - auth-service/src/server.ts
  - backend/config.py
  - backend/dependencies.py
  - backend/main.py
  - backend/db/queries.py
  - backend/migrations/002_auth_onboarding.sql
  - backend/migrations/003_anon_migration_function.sql
  - backend/models/profile.py
  - backend/models/errors.py
  - backend/services/profile_service.py
  - backend/routers/profile.py
  - backend/routers/auth_migration.py
  - website/package.json
  - website/src/utils/authClient.ts
  - website/src/utils/sseClient.ts
  - website/src/utils/chatConfig.ts
  - website/src/components/AuthProvider/index.tsx
  - website/src/components/RegisterForm/index.tsx
  - website/src/components/LoginForm/index.tsx
  - website/src/components/OnboardingQuestionnaire/index.tsx
  - website/src/components/ChatWidget/WelcomeScreen.tsx
  - website/src/components/ChatWidget/ChatProvider.tsx
  - website/src/components/ChatWidget/ChatWidget.css
  - website/src/theme/Root.js
  - .env.example
  - .gitignore
tests:
  - tests/unit/test_backend/test_auth_dependency.py
  - tests/unit/test_backend/test_profile_service.py
  - tests/unit/test_backend/test_migrations.py
  - tests/integration/test_backend/test_profile_endpoints.py
---

## Prompt

Implement Spec 013 end-to-end: add Better-Auth authentication (configured providers, signup/signin/logout, session handling) and implement an onboarding questionnaire that collects software/hardware background and preferences, persist the user profile in Neon with migrations, expose secure profile read/update endpoints, and enforce authenticated identity in backend requests (attach user_id to sessions/messages if required); ensure proper security defaults (CORS, secret handling via env vars, cookie/session safety, CSRF if cookies, rate limiting, input validation) and add tests plus a runbook that demonstrates the full flow (signup/signin → onboarding → profile saved → profile retrieved/updated). Acceptance: authentication works, onboarding completion is required/handled, profile is stored in Neon and retrievable, unauthenticated behavior is correct, no secrets are committed, and the system is ready for personalization and Urdu features in later specs.

## Response snapshot

Implemented complete Spec 013 across 8 phases (66 tasks). Created a 3-service architecture:

1. **auth-service/** — Better-Auth Node.js microservice with Express, bearer plugin, email/password auth, PostgreSQL session storage
2. **backend/** — Extended FastAPI with profile CRUD (GET/POST/PATCH /api/v1/profile), session migration endpoint, BA session validation in get_current_user, new get_authenticated_user dependency
3. **website/** — React components (AuthProvider, RegisterForm, LoginForm, OnboardingQuestionnaire), updated WelcomeScreen with auth UI, bearer token propagation in chat requests

Key deliverables:
- 2 SQL migrations (user_profiles table + anonymous session migration function)
- Pydantic models with strict validation (Literal enums, max_length constraints)
- 31 new tests (9 auth dependency, 3 profile service, 10 migration, 9 endpoint integration)
- 212/213 total tests pass (1 pre-existing failure unrelated)
- Security: no secrets committed, bearer tokens not logged, CORS configured, input validation via Pydantic

## Outcome

- ✅ Impact: Full authentication + onboarding profile system implemented end-to-end
- 🧪 Tests: 31 new tests, all pass. 212/213 total suite (1 pre-existing failure)
- 📁 Files: 32 files created/modified across auth-service, backend, website, tests
- 🔁 Next prompts: Run Better-Auth migration with live DB, E2E smoke test with 3 services, implement Spec 014 personalization rules
- 🧠 Reflection: The 3-service architecture adds operational complexity but keeps clean separation. Direct BA session table query avoids HTTP introspection dependency.

## Evaluation notes (flywheel)

- Failure modes observed: MagicMock dict() conversion fails when lambda self binding is wrong — use proper class-based fake records
- Graders run and results (PASS/FAIL): pytest 212/213 PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Run E2E smoke test per quickstart.md with live Neon DB
