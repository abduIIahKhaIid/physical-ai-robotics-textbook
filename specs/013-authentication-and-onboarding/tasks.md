# Tasks: Authentication & Onboarding Profile

**Input**: Design documents from `/specs/013-authentication-and-onboarding/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/, research.md, quickstart.md

**Tests**: Included per constitution principle III (Test-First NON-NEGOTIABLE).

**Organization**: Tasks grouped by user story. Constitution requires TDD: tests written first, fail, then implement.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (US1–US5)
- Exact file paths included

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create auth-service project, install dependencies, configure environment

- [X] T001 Create `auth-service/` project scaffold: `auth-service/package.json`, `auth-service/tsconfig.json`, `auth-service/.env.example`
- [X] T002 Install Better-Auth dependencies in auth-service: `cd auth-service && npm init -y && npm install better-auth pg express dotenv`
- [X] T003 [P] Install Better-Auth client in Docusaurus: `cd website && npm install better-auth`
- [X] T004 [P] Create environment variable validation in `auth-service/src/env.ts` — validate DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_PORT, FRONTEND_URL
- [X] T005 [P] Update `.env.example` at repo root — add BETTER_AUTH_URL, BETTER_AUTH_SECRET, BETTER_AUTH_PORT placeholders (no real values)
- [X] T006 [P] Update `backend/config.py` — add `better_auth_url: str` field to `BackendSettings`, load from `BETTER_AUTH_URL` env var

**Run**: `cd auth-service && npm install` · `cd website && npm install`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Better-Auth server, DB migrations, and updated auth dependency — MUST complete before any user story

**CRITICAL**: No user story work can begin until this phase is complete.

### Better-Auth Server

- [X] T007 Create Better-Auth configuration in `auth-service/src/auth.ts` — configure: `database` (pg Pool with DATABASE_URL), `emailAndPassword` (enabled, minPasswordLength: 8), `session` (expiresIn: 86400), `trustedOrigins` (from FRONTEND_URL env), `plugins` ([bearer()]), `secret` (from BETTER_AUTH_SECRET)
- [X] T008 Create Express HTTP server in `auth-service/src/server.ts` — mount `auth.handler` on all `/api/auth/*` routes, add CORS middleware (allow FRONTEND_URL origin), listen on BETTER_AUTH_PORT (default 4000)
- [X] T009 Run Better-Auth migration to auto-create tables: `cd auth-service && npx @better-auth/cli migrate` — verify `user`, `session`, `account`, `verification` tables exist in Neon DB

### Database Migrations (Application)

- [X] T010 Create migration `backend/migrations/002_auth_onboarding.sql` — CREATE TABLE `user_profiles` per data-model.md (id UUID PK, user_id TEXT UNIQUE NOT NULL, software_level VARCHAR(20) with CHECK, programming_languages TEXT, hardware_level VARCHAR(20) with CHECK, available_hardware TEXT[], learning_goal TEXT, preferred_pace VARCHAR(20) with CHECK, onboarding_completed BOOLEAN DEFAULT false, created_at TIMESTAMPTZ, updated_at TIMESTAMPTZ); ALTER TABLE sessions ADD COLUMN ba_user_id TEXT; CREATE INDEX idx_sessions_ba_user_id
- [X] T011 Create migration `backend/migrations/003_anon_migration_function.sql` — CREATE FUNCTION migrate_anonymous_sessions(p_anon_token TEXT, p_ba_user_id TEXT) per data-model.md
- [X] T012 Apply migrations: `python -m backend.db.migrate` — verify user_profiles table and function exist

### Updated Auth Dependency

- [X] T013 Update `backend/dependencies.py` — modify `get_current_user()`: extract Bearer token → query BA `session` table (`SELECT "userId", "expiresAt" FROM session WHERE token = $1 AND "expiresAt" > now()`) → if valid return `{id: userId, tier: "identified", token: token}` → else fall back to existing anonymous user creation. Add new helper `get_authenticated_user()` that returns 401 if not authenticated (no anonymous fallback).
- [X] T014 Add BA session lookup query in `backend/db/queries.py` — new function `get_ba_session_by_token(pool, token) -> Record | None` that queries Better-Auth's `session` table with camelCase column names (double-quoted)
- [X] T015 Update `backend/main.py` — add Better-Auth service URL to CORS `allow_origins` list, register profile and auth_migration routers

### Auth Client (Frontend Foundation)

- [X] T016 Create `website/src/utils/authClient.ts` — configure `createAuthClient` from `better-auth/react` with `baseURL` from env (BETTER_AUTH_URL), `fetchOptions.auth` set to Bearer type reading from `localStorage.getItem("bearer_token")`
- [X] T017 Create `website/src/components/AuthProvider/index.tsx` — React context provider wrapping app, exposes `{user, session, isAuthenticated, isLoading, signIn, signUp, signOut}` using `authClient.useSession()`, stores/clears bearer token in localStorage on auth events

**Checkpoint**: Auth service runs, migrations applied, get_current_user validates BA sessions, frontend auth client configured.

**Verify**: `curl -X POST http://localhost:4000/api/auth/sign-up/email -H "Content-Type: application/json" -d '{"name":"Test","email":"test@example.com","password":"password123"}' -v` returns 200 with `set-auth-token` header.

---

## Phase 3: User Story 1 — New User Registers (Priority: P1) MVP

**Goal**: A visitor can register with email/password and receive a valid bearer token.

**Independent Test**: Register via form → verify token returned → verify token grants access to authenticated endpoints.

### Tests for US1

- [X] T018 [P] [US1] Write test `tests/test_auth_dependency.py::test_valid_ba_session_returns_identified_user` — mock BA session table row with valid token and future expiresAt, assert get_current_user returns tier="identified" and correct userId
- [X] T019 [P] [US1] Write test `tests/test_auth_dependency.py::test_expired_ba_session_falls_back_to_anonymous` — mock BA session with past expiresAt, assert get_current_user returns tier="anonymous"
- [X] T020 [P] [US1] Write test `tests/test_auth_dependency.py::test_no_token_creates_anonymous_user` — no Authorization header, assert get_current_user creates anonymous user with anon- token
- [X] T021 [P] [US1] Write test `tests/test_auth_dependency.py::test_authenticated_user_required_returns_401` — call get_authenticated_user with no token, assert HTTPException 401

### Implementation for US1

- [X] T022 [P] [US1] Create `website/src/components/RegisterForm/index.tsx` — form with name, email, password fields; calls `authClient.signUp.email()`; on success extracts token from `set-auth-token` response header via onSuccess callback, stores in localStorage as `bearer_token`, updates AuthProvider state
- [X] T023 [US1] Update `website/src/theme/Root.js` — wrap app children with `<AuthProvider>` component
- [X] T024 [US1] Update `website/src/components/ChatWidget/WelcomeScreen.tsx` — add "Sign Up" and "Sign In" links/buttons for unauthenticated users, show user email when authenticated

**Run tests**: `pytest tests/test_auth_dependency.py -v`

**Checkpoint**: User can register, receive token, and be recognized as authenticated by FastAPI.

---

## Phase 4: User Story 2 — Returning User Logs In (Priority: P1)

**Goal**: A registered user can log in with email/password and access their sessions.

**Independent Test**: Login → verify token → verify existing sessions accessible.

### Tests for US2

- [X] T025 [P] [US2] Write test `tests/test_session_migration.py::test_migrate_anonymous_sessions_success` — insert anonymous user + sessions in DB, call migration function, assert sessions updated with ba_user_id
- [X] T026 [P] [US2] Write test `tests/test_session_migration.py::test_migrate_no_anonymous_sessions_returns_zero` — call migration with non-existent token, assert returns 0
- [X] T027 [P] [US2] Write test `tests/test_session_migration.py::test_migrate_endpoint_requires_auth` — POST /api/v1/auth/migrate-sessions without token returns 401

### Implementation for US2

- [X] T028 [P] [US2] Create `website/src/components/LoginForm/index.tsx` — form with email, password fields; calls `authClient.signIn.email()`; on success stores bearer token in localStorage, updates AuthProvider state; shows generic error on failure (FR-015)
- [X] T029 [US2] Add session migration query in `backend/db/queries.py` — new function `migrate_anonymous_sessions(pool, anon_token, ba_user_id) -> int` that calls the `migrate_anonymous_sessions` SQL function from migration 003
- [X] T030 [US2] Create `backend/routers/auth_migration.py` — POST `/api/v1/auth/migrate-sessions` endpoint: requires authenticated user (get_authenticated_user), accepts `{"anonymous_token": "anon-..."}`, calls migrate query, returns `{"migrated_count": N}`
- [X] T031 [US2] Update `website/src/utils/authClient.ts` — after successful sign-in, if `sessionStorage` contains anonymous token, call `/api/v1/auth/migrate-sessions` with that token to migrate anonymous sessions
- [X] T032 [US2] Update `website/src/components/ChatWidget/ChatProvider.tsx` — attach Bearer token from localStorage to all chat API requests in Authorization header; store anonymous token in sessionStorage for migration on login

**Run tests**: `pytest tests/test_session_migration.py -v`

**Checkpoint**: User can log in, anonymous sessions migrate to their account, chat requests include Bearer token.

---

## Phase 5: User Story 3 — Onboarding Questionnaire (Priority: P2)

**Goal**: After registration, user completes a questionnaire capturing software/hardware background; profile saved to Neon.

**Independent Test**: Complete questionnaire → verify profile saved in DB → verify profile retrievable via GET endpoint.

### Tests for US3

- [X] T033 [P] [US3] Write test `tests/test_profile_queries.py::test_create_profile_inserts_row` — call create_profile query with valid data, assert row exists in user_profiles with correct fields
- [X] T034 [P] [US3] Write test `tests/test_profile_queries.py::test_get_profile_by_user_id` — insert profile, call get_profile, assert all fields match
- [X] T035 [P] [US3] Write test `tests/test_profile_queries.py::test_create_profile_duplicate_user_fails` — insert profile, attempt second insert for same user_id, assert unique constraint error
- [X] T036 [P] [US3] Write test `tests/test_profile_endpoints.py::test_post_profile_creates_and_returns_201` — POST /api/v1/profile with valid data and auth token, assert 201 and onboarding_completed=true
- [X] T037 [P] [US3] Write test `tests/test_profile_endpoints.py::test_get_profile_returns_existing_profile` — create profile then GET /api/v1/profile, assert all fields match
- [X] T038 [P] [US3] Write test `tests/test_profile_endpoints.py::test_post_profile_rejects_invalid_software_level` — POST with software_level="expert", assert 422
- [X] T039 [P] [US3] Write test `tests/test_profile_endpoints.py::test_post_profile_rejects_long_learning_goal` — POST with learning_goal of 501 chars, assert 422
- [X] T040 [P] [US3] Write test `tests/test_profile_endpoints.py::test_profile_requires_authentication` — GET /api/v1/profile without token, assert 401

### Implementation for US3

- [X] T041 [P] [US3] Create `backend/models/profile.py` — Pydantic models: `ProfileCreate` (software_level: Literal["beginner","intermediate","advanced"], programming_languages: str = "" with max_length=200, hardware_level: Literal["none","hobbyist","academic","professional"], available_hardware: list[Literal[...]] = [], learning_goal: str = "" with max_length=500, preferred_pace: Literal["self_paced","structured_weekly"]), `ProfileResponse` (adds user_id, onboarding_completed, created_at, updated_at), `ProfileUpdate` (all fields Optional)
- [X] T042 [US3] Add profile CRUD queries in `backend/db/queries.py` — functions: `create_profile(pool, user_id, data) -> Record`, `get_profile_by_user_id(pool, user_id) -> Record | None`, `upsert_profile(pool, user_id, data) -> Record`
- [X] T043 [US3] Create `backend/services/profile_service.py` — business logic: `create_or_update_profile(pool, user_id, profile_data) -> dict` (sets onboarding_completed=true, updated_at=now()), `get_user_profile(pool, user_id) -> dict` (returns defaults if no profile exists)
- [X] T044 [US3] Create `backend/routers/profile.py` — endpoints: GET `/api/v1/profile` (get_authenticated_user dependency → get_user_profile), POST `/api/v1/profile` (get_authenticated_user → create_or_update_profile → 201)
- [X] T045 [US3] Add `ProfileNotFoundError` to `backend/models/errors.py` and register handler in `backend/main.py`
- [X] T046 [P] [US3] Create `website/src/components/OnboardingQuestionnaire/index.tsx` — form with 6 fields per FR-010: software_level (radio: beginner/intermediate/advanced), programming_languages (text input, max 200), hardware_level (radio: none/hobbyist/academic/professional), available_hardware (checkbox multi-select: Jetson Nano/Orin, Raspberry Pi, ROS 2 workstation, GPU workstation, Simulation only), learning_goal (textarea, max 500), preferred_pace (radio: self-paced/structured weekly). Skip button saves defaults. Submit calls POST /api/v1/profile with Bearer token.
- [X] T047 [US3] Update `website/src/components/AuthProvider/index.tsx` — after sign-up, check if profile exists (GET /api/v1/profile → onboarding_completed), if false show OnboardingQuestionnaire overlay, on completion redirect to textbook content

**Run tests**: `pytest tests/test_profile_queries.py tests/test_profile_endpoints.py -v`

**Checkpoint**: New user registers → sees questionnaire → submits → profile stored in Neon `user_profiles` table → GET /api/v1/profile returns saved data.

---

## Phase 6: User Story 4 — User Updates Profile (Priority: P3)

**Goal**: Authenticated user can view and edit their onboarding profile at any time.

**Independent Test**: Access profile settings → modify fields → save → verify update persisted.

### Tests for US4

- [X] T048 [P] [US4] Write test `tests/test_profile_queries.py::test_update_profile_partial_fields` — update only software_level, assert other fields unchanged, updated_at changed
- [X] T049 [P] [US4] Write test `tests/test_profile_endpoints.py::test_patch_profile_updates_partial_fields` — PATCH /api/v1/profile with `{"software_level": "advanced"}`, assert 200 with updated field and others unchanged
- [X] T050 [P] [US4] Write test `tests/test_profile_endpoints.py::test_patch_profile_404_without_existing_profile` — PATCH without prior POST, assert 404

### Implementation for US4

- [X] T051 [US4] Add update query in `backend/db/queries.py` — function `update_profile(pool, user_id, **fields) -> Record` that updates only provided fields and sets updated_at=now()
- [X] T052 [US4] Add PATCH endpoint to `backend/routers/profile.py` — PATCH `/api/v1/profile` (get_authenticated_user → validate ProfileUpdate → update_profile → return updated profile; 404 if no existing profile)
- [X] T053 [US4] Create profile settings UI — add "Edit Profile" button/link in `website/src/components/ChatWidget/WelcomeScreen.tsx` for authenticated users; clicking opens OnboardingQuestionnaire pre-filled with current values (GET /api/v1/profile), submit calls PATCH /api/v1/profile

**Run tests**: `pytest tests/test_profile_endpoints.py::test_patch_profile_updates_partial_fields tests/test_profile_endpoints.py::test_patch_profile_404_without_existing_profile -v`

**Checkpoint**: User can view pre-filled profile, update fields, and verify persistence.

---

## Phase 7: User Story 5 — User Logs Out (Priority: P3)

**Goal**: Authenticated user can log out, invalidating their session.

**Independent Test**: Log out → verify token returns 401 on subsequent requests.

### Tests for US5

- [X] T054 [P] [US5] Write test `tests/test_auth_dependency.py::test_invalidated_session_returns_anonymous` — delete BA session row from DB, assert get_current_user with that token falls back to anonymous

### Implementation for US5

- [X] T055 [US5] Add logout handler in `website/src/components/AuthProvider/index.tsx` — call `authClient.signOut()`, clear `bearer_token` from localStorage, clear anonymous token from sessionStorage, reset auth context state to unauthenticated
- [X] T056 [US5] Add "Sign Out" button in `website/src/components/ChatWidget/WelcomeScreen.tsx` — visible when user is authenticated, calls logout handler from AuthProvider context
- [X] T057 [US5] Handle token expiry in `website/src/utils/authClient.ts` — if any API call returns 401, clear localStorage token and redirect to sign-in state (update AuthProvider isAuthenticated to false)

**Run tests**: `pytest tests/test_auth_dependency.py::test_invalidated_session_returns_anonymous -v`

**Checkpoint**: User can log out, token cleared, subsequent requests treated as anonymous.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Security hardening, integration testing, e2e validation

### Security

- [X] T058 Update `backend/main.py` CORS — ensure `allow_origins` includes both Better-Auth URL and frontend URL from env vars; `allow_credentials=True`; `allow_methods=["GET","POST","PATCH","OPTIONS"]`
- [X] T059 [P] Verify `.env.example` files — `auth-service/.env.example` and root `.env.example` contain only placeholder values (no real secrets); add `BETTER_AUTH_SECRET=your-secret-min-32-chars-here`
- [X] T060 [P] Add logging redaction in `backend/dependencies.py` — ensure Bearer token values are NOT logged in any log statement; log only userId and tier

### Integration Tests

- [X] T061 [P] Write test `tests/test_profile_service.py::test_get_user_profile_returns_defaults_when_no_profile` — call get_user_profile for user with no profile, assert default values returned
- [X] T062 [P] Write test `tests/test_profile_service.py::test_create_profile_sets_onboarding_completed` — create profile, assert onboarding_completed=true in returned data
- [X] T063 Write migration test `tests/test_migrations.py::test_002_creates_user_profiles_table` — apply migration 002 to test DB, assert user_profiles table exists with correct columns and constraints
- [X] T064 Write migration test `tests/test_migrations.py::test_003_creates_migration_function` — apply migration 003, assert migrate_anonymous_sessions function callable

### E2E Validation

- [X] T065 Run end-to-end smoke test per `specs/013-authentication-and-onboarding/quickstart.md`:
  1. Start all 3 services: `cd auth-service && npm run dev`, `uvicorn backend.main:app --reload --port 8000`, `cd website && npm start`
  2. Register: `curl -X POST http://localhost:4000/api/auth/sign-up/email -H "Content-Type: application/json" -d '{"name":"Test","email":"test@example.com","password":"password123"}' -v` → verify `set-auth-token` header
  3. Create profile: `curl -X POST http://localhost:8000/api/v1/profile -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" -d '{"software_level":"intermediate","hardware_level":"hobbyist","available_hardware":["jetson_nano_orin"],"preferred_pace":"self_paced"}'` → verify 201
  4. Fetch profile: `curl http://localhost:8000/api/v1/profile -H "Authorization: Bearer $TOKEN"` → verify onboarding_completed=true and fields match
  5. Chat with auth: `curl -X POST http://localhost:8000/api/v1/chat -H "Authorization: Bearer $TOKEN" -H "Content-Type: application/json" -d '{"message":"Hello"}'` → verify authenticated session
  6. Chat anonymous: `curl -X POST http://localhost:8000/api/v1/chat -H "Content-Type: application/json" -d '{"message":"Hello"}'` → verify anonymous still works (backward compat)
  7. DB verify: `SELECT * FROM user_profiles WHERE onboarding_completed = true;` → verify row exists

- [X] T066 Run full test suite and verify coverage: `pytest tests/ -v --cov=backend --cov-report=term-missing` — ensure >=80% coverage on new files (dependencies.py, profile.py, queries.py profile functions)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 — BLOCKS all user stories
- **Phase 3 (US1 Register)**: Depends on Phase 2
- **Phase 4 (US2 Login)**: Depends on Phase 2; can run in parallel with Phase 3
- **Phase 5 (US3 Onboarding)**: Depends on Phase 2; depends on Phase 3 (registration needed first)
- **Phase 6 (US4 Profile Update)**: Depends on Phase 5 (profile must exist)
- **Phase 7 (US5 Logout)**: Depends on Phase 2; can run in parallel with Phases 3–6
- **Phase 8 (Polish)**: Depends on Phases 3–7

### User Story Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundation) ──────────────────────────┐
    ↓              ↓              ↓             ↓
Phase 3 (US1)  Phase 4 (US2)  Phase 7 (US5)   │
 Register        Login          Logout          │
    ↓                                           │
Phase 5 (US3)                                   │
 Onboarding                                     │
    ↓                                           │
Phase 6 (US4)                                   │
 Profile Update                                 │
    ↓──────────────────────────────────────────↓
Phase 8 (Polish + E2E)
```

### Parallel Opportunities

- T003, T004, T005, T006 (all Phase 1 setup) — parallel
- T007, T008 (auth server files) — parallel
- T010, T011 (migration files) — parallel
- T018–T021 (US1 tests) — parallel
- T025–T027 (US2 tests) — parallel
- T033–T040 (US3 tests) — parallel
- T048–T050 (US4 tests) — parallel
- Phase 3 (US1) and Phase 4 (US2) — parallel after Phase 2
- Phase 7 (US5) — parallel with all story phases

---

## Implementation Strategy

**MVP**: Phases 1–3 (Setup + Foundation + Register) — delivers account creation with token auth.

**Increment 2**: Phase 4 (Login) + Phase 5 (Onboarding) — delivers full auth + profile storage.

**Increment 3**: Phase 6 (Profile Update) + Phase 7 (Logout) + Phase 8 (Polish) — complete feature.

---

## Summary

| Metric | Count |
|--------|-------|
| Total tasks | 66 |
| Phase 1 (Setup) | 6 |
| Phase 2 (Foundation) | 11 |
| Phase 3 (US1 Register) | 7 |
| Phase 4 (US2 Login) | 8 |
| Phase 5 (US3 Onboarding) | 15 |
| Phase 6 (US4 Profile Update) | 6 |
| Phase 7 (US5 Logout) | 4 |
| Phase 8 (Polish) | 9 |
| Parallelizable tasks | 33 |
| Test tasks | 22 |
| Implementation tasks | 44 |
