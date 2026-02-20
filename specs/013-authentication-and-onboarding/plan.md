# Implementation Plan: Authentication & Onboarding Profile

**Branch**: `013-authentication-and-onboarding` | **Date**: 2026-02-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/013-authentication-and-onboarding/spec.md`

## Summary

Add authentication (email/password via Better-Auth) and an onboarding questionnaire to the Physical AI textbook. Better-Auth runs as a **separate Node.js microservice** alongside the existing FastAPI backend, both connecting to the same Neon Postgres database. The Docusaurus frontend authenticates via **Bearer tokens** (Better-Auth Bearer plugin), sending tokens to both services. Onboarding profiles are stored in a new `user_profiles` table managed by FastAPI. Anonymous users continue to work without login (backward-compatible).

## Technical Context

**Language/Version**: TypeScript/Node.js 18+ (auth service), Python 3.11 (FastAPI backend), React 18 (Docusaurus frontend)
**Primary Dependencies**: `better-auth` (auth service), `pg` (Node.js Postgres), FastAPI + asyncpg (existing), `better-auth/react` (client)
**Storage**: Neon Postgres (shared — Better-Auth tables + application tables)
**Testing**: pytest (backend), vitest or manual (auth service), Cypress/manual (e2e)
**Target Platform**: GitHub Pages (static frontend), cloud-hosted services (auth + FastAPI)
**Project Type**: Web application (3-service architecture)
**Performance Goals**: Auth operations < 500ms, profile CRUD < 200ms, 500 concurrent authenticated users
**Constraints**: Cross-origin (3 services), no cookies (bearer tokens only), backward-compatible anonymous flow
**Scale/Scope**: ~200-500 users (educational textbook), 6 onboarding fields, 4 new API endpoints

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Documentation-First | PASS | Spec, plan, data-model, contracts all created before implementation |
| II. Selected Text Only Mode | PASS | No impact — auth is orthogonal to RAG answering mode |
| III. Test-First (NON-NEGOTIABLE) | PASS | Test plan defined: migration tests, auth dependency tests, profile endpoint tests, e2e smoke tests |
| IV. Secure Architecture | PASS | No secrets in repo; env vars for DATABASE_URL, BETTER_AUTH_SECRET; passwords hashed by Better-Auth; bearer tokens expire |
| V. Scalable Cloud Infrastructure | PASS | Microservice architecture; shared Neon DB with connection pooling; stateless token validation |
| VI. Modular Component Design | PASS | Auth service is a separate project; profile API in its own router; frontend auth is a self-contained utility module |

**Quality gates**:
- All tests must pass before merge: WILL ENFORCE
- Code coverage minimum 80% for new code: WILL ENFORCE (pytest-cov for backend, profile endpoints)
- Security scan for credentials: WILL ENFORCE (no secrets in repo, .env.example with placeholders)

## Project Structure

### Documentation (this feature)

```text
specs/013-authentication-and-onboarding/
├── plan.md              # This file
├── research.md          # Phase 0 output — Better-Auth research
├── data-model.md        # Phase 1 output — entity schemas
├── quickstart.md        # Phase 1 output — dev setup guide
├── contracts/
│   ├── auth-service-api.md    # Better-Auth endpoints reference
│   └── fastapi-profile-api.md # Profile CRUD + auth integration
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
auth-service/                    # NEW: Better-Auth Node.js microservice
├── package.json
├── tsconfig.json
├── src/
│   ├── auth.ts                  # betterAuth() config (DB, bearer plugin, session, CORS)
│   ├── server.ts                # Express/Node HTTP server mounting auth handler
│   └── env.ts                   # Environment variable validation
├── .env.example
└── README.md

backend/                         # EXISTING: FastAPI backend (modified)
├── config.py                    # ADD: BETTER_AUTH_URL env var
├── dependencies.py              # MODIFY: get_current_user validates BA session table
├── db/
│   ├── queries.py               # ADD: profile CRUD queries, BA session lookup
│   └── migrate.py               # EXISTING: runs new migration files
├── migrations/
│   ├── 001_create_tables.sql    # EXISTING: unchanged
│   ├── 002_auth_onboarding.sql  # NEW: user_profiles table
│   └── 003_anon_migration.sql   # NEW: anonymous session migration function
├── models/
│   ├── profile.py               # NEW: Pydantic models for profile request/response
│   └── errors.py                # MODIFY: add ProfileNotFoundError
├── routers/
│   ├── profile.py               # NEW: GET/POST/PATCH /api/v1/profile
│   └── auth_migration.py        # NEW: POST /api/v1/auth/migrate-sessions
├── services/
│   └── profile_service.py       # NEW: profile business logic
└── main.py                      # MODIFY: register new routers, add BA URL to CORS

website/                         # EXISTING: Docusaurus frontend (modified)
├── package.json                 # MODIFY: add better-auth dependency
└── src/
    ├── utils/
    │   ├── authClient.ts        # NEW: createAuthClient config
    │   └── chatConfig.ts        # MODIFY: attach bearer token to chat requests
    ├── components/
    │   ├── AuthProvider/
    │   │   └── index.tsx         # NEW: React context for auth state
    │   ├── LoginForm/
    │   │   └── index.tsx         # NEW: email/password sign-in form
    │   ├── RegisterForm/
    │   │   └── index.tsx         # NEW: email/password sign-up form
    │   ├── OnboardingQuestionnaire/
    │   │   └── index.tsx         # NEW: onboarding form component
    │   └── ChatWidget/
    │       ├── ChatProvider.tsx  # MODIFY: integrate auth state, attach token
    │       └── WelcomeScreen.tsx # MODIFY: show login/register prompt
    └── theme/
        └── Root.js              # MODIFY: wrap with AuthProvider

tests/                           # EXISTING: test directory (new test files)
├── test_auth_dependency.py      # NEW: test updated get_current_user
├── test_profile_endpoints.py    # NEW: test profile CRUD endpoints
├── test_profile_service.py      # NEW: test profile business logic
├── test_profile_queries.py      # NEW: test profile SQL queries
├── test_session_migration.py    # NEW: test anonymous→authenticated migration
└── test_migrations.py           # MODIFY: add 002, 003 migration tests
```

**Structure Decision**: 3-service web application. Auth service is a minimal Node.js project (Better-Auth + Express). FastAPI backend extended with profile router. Docusaurus frontend adds auth components.

## Architecture Decisions

### AD-1: Better-Auth as Separate Microservice

Better-Auth is a TypeScript library. The existing backend is Python/FastAPI. Running Better-Auth as a separate Node.js service keeps runtimes isolated and follows the constitution's microservice principle (VI).

**Trade-off**: Added operational complexity (3 services to deploy) vs. clean separation and standard Better-Auth usage.

### AD-2: Bearer Tokens (Not Cookies)

Cross-origin architecture (GitHub Pages → Auth Service → FastAPI) makes cookies complex (SameSite, Secure, proxy requirements). Bearer tokens stored in localStorage are simpler to propagate to multiple services.

**Trade-off**: localStorage is vulnerable to XSS (mitigated by CSP headers and no user-generated HTML rendering) vs. cookie complexity and CSRF concerns.

### AD-3: Shared Database, Direct Session Validation

FastAPI validates tokens by querying Better-Auth's `session` table directly in the shared Neon database, rather than making HTTP calls to the auth service. This eliminates a runtime dependency between services.

**Trade-off**: Couples FastAPI to Better-Auth's schema (camelCase columns) vs. added latency and failure modes of HTTP introspection.

### AD-4: Separate user_profiles Table

Onboarding data stored in a dedicated `user_profiles` table (managed by FastAPI migrations) rather than extending Better-Auth's `user` table with `additionalFields`. This decouples profile schema from auth schema.

**Trade-off**: Extra JOIN when fetching user+profile vs. cleaner separation of concerns and independent schema evolution.

### AD-5: Backward-Compatible Anonymous Flow

Existing anonymous users (identified by IP-based tokens) continue to work. The updated `get_current_user` dependency checks Better-Auth sessions first, then falls back to the existing anonymous user creation.

**Trade-off**: Dual auth path adds complexity to the dependency vs. no breaking changes for existing anonymous users.

## Implementation Phases

### Phase 1: Auth Service Setup (foundation)

1. **Create `auth-service/` project**: package.json, tsconfig, .env.example
2. **Configure Better-Auth**: database connection (Neon), email/password enabled, bearer plugin, session config (24h expiry), trusted origins
3. **Create Express server**: mount `auth.handler` at `/api/auth/*`
4. **Run Better-Auth migration**: auto-creates user, session, account, verification tables
5. **Smoke test**: Register user via curl, verify tables populated

### Phase 2: FastAPI Auth Integration (backend)

1. **Migration 002**: Create `user_profiles` table
2. **Migration 003**: Create anonymous session migration function
3. **Update `get_current_user`**: Query BA session table, fall back to anonymous
4. **Add profile Pydantic models**: request/response schemas with validation
5. **Add profile queries**: CRUD operations for user_profiles
6. **Add profile router**: GET/POST/PATCH /api/v1/profile
7. **Add session migration endpoint**: POST /api/v1/auth/migrate-sessions
8. **Update CORS config**: add auth service origin
9. **Register new routers** in main.py

### Phase 3: Frontend Integration (Docusaurus)

1. **Install `better-auth`** npm package in website/
2. **Create `authClient.ts`**: configure createAuthClient with bearer plugin
3. **Create `AuthProvider`**: React context wrapping the app with auth state
4. **Create Login/Register forms**: email/password UI components
5. **Create Onboarding Questionnaire**: form with all 6 fields from spec
6. **Update `ChatProvider`**: attach bearer token to chat API requests
7. **Update `WelcomeScreen`**: show login prompt for unauthenticated users
8. **Update `Root.js`**: wrap app with AuthProvider

### Phase 4: Testing & Validation

1. **Backend unit tests**: auth dependency, profile queries, profile service
2. **Backend integration tests**: profile endpoints, session migration
3. **Migration tests**: verify 002 and 003 apply cleanly
4. **E2E smoke test**: register → onboarding → profile fetch → chat with token
5. **Backward compatibility test**: anonymous chat still works without token

## Security Controls

| Control | Implementation |
|---------|---------------|
| Password hashing | Better-Auth built-in (bcrypt, configurable) |
| Token expiry | 24h session lifetime, configurable via `session.expiresIn` |
| CSRF protection | Better-Auth built-in for its endpoints; FastAPI uses bearer tokens (CSRF-free) |
| CORS | Better-Auth `trustedOrigins`; FastAPI `CORSMiddleware` with explicit origins |
| Rate limiting | Existing sliding window rate limiter applies to all endpoints including profile |
| Input validation | Pydantic models with field constraints (lengths, enums) |
| Error messages | Generic auth errors (no email/account existence leakage) per FR-015 |
| Secrets management | All secrets via env vars; .env.example with placeholders only |
| Logging redaction | Bearer tokens NOT logged; only userId and request metadata |

## Future Extensibility

| Feature | How This Plan Supports It |
|---------|--------------------------|
| Personalization (Spec 014) | `user_profiles` table provides all fields needed for content personalization rules |
| Urdu translation (Spec 015) | `preferred_language` can be added to user_profiles via migration |
| Social login (OAuth) | Better-Auth supports social providers — add to auth.ts config |
| Email verification | Better-Auth has built-in email verification — enable in config |
| Admin dashboard | Better-Auth admin plugin available — add when needed |

## Validation Plan

### DB Migration Test

```python
# tests/test_migrations.py
def test_002_creates_user_profiles_table():
    """Apply migration 002 and verify user_profiles table exists with correct columns."""

def test_003_creates_migration_function():
    """Apply migration 003 and verify migrate_anonymous_sessions function exists."""
```

### Auth Dependency Test

```python
# tests/test_auth_dependency.py
def test_valid_ba_session_returns_identified_user():
    """Given a valid BA session token, get_current_user returns tier='identified'."""

def test_expired_ba_session_falls_back_to_anonymous():
    """Given an expired BA session token, get_current_user creates anonymous user."""

def test_no_token_creates_anonymous_user():
    """Given no Authorization header, get_current_user creates anonymous user."""
```

### Profile Endpoint Tests

```python
# tests/test_profile_endpoints.py
def test_post_profile_creates_and_returns_201():
    """POST /api/v1/profile with valid data creates profile and returns 201."""

def test_get_profile_returns_existing_profile():
    """GET /api/v1/profile returns saved profile data."""

def test_patch_profile_updates_partial_fields():
    """PATCH /api/v1/profile updates only specified fields."""

def test_post_profile_rejects_invalid_software_level():
    """POST /api/v1/profile with invalid enum returns 422."""

def test_profile_requires_authentication():
    """Profile endpoints return 401 without valid bearer token."""
```

### E2E Smoke Test (Manual)

See [quickstart.md](./quickstart.md) for the full runbook:
1. Start all 3 services
2. Register user via curl → verify `set-auth-token` header
3. Create profile via FastAPI → verify 201
4. Fetch profile → verify persistence
5. Chat with token → verify authenticated session
6. Chat without token → verify anonymous still works

## Complexity Tracking

No constitution violations. No complexity justifications needed.
