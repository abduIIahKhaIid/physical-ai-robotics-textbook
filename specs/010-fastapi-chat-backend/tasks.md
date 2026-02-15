# Tasks: FastAPI Chat Backend

**Input**: Design documents from `/specs/010-fastapi-chat-backend/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/chat-api.yaml
**Tests**: Explicitly requested — TDD approach with test tasks included per user story.

**Organization**: Tasks grouped by user story. Gemini migration (spec 009 refactor) is a blocking prerequisite in Phase 2.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1–US5)
- All paths relative to repository root `/workspaces/physical-ai-robotics-textbook/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Scaffold the `backend/` package, update `pyproject.toml`, create directory structure

- [x] T001 Create backend package directory structure: `backend/__init__.py`, `backend/routers/__init__.py`, `backend/services/__init__.py`, `backend/models/__init__.py`, `backend/db/__init__.py`, `backend/migrations/` per plan.md project structure
- [x] T002 Create test directory structure: `tests/unit/test_backend/__init__.py`, `tests/integration/test_backend/__init__.py`
- [x] T003 Update `pyproject.toml` to replace `openai>=1.0.0` with `google-genai>=1.0.0` in base dependencies, and add `[api]` optional-dependencies group with `fastapi>=0.110`, `uvicorn[standard]>=0.27`, `asyncpg>=0.29`, `sse-starlette>=1.8`, `httpx>=0.27` (test client). Add `backend` to `[tool.setuptools.packages.find] include`
- [x] T004 [P] Create `.env.example` at repository root with all env vars from quickstart.md: `DATABASE_URL`, `GEMINI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION`, `EMBEDDING_MODEL`, `EMBEDDING_DIMENSIONS`, `LLM_MODEL`, `CORS_ORIGINS`, `RATE_LIMIT_ANON`, `RATE_LIMIT_IDENTIFIED`, `LOG_LEVEL`
- [x] T005 [P] Create `backend/config.py` with `BackendSettings(BaseModel)` per plan.md configuration section: `database_url`, `gemini_api_key`, `cors_origins`, `llm_model` (default `gemini-2.0-flash`), `embedding_model` (default `gemini-embedding-001`), `embedding_dimensions` (default 768), `rate_limit_anon` (10), `rate_limit_identified` (20), `max_message_length` (4000), `max_selected_text_length` (10000), `max_history_messages` (10), `log_level` (INFO). Add `load_backend_settings()` loading from env vars

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Gemini migration of spec 009, DB schema, core models, error handling — MUST complete before any user story

**CRITICAL**: No user story work can begin until this phase is complete

### Phase 2a: Gemini Migration (Spec 009 Refactor)

- [x] T006 Update `rag/config.py`: replace `openai_api_key` field with `gemini_api_key`, change `embedding_model` default to `gemini-embedding-001`, change `embedding_dimensions` default to `768`. Update `load_settings()` to read `GEMINI_API_KEY` env var instead of `OPENAI_API_KEY`
- [x] T007 Rename `rag/embedder/openai_embedder.py` to `rag/embedder/gemini_embedder.py`. Rewrite to use `from google import genai`; replace `OpenAI.embeddings.create()` with `client.models.embed_content(model=..., contents=..., config={"output_dimensionality": 768})`. Keep same function signature `async def batch_embed(chunks, client=None, model=None, batch_size=100, max_retries=3) -> list[Chunk]` but accept `genai.Client` instead of `OpenAI`
- [x] T008 Update `rag/retriever/query_engine.py`: replace `from openai import OpenAI` with `from google import genai`. In `retrieve()` and `_retrieve_normal()`, change `openai_client` param to `gemini_client: genai.Client | None`. Replace `client.embeddings.create()` call with `client.models.embed_content(model=settings.embedding_model, contents=query.question, config={"output_dimensionality": settings.embedding_dimensions})` and extract vector from `result.embeddings[0].values`
- [x] T009 Update `rag/store/collection_config.py`: change `VECTOR_CONFIG = VectorParams(size=1536, ...)` to `size=768`
- [x] T010 Update `tests/unit/test_openai_embedder.py`: rename to `tests/unit/test_gemini_embedder.py`, update all mocks from `OpenAI` to `genai.Client`, verify `embed_content()` is called with correct params, assert returned embeddings are 768-dimensional
- [x] T011 Update existing tests that mock `OpenAI` client: scan `tests/unit/test_models.py`, `tests/integration/test_selected_text_leak.py` and any other files referencing `openai` — update imports and mocks to `google.genai`. Verify `test_selected_text_leak.py` source-string checks still pass (replace `"import openai"` checks with `"import google"` if applicable, but keep `rag.store` isolation checks unchanged)
- [x] T012 Run `pytest tests/ -v` to verify all 110+ existing tests still pass after Gemini migration. Fix any failures

### Phase 2b: Database and Core Infrastructure

- [x] T013 Write SQL migration `backend/migrations/001_create_tables.sql` per data-model.md: CREATE TABLE `users` (id UUID PK default gen_random_uuid(), token VARCHAR(255) UNIQUE NOT NULL, tier VARCHAR(20) NOT NULL default 'anonymous', ip_address INET, created_at TIMESTAMPTZ NOT NULL default now(), last_seen_at TIMESTAMPTZ NOT NULL default now()); CREATE TABLE `sessions` (id UUID PK, user_id UUID FK references users(id) ON DELETE CASCADE, title VARCHAR(255) NOT NULL default '', status VARCHAR(20) NOT NULL default 'active', created_at TIMESTAMPTZ, updated_at TIMESTAMPTZ); CREATE TABLE `messages` (id UUID PK, session_id UUID FK references sessions(id) ON DELETE CASCADE, role VARCHAR(20) NOT NULL, content TEXT NOT NULL, retrieval_mode VARCHAR(30), selected_text TEXT, chunk_count INTEGER, latency_ms INTEGER, created_at TIMESTAMPTZ NOT NULL default now()); CREATE indexes per data-model.md
- [x] T014 Implement `backend/db/pool.py`: async function `create_pool(database_url: str) -> asyncpg.Pool` using `asyncpg.create_pool(dsn=..., min_size=2, max_size=20)` and `close_pool(pool)`. Pool created on app startup, closed on shutdown
- [x] T015 Implement `backend/db/migrate.py`: migration runner that creates `schema_migrations` table if not exists, reads `.sql` files from `backend/migrations/` sorted by prefix, skips applied, executes unapplied in transaction, records version. Expose as `python -m backend.db.migrate`
- [x] T016 [P] Implement `backend/models/api_models.py` with Pydantic request/response models per contracts/chat-api.yaml: `ChatRequest(message: str, session_id: UUID | None, mode: str = "normal", selected_text: str | None, source_doc_path: str | None, source_section: str | None, filters: QueryFilters | None)` with validators (message min 1 / max 4000 chars, selected_text required when mode=selected_text_only, max 10000 chars); `TokenEvent`, `DoneEvent`, `Citation`, `SessionSummary`, `SessionList`, `MessageResponse`, `MessageList`, `HealthResponse`
- [x] T017 [P] Implement `backend/models/errors.py`: `ErrorResponse` model with `error.code` and `error.message` fields; `ChatBackendError` exception base class; subclasses `ValidationError`, `RateLimitError`, `SessionNotFoundError`, `RetrievalError`, `GenerationError`; FastAPI exception handlers that return consistent JSON per plan.md error handling strategy — never expose stack traces
- [x] T018 Implement `backend/db/queries.py`: async SQL query functions using asyncpg pool — `create_user(pool, token, tier, ip) -> UUID`, `get_user_by_token(pool, token) -> Row|None`, `create_session(pool, user_id, title) -> UUID`, `get_session(pool, session_id) -> Row|None`, `list_sessions(pool, user_id, limit, offset) -> list[Row]`, `create_message(pool, session_id, role, content, retrieval_mode, selected_text, chunk_count, latency_ms) -> UUID`, `get_messages(pool, session_id) -> list[Row]`, `update_session_activity(pool, session_id)`
- [x] T019 Implement `backend/dependencies.py`: FastAPI dependency functions — `get_db_pool()` returns app-state pool, `get_gemini_client()` returns `genai.Client(api_key=settings.gemini_api_key)`, `get_current_user(request, pool)` extracts Bearer token from Authorization header → lookup user → return user row or create anonymous user from IP. Load settings via `load_backend_settings()`
- [x] T020 Implement `backend/main.py`: FastAPI app factory with lifespan (create pool on startup, close on shutdown), CORS middleware using `settings.cors_origins`, include routers from `backend.routers.chat`, `backend.routers.sessions`, `backend.routers.health`. Store pool and Gemini client in `app.state`

**Checkpoint**: Foundation ready — DB schema, Gemini migration, core models, app skeleton all in place

---

## Phase 3: User Story 1 — Student Asks a Question via Chat (Priority: P1) MVP

**Goal**: A student sends a question, the backend retrieves textbook content via Qdrant, generates a grounded LLM response, and streams it back as SSE

**Independent Test**: `curl -X POST http://localhost:8000/api/v1/chat -H "Content-Type: application/json" -d '{"message": "What is Physical AI?"}' --no-buffer` returns SSE stream with token events and a done event with citations

### Tests for User Story 1

- [x] T021 [P] [US1] Write unit test `tests/unit/test_backend/test_api_models.py`: test ChatRequest rejects empty message, rejects message > 4000 chars, requires selected_text when mode=selected_text_only, accepts valid normal-mode request, accepts valid selected-text request with all fields
- [x] T022 [P] [US1] Write unit test `tests/unit/test_backend/test_chat_service.py`: mock `rag.retriever.query_engine.retrieve()`, mock `rag.response.grounding.apply_grounding_policy()`, mock `genai.Client.models.generate_content_stream()`. Test that: (1) retrieve is called with correct `rag.models.Query` for normal mode, (2) grounding policy is applied, (3) LLM stream tokens are yielded, (4) full response text is accumulated, (5) conversation history (last N messages) is included in LLM prompt
- [x] T023 [P] [US1] Write integration test `tests/integration/test_backend/test_chat_endpoint.py`: use `httpx.AsyncClient` with FastAPI TestClient. Mock Gemini client and rag retrieval. POST to `/api/v1/chat` with `{"message": "What is Physical AI?"}`, verify response is `text/event-stream`, parse SSE events, assert at least one `token` event and one `done` event with `message_id`, `session_id`, and `citations` fields

### Implementation for User Story 1

- [x] T024 [US1] Implement `backend/services/chat_service.py`: async generator `stream_chat_response(message, session_id, mode, selected_text, source_doc_path, source_section, filters, pool, gemini_client, user) -> AsyncGenerator[str, None]` that: (1) resolves/creates session, (2) persists user message to DB via `queries.create_message()` — wrap in try/except so DB failure logs warning but does NOT abort chat flow (edge case: DB unreachable), (3) loads conversation history (last `max_history_messages` messages; if total exceeds context budget, drop oldest first — MVP uses message count, not token counting), (4) constructs `rag.models.Query` from request params, (5) calls `rag.retriever.query_engine.retrieve(query)`, (6) calls `rag.response.grounding.apply_grounding_policy(result, message)`, (7) builds Gemini prompt with system_instruction + context + history + question, (8) calls `gemini_client.models.generate_content_stream()`, (9) yields SSE-formatted token events, (10) accumulates full response, (11) on completion OR on client disconnect (`asyncio.CancelledError`): persist assistant message with metadata (retrieval_mode, chunk_count, selected_text, latency_ms) — persist partial text if client disconnected mid-stream, (12) yields done event with message_id, session_id, citations (includes `persistence_warning: true` if DB writes failed in steps 2 or 11)
- [x] T025 [US1] Implement `backend/routers/chat.py`: `POST /api/v1/chat` route accepting `ChatRequest` body, using `Depends(get_db_pool)`, `Depends(get_gemini_client)`, `Depends(get_current_user)`. Call `chat_service.stream_chat_response()` and return `EventSourceResponse` from `sse-starlette`. Handle errors with try/except yielding SSE error events
- [x] T026 [US1] Run tests: `pytest tests/unit/test_backend/test_api_models.py tests/unit/test_backend/test_chat_service.py tests/integration/test_backend/test_chat_endpoint.py -v` — all must pass

**Checkpoint**: Core chat with retrieval and streaming works. `POST /api/v1/chat` accepts a question, retrieves context, streams a grounded answer.

---

## Phase 4: User Story 2 — Selected-Text-Only Chat (Priority: P1)

**Goal**: Student highlights text, asks a question, and the backend responds using ONLY the selected text — zero Qdrant access, zero external knowledge

**Independent Test**: POST with `mode: "selected_text_only"` and `selected_text` returns answer grounded exclusively in the selection. Mock Qdrant is never called.

### Tests for User Story 2

- [x] T027 [P] [US2] Write integration test `tests/integration/test_backend/test_selected_text_isolation.py`: mock Qdrant client and Gemini embedding call. POST `/api/v1/chat` with `{"message": "Explain this", "mode": "selected_text_only", "selected_text": "Physical AI systems operate in real-time."}`. Assert: (1) Qdrant search mock NOT called, (2) Gemini embed_content mock NOT called, (3) SSE response references only the selected text, (4) done event citations reference "Selected Text" source. This mirrors existing `tests/integration/test_selected_text_leak.py` pattern
- [x] T028 [P] [US2] Write unit test addition in `tests/unit/test_backend/test_chat_service.py`: add test case for selected-text-only mode — assert `retrieve()` receives Query with `mode="selected_text_only"` and `selected_text` populated, assert no Qdrant or embedding client is accessed
- [x] T029 [P] [US2] Write unit test in `tests/unit/test_backend/test_api_models.py`: add tests for selected-text-only validation — rejects request with `mode=selected_text_only` but no selected_text, rejects selected_text > 10000 chars, accepts short selected_text (< 10 words) with warning field

### Implementation for User Story 2

- [x] T030 [US2] Verify `backend/services/chat_service.py` correctly routes selected-text-only mode: when `mode="selected_text_only"`, the `rag.models.Query` is constructed with `selected_text`, `source_doc_path`, `source_section` — `retrieve()` routes to `retrieve_from_selection()` which bypasses Qdrant entirely. No additional implementation needed beyond what T024 built — this task verifies the path works end-to-end
- [x] T031 [US2] Run selected-text isolation tests: `pytest tests/integration/test_backend/test_selected_text_isolation.py tests/integration/test_selected_text_leak.py -v` — all must pass, confirming zero Qdrant/Gemini leakage

**Checkpoint**: Selected-text-only mode proven isolated. Zero external retrieval confirmed by tests.

---

## Phase 5: User Story 3 — Session and Message Persistence (Priority: P2)

**Goal**: Sessions and messages are persisted to Neon Postgres. Users can list sessions and retrieve message history.

**Independent Test**: Create session via chat → GET `/api/v1/sessions` returns session → GET `/api/v1/sessions/{id}/messages` returns all messages in order

### Tests for User Story 3

- [x] T032 [P] [US3] Write unit test `tests/unit/test_backend/test_session_service.py`: mock asyncpg pool. Test `create_session()`, `list_sessions()` (returns sorted by updated_at desc), `get_messages()` (returns in chronological order), `update_session_activity()` (updates updated_at). Test session title auto-generation from first user message (truncated to 255 chars)
- [x] T033 [P] [US3] Write integration test `tests/integration/test_backend/test_session_endpoints.py`: use httpx TestClient. (1) POST `/api/v1/chat` with message → parse done event for session_id, (2) POST `/api/v1/chat` with same session_id → second message, (3) GET `/api/v1/sessions` → verify session appears with message_count=4 (2 user + 2 assistant), (4) GET `/api/v1/sessions/{session_id}/messages` → verify 4 messages in order with correct roles and timestamps, (5) POST with invalid session_id → verify 404 error

### Implementation for User Story 3

- [x] T034 [US3] Implement `backend/services/session_service.py`: async functions `create_session(pool, user_id, first_message) -> UUID`, `list_user_sessions(pool, user_id, limit, offset) -> list[SessionSummary]` (query sessions with message count via subquery), `get_session_messages(pool, session_id, user_id) -> list[MessageResponse]` (verify session belongs to user), `auto_title(message: str) -> str` (first 100 chars of first message, truncated at word boundary)
- [x] T035 [US3] Implement `backend/routers/sessions.py`: `GET /api/v1/sessions` (list sessions for current user, paginated with limit/offset), `GET /api/v1/sessions/{session_id}/messages` (return message history). Both require `Depends(get_current_user)` and `Depends(get_db_pool)`. Return 404 if session not found or doesn't belong to user
- [x] T036 [US3] Run persistence tests: `pytest tests/unit/test_backend/test_session_service.py tests/integration/test_backend/test_session_endpoints.py -v` — all must pass

**Checkpoint**: Sessions and messages persist. History endpoints return correct ordered data.

---

## Phase 6: User Story 4 — Rate Limiting and Abuse Prevention (Priority: P2)

**Goal**: In-memory sliding window rate limiter blocks excess requests per user/IP with Retry-After header

**Independent Test**: Send 11 rapid requests as anonymous user → 11th returns 429 with Retry-After header

### Tests for User Story 4

- [x] T037 [P] [US4] Write unit test `tests/unit/test_backend/test_rate_limiter.py`: test `SlidingWindowRateLimiter`: (1) allows requests under limit, (2) blocks request at limit+1, (3) returns correct retry_after seconds, (4) resets after window expires, (5) tracks different identifiers independently, (6) anonymous limit (10/min) differs from identified limit (20/min)

### Implementation for User Story 4

- [x] T038 [US4] Implement `backend/services/rate_limiter.py`: `class SlidingWindowRateLimiter` with `__init__(self, window_seconds=60)`, `async def check(self, identifier: str, limit: int) -> tuple[bool, int]` (returns allowed + retry_after_seconds). Uses `dict[str, list[float]]` for timestamp tracking and `asyncio.Lock` for thread safety. Prune expired entries on each check
- [x] T039 [US4] Integrate rate limiter into `backend/routers/chat.py`: add rate limit check before processing — extract identifier (user_id for identified, IP for anonymous), determine limit from user tier, call `rate_limiter.check()`, return 429 `ErrorResponse` with `Retry-After` header if blocked. Instantiate limiter as app-state singleton in `backend/main.py`
- [x] T040 [US4] Run rate limit tests: `pytest tests/unit/test_backend/test_rate_limiter.py -v` — all must pass

**Checkpoint**: Rate limiting enforced. Excess requests get 429 with Retry-After.

---

## Phase 7: User Story 5 — Health Check and Operational Monitoring (Priority: P3)

**Goal**: GET `/api/v1/health` reports status of database and vector store dependencies

**Independent Test**: `curl http://localhost:8000/api/v1/health` returns `{"status": "healthy", "components": {"database": "ok", "vector_store": "ok"}}`

### Tests for User Story 5

- [x] T041 [P] [US5] Write integration test `tests/integration/test_backend/test_health_endpoint.py`: (1) with all deps mocked healthy → returns 200 with status "healthy", (2) with DB pool mock raising error → returns 503 with status "degraded" and database "error", (3) with Qdrant mock unreachable → returns 503 with status "degraded" and vector_store "error"

### Implementation for User Story 5

- [x] T042 [US5] Implement `backend/routers/health.py`: `GET /api/v1/health` that checks DB pool (`pool.execute("SELECT 1")`) and Qdrant (`qdrant.get_collections()`). Return `HealthResponse` with component statuses. Return 200 if all healthy, 503 if any degraded. Include timestamp
- [x] T043 [US5] Run health tests: `pytest tests/integration/test_backend/test_health_endpoint.py -v` — all must pass

**Checkpoint**: Health endpoint reports dependency status accurately.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Error handling integration, logging, CORS hardening, final validation

- [x] T044 [P] Write unit test `tests/unit/test_backend/test_error_handling.py`: test all exception handlers — `ValidationError` → 400, `RateLimitError` → 429 with Retry-After, `SessionNotFoundError` → 404, `RetrievalError` → 503, `GenerationError` → 503. Verify no stack traces in any error response body
- [x] T045 [P] Add request logging middleware in `backend/main.py`: log each chat request with timestamp, session_id, retrieval_mode, response_latency_ms, status_code. Exclude message content from logs (privacy). Use Python `logging` module with `settings.log_level`
- [x] T046 Register all exception handlers from `backend/models/errors.py` in `backend/main.py` app factory
- [x] T047 Run full test suite: `pytest tests/ -v --cov=backend --cov=rag --cov-report=term-missing` — all tests pass, backend coverage >= 80%
- [x] T048 Run quickstart.md validation sequence: (1) `pip install -e ".[api]"` succeeds, (2) `python -m backend.db.migrate` runs without error (against test DB or skip if no DB), (3) `uvicorn backend.main:app --port 8000` starts without error, (4) `curl http://localhost:8000/api/v1/health` returns JSON, (5) verify CORS headers present for configured origin

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — start immediately
- **Foundational (Phase 2)**: Depends on Setup — BLOCKS all user stories
  - Phase 2a (Gemini migration) and Phase 2b (DB/core) can run in parallel
- **User Stories (Phase 3–7)**: All depend on Phase 2 completion
  - US1 and US2 share the chat endpoint — US2 depends on US1 (T024–T025)
  - US3 (sessions) can start after Phase 2 independently
  - US4 (rate limiting) can start after Phase 2 independently
  - US5 (health) can start after Phase 2 independently
- **Polish (Phase 8)**: Depends on all user stories complete

### User Story Dependencies

- **US1 (P1 — Chat)**: Depends on Phase 2 only. MVP target.
- **US2 (P1 — Selected-Text)**: Depends on US1 (reuses chat endpoint and service)
- **US3 (P2 — Sessions)**: Depends on Phase 2 only. Can parallelize with US1.
- **US4 (P2 — Rate Limiting)**: Depends on Phase 2 only. Can parallelize with US1.
- **US5 (P3 — Health)**: Depends on Phase 2 only. Can parallelize with US1.

### Within Each User Story

- Tests MUST be written and FAIL before implementation
- Models before services
- Services before routes
- Run story-specific tests to verify after each story

### Parallel Opportunities

- T004 and T005 can run in parallel during Setup
- T006–T009 (Gemini migration) and T013–T020 (DB/core) are independent groups within Phase 2
- T016 and T017 (Pydantic models, error models) can run in parallel
- T021, T022, T023 (US1 tests) can run in parallel
- T027, T028, T029 (US2 tests) can run in parallel
- US3, US4, US5 can all run in parallel after US1 completes
- T044 and T045 (Polish) can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all US1 tests together (write first, must fail):
Task T021: "Unit test api_models in tests/unit/test_backend/test_api_models.py"
Task T022: "Unit test chat_service in tests/unit/test_backend/test_chat_service.py"
Task T023: "Integration test chat endpoint in tests/integration/test_backend/test_chat_endpoint.py"

# Then implement sequentially:
Task T024: "Implement chat_service in backend/services/chat_service.py"
Task T025: "Implement chat router in backend/routers/chat.py"
Task T026: "Run US1 tests — all must pass"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001–T005)
2. Complete Phase 2: Foundational (T006–T020) — Gemini migration + DB + core
3. Complete Phase 3: User Story 1 (T021–T026)
4. **STOP and VALIDATE**: `pytest tests/ -v` — all tests pass, chat endpoint streams responses
5. Deploy/demo if ready

### Incremental Delivery

1. Setup + Foundational → Foundation ready
2. Add US1 (Chat) → Streaming chat works → **MVP!**
3. Add US2 (Selected-Text) → Selection isolation proven
4. Add US3 (Sessions) → Persistence + history endpoints
5. Add US4 (Rate Limiting) → Abuse prevention active
6. Add US5 (Health) → Operational monitoring
7. Polish → Error handling, logging, CORS hardened

### Key Validation Commands

```bash
# Run all tests
pytest tests/ -v --cov=backend --cov=rag --cov-report=term-missing

# Run only backend tests
pytest tests/unit/test_backend/ tests/integration/test_backend/ -v

# Run selected-text isolation tests
pytest tests/integration/test_backend/test_selected_text_isolation.py tests/integration/test_selected_text_leak.py -v

# Start server locally
uvicorn backend.main:app --reload --port 8000

# Test health endpoint
curl http://localhost:8000/api/v1/health

# Test chat (normal mode)
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}' --no-buffer

# Test chat (selected-text-only mode)
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Explain this", "mode": "selected_text_only", "selected_text": "Physical AI systems operate in real-time."}' --no-buffer

# Run database migration
python -m backend.db.migrate
```

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks in same phase
- [Story] label maps task to specific user story for traceability
- Total: 48 tasks (5 setup, 15 foundational, 6 US1, 5 US2, 5 US3, 4 US4, 3 US5, 5 polish)
- Gemini migration (T006–T012) is a breaking change — existing tests must pass before proceeding
- Selected-text isolation is validated at both RAG level (existing leak tests) and backend level (new T027)
