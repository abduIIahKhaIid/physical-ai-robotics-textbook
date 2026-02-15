# Implementation Plan: FastAPI Chat Backend

**Branch**: `010-fastapi-chat-backend` | **Date**: 2026-02-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/010-fastapi-chat-backend/spec.md`

## Summary

Build a FastAPI backend that serves the RoboTutor chatbot for the Physical AI textbook. The backend provides an SSE-streaming chat endpoint that integrates with the existing RAG retrieval pipeline (spec 009) for full-book and selected-text-only query modes, persists conversations to Neon Postgres, and enforces rate limiting and CORS policies. The architecture adds a `backend/` package alongside the existing `rag/` package, reusing spec 009's retrieval and grounding modules directly.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI, uvicorn, asyncpg, sse-starlette, google-genai (replaces openai), pydantic (existing)
**Storage**: Neon Postgres (asyncpg), Qdrant Cloud (via existing `rag.store`)
**Testing**: pytest, pytest-asyncio, httpx (for TestClient), pytest-cov
**Target Platform**: Linux container (single instance for MVP)
**Project Type**: Web backend (API server)
**Performance Goals**: First token <2s, full response <15s p95, 50 concurrent users
**Constraints**: Single-container deployment, in-memory rate limiting, no external auth provider
**Scale/Scope**: 50 concurrent users, 3 DB tables, 4 API endpoints

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Documentation-First | PASS | Spec written before implementation; plan defines all artifacts |
| II. Selected Text Only Mode | PASS | Backend routes to `selected_text_only` retrieval path that bypasses Qdrant entirely (spec 009 `retrieve_from_selection`). Grounding policy enforces strict instruction. Integration test validates zero Qdrant access. |
| III. Test-First (NON-NEGOTIABLE) | PASS | Plan defines test files before implementation files. TDD cycle: write test → implement → refactor. |
| IV. Secure Architecture | PASS | No secrets in code — all via env vars. CORS locked to configured origins. Error responses sanitized. Input validation on all endpoints. |
| V. Scalable Cloud Infrastructure | PASS | asyncpg connection pooling. Stateless API design (rate limit state is the only in-memory state, acceptable per spec for single-instance MVP). |
| VI. Modular Component Design | PASS | Clean separation: routers → services → db layer. Backend imports RAG pipeline as a library, does not duplicate its logic. |
| Performance Standards | PASS | Chat response <2s first token; health check instant. asyncpg pool for concurrent DB access. |
| Quality Gates | PASS | 80% coverage target. All tests pass before merge. Security scan via input validation. |

## Project Structure

### Documentation (this feature)

```text
specs/010-fastapi-chat-backend/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0: technology decisions
├── data-model.md        # Phase 1: database schema
├── quickstart.md        # Phase 1: local dev setup
├── contracts/
│   └── chat-api.yaml    # Phase 1: OpenAPI contract
└── tasks.md             # Phase 2 output (from /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── __init__.py
├── main.py              # FastAPI app factory, lifespan, CORS
├── config.py            # Settings from env vars (extends rag.config)
├── dependencies.py      # FastAPI dependency injection (db pool, clients)
├── routers/
│   ├── __init__.py
│   ├── chat.py          # POST /api/v1/chat (SSE streaming)
│   ├── sessions.py      # GET /api/v1/sessions, GET .../messages
│   └── health.py        # GET /api/v1/health
├── services/
│   ├── __init__.py
│   ├── chat_service.py  # Orchestrates: retrieve → ground → generate → stream
│   ├── session_service.py  # Session/message CRUD
│   └── rate_limiter.py  # In-memory sliding window rate limiter
├── models/
│   ├── __init__.py
│   ├── api_models.py    # Pydantic request/response models
│   └── errors.py        # Error response models and exception handlers
├── db/
│   ├── __init__.py
│   ├── pool.py          # asyncpg pool creation and lifecycle
│   ├── queries.py       # SQL query functions (sessions, messages, users)
│   └── migrate.py       # Migration runner
└── migrations/
    └── 001_create_tables.sql  # Initial schema

tests/
├── unit/
│   ├── test_backend/
│   │   ├── __init__.py
│   │   ├── test_api_models.py       # Request/response validation
│   │   ├── test_chat_service.py     # Chat orchestration with mocked deps
│   │   ├── test_session_service.py  # Session CRUD with mocked DB
│   │   ├── test_rate_limiter.py     # Rate limit logic
│   │   └── test_error_handling.py   # Error response formatting
│   └── ... (existing rag tests)
├── integration/
│   ├── test_backend/
│   │   ├── __init__.py
│   │   ├── test_chat_endpoint.py    # Full chat flow with mocked LLM
│   │   ├── test_selected_text_isolation.py  # Verify zero Qdrant access
│   │   ├── test_session_endpoints.py  # Session CRUD via HTTP
│   │   └── test_health_endpoint.py  # Health check responses
│   └── ... (existing rag tests)
└── ... (existing structure)
```

**Structure Decision**: The `backend/` package sits alongside `rag/` at the repository root. It imports from `rag` as a library. Both packages are installable via `pyproject.toml` with an `[api]` optional dependency group. Tests follow the existing pattern under `tests/unit/` and `tests/integration/` with a `test_backend/` sub-directory.

## Architecture Design

### Runtime Flow

```
Client (ChatKit widget)
    │
    │ POST /api/v1/chat  {message, session_id?, mode, selected_text?}
    │ Authorization: Bearer <token>  (optional)
    ▼
┌─────────────────────────────────────────────────────────┐
│ FastAPI Router (chat.py)                                 │
│  1. Validate request (Pydantic ChatRequest model)        │
│  2. Identify user (token or IP) → rate limit check       │
│  3. Resolve/create session                               │
└──────────────┬──────────────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────────────┐
│ Chat Service (chat_service.py)                           │
│  4. Persist user message to DB                           │
│  5. Load conversation history (last N messages)          │
│  6. Construct rag.models.Query from request              │
│  7. Call rag.retriever.query_engine.retrieve(query)      │
│     ├── mode="normal" → Qdrant vector search             │
│     └── mode="selected_text_only" → bypass (no Qdrant)   │
│  8. Call rag.response.grounding.apply_grounding_policy() │
│  9. Build LLM messages: system_instruction + context     │
│     + conversation history + user question               │
│ 10. Stream LLM response (Gemini generate_content_stream) │
│ 11. Accumulate full response text                        │
│ 12. Persist assistant message to DB                      │
│ 13. Yield SSE events: token → token → ... → done         │
└──────────────┬──────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────┐   ┌──────────────────────┐
│ Neon Postgres (asyncpg)      │   │ Qdrant Cloud          │
│  - users, sessions, messages │   │  - textbook_chunks    │
│  - via connection pool       │   │  - (normal mode only) │
└──────────────────────────────┘   └──────────────────────┘
```

### SSE Event Protocol

The chat endpoint returns `text/event-stream` with three event types:

```
event: token
data: {"content": "partial text"}

event: token
data: {"content": " more text"}

...

event: done
data: {"message_id": "uuid", "session_id": "uuid", "citations": [...]}
```

On error:
```
event: error
data: {"code": "retrieval_failed", "message": "Search is temporarily unavailable"}
```

### Dependency Injection

```python
# dependencies.py
async def get_db_pool() -> asyncpg.Pool:
    """Yield the app-level asyncpg connection pool."""
    ...

async def get_gemini_client() -> genai.Client:
    """Yield the google-genai Client for text generation and embeddings."""
    ...

async def get_current_user(request: Request, pool=Depends(get_db_pool)) -> User:
    """Extract user from Authorization header or create anonymous user."""
    ...
```

### Rate Limiting Design

```python
# rate_limiter.py
class SlidingWindowRateLimiter:
    """In-memory sliding window rate limiter.

    Tracks request timestamps per identifier (user_id or IP).
    Thread-safe via asyncio.Lock.
    """
    def __init__(self, window_seconds=60):
        self._windows: dict[str, list[float]] = {}
        self._lock = asyncio.Lock()

    async def check(self, identifier: str, limit: int) -> tuple[bool, int]:
        """Returns (allowed, retry_after_seconds)."""
        ...
```

Applied as a FastAPI dependency on the chat router. Limits: anonymous=10/min, identified=20/min (configurable via env vars).

### Context Window Management

For multi-turn conversations, the system loads the most recent N messages from the session (configurable, default 10). If the total token count exceeds the LLM's context budget (minus the system instruction + retrieved context), messages are truncated from the oldest first. The system always includes at minimum the current user message and the system instruction.

### Error Handling Strategy

All errors are caught by FastAPI exception handlers and returned in a consistent format:

```json
{
  "error": {
    "code": "rate_limit_exceeded",
    "message": "You've sent too many messages. Please try again in 45 seconds."
  }
}
```

Error codes: `validation_error`, `rate_limit_exceeded`, `session_not_found`, `retrieval_failed`, `generation_failed`, `service_unavailable`, `internal_error`.

### Database Migration

A single SQL migration file creates all three tables. Migrations are tracked by a `schema_migrations` table. The migration runner:

1. Creates `schema_migrations` if it doesn't exist
2. Reads `.sql` files from `backend/migrations/`, sorted by prefix
3. Skips already-applied versions
4. Executes unapplied migrations in a transaction
5. Records the version in `schema_migrations`

### Configuration Extension

The backend config extends the existing `rag.config.Settings`:

```python
# backend/config.py
class BackendSettings(BaseModel):
    database_url: str
    gemini_api_key: str            # Google Gemini API key
    cors_origins: list[str] = ["https://abdullahkhalid.com"]
    llm_model: str = "gemini-2.0-flash"
    embedding_model: str = "gemini-embedding-001"
    embedding_dimensions: int = 768
    rate_limit_anon: int = 10
    rate_limit_identified: int = 20
    max_message_length: int = 4000
    max_selected_text_length: int = 10000
    max_history_messages: int = 10
    log_level: str = "INFO"
```

Loaded from environment variables, following the same pattern as `rag.config.load_settings()`.

### LLM Provider: Gemini

The backend uses Google Gemini (`google-genai` SDK) for both text generation and embeddings, replacing OpenAI:

- **Text generation**: `client.models.generate_content_stream()` with `gemini-2.0-flash`
- **Embeddings**: `client.models.embed_content()` with `gemini-embedding-001` at 768 dimensions
- **Auth**: `GEMINI_API_KEY` environment variable

**Streaming pattern**:
```python
from google import genai

client = genai.Client(api_key=settings.gemini_api_key)

response = client.models.generate_content_stream(
    model=settings.llm_model,
    contents=[system_instruction + "\n\n" + context + "\n\nUser: " + question],
)
for chunk in response:
    if chunk.text:
        yield chunk.text
```

## Integration with Spec 009

### Direct Imports (no duplication)

The backend imports these spec 009 modules directly:

| Module | Function | Used For |
|--------|----------|----------|
| `rag.models.Query` | Data model | Construct retrieval query from HTTP request |
| `rag.models.QueryFilters` | Data model | Pass through user's filter preferences |
| `rag.models.GroundedResponse` | Data model | Receive grounded context + instruction |
| `rag.retriever.query_engine.retrieve()` | Function | Execute retrieval (routes normal vs selected-text) |
| `rag.response.grounding.apply_grounding_policy()` | Function | Apply grounding rules to retrieval result |
| `rag.config.load_settings()` | Function | Load Qdrant/embedding config |

### Spec 009 Refactoring: OpenAI → Gemini Embeddings

**BREAKING CHANGE**: The existing RAG pipeline must be refactored to use Gemini embeddings instead of OpenAI. This is a prerequisite for the backend since both ingestion and query-time embeddings must use the same model.

**Files requiring changes in `rag/`**:

| File | Change |
|------|--------|
| `rag/config.py` | Replace `openai_api_key` with `gemini_api_key`; change `embedding_model` default to `gemini-embedding-001`; change `embedding_dimensions` default to `768` |
| `rag/embedder/openai_embedder.py` | Rename to `gemini_embedder.py`; replace `openai.OpenAI` with `google.genai.Client`; use `client.models.embed_content()` |
| `rag/retriever/query_engine.py` | Replace `OpenAI` import and `embeddings.create()` call with Gemini `embed_content()` |
| `rag/store/collection_config.py` | Change `VECTOR_CONFIG` size from `1536` to `768` |
| `pyproject.toml` | Replace `openai>=1.0.0` dependency with `google-genai>=1.0.0` |

**Qdrant collection impact**: The vector dimension change (1536 → 768) requires:
1. Delete the existing `textbook_chunks` collection
2. Recreate with `size=768`
3. Re-run the full ingestion pipeline to re-embed all content with Gemini

**Existing test impact**: Tests that mock `OpenAI` client calls must be updated to mock `genai.Client` calls instead. The selected-text leak tests (`test_selected_text_leak.py`) remain valid since they test isolation from Qdrant, not from any specific embedding provider.

### Selected-Text Isolation Preservation

**CRITICAL**: The backend MUST NOT break the selected-text isolation pattern:

1. `chat_service.py` checks `mode` before constructing the `Query`
2. When `mode="selected_text_only"`, the `Query` object is constructed with the selected text, and `retrieve()` routes to `retrieve_from_selection()` which never imports Qdrant or any embedding client
3. The backend MUST NOT pre-initialize Qdrant or Gemini clients for selected-text-only requests
4. Integration test `test_selected_text_isolation.py` verifies zero Qdrant calls and zero Gemini calls via mocking (mirrors existing `test_selected_text_leak.py`)

## Validation Plan

### Unit Tests (TDD — write first)

| Test File | Validates | Key Assertions |
|-----------|-----------|----------------|
| `test_api_models.py` | Pydantic request/response validation | Rejects empty messages, requires selected_text when mode=selected_text_only, validates max lengths |
| `test_chat_service.py` | Chat orchestration | Calls retrieve() with correct Query, applies grounding, streams LLM tokens, persists messages |
| `test_session_service.py` | Session CRUD | Creates sessions, loads history in order, lists sessions sorted by activity |
| `test_rate_limiter.py` | Sliding window logic | Allows requests under limit, blocks over limit, resets after window, returns correct Retry-After |
| `test_error_handling.py` | Error formatting | All error codes return consistent JSON, no stack traces exposed |

### Integration Tests

| Test File | Validates | Key Assertions |
|-----------|-----------|----------------|
| `test_chat_endpoint.py` | Full HTTP chat flow | POST returns SSE stream, tokens arrive progressively, done event has citations |
| `test_selected_text_isolation.py` | Zero Qdrant/Gemini access in selected-text mode | Qdrant mock not called, no Gemini embedding call, response uses only selected text |
| `test_session_endpoints.py` | Session persistence via HTTP | Create session → send messages → GET history returns all in order |
| `test_health_endpoint.py` | Health check accuracy | Reports healthy when deps up, degraded when DB down |

### Gemini Migration Tests (Spec 009 refactor)

| Test File | Validates | Key Assertions |
|-----------|-----------|----------------|
| `test_gemini_embedder.py` | Gemini embedding integration | Returns 768-dim vectors, batch works, retry on error |
| `test_query_engine.py` (updated) | Query-time embedding uses Gemini | `genai.Client.models.embed_content()` called, not `openai` |
| `test_selected_text_leak.py` (updated) | Isolation still holds with Gemini | No `google.genai` or `rag.store` imports in selected_text module |

### Database Migration Tests

| Validation | Method |
|------------|--------|
| Schema correctness | Run migration against a test database, verify tables/indexes exist |
| Idempotency | Run migration twice, verify no errors |
| Rollback safety | Each migration can be reversed (DROP TABLE) |

### Local Run Validation

```bash
# 1. Run all tests (including new backend tests)
pytest tests/ -v --cov=backend --cov=rag --cov-report=term-missing

# 2. Verify migration
python -m backend.db.migrate

# 3. Start server and test health
uvicorn backend.main:app --port 8000 &
curl http://localhost:8000/api/v1/health

# 4. Test chat endpoint
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}' \
  --no-buffer

# 5. Test selected-text-only mode
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Explain this", "mode": "selected_text_only", "selected_text": "Physical AI systems operate in real-time."}' \
  --no-buffer
```

## Complexity Tracking

| Deviation | Why Needed | Constitution Reference |
|-----------|------------|----------------------|
| Gemini instead of OpenAI Agents SDK | Free-tier access, single SDK for both embeddings and text generation, reduces cost to zero for development | Constitution specifies "OpenAI Agents SDK for NLP" — Gemini provides equivalent capability with open access |

## Risks

1. **LLM latency variance**: OpenAI response times can spike during high-traffic periods, potentially exceeding the 2s first-token target. *Mitigation*: Configure request timeouts, return graceful error on timeout, log latency for monitoring.
2. **asyncpg pool exhaustion**: Under 50 concurrent users, the default pool size (2-10) could saturate if DB queries are slow. *Mitigation*: Set pool `max_size=20`, add connection timeout, monitor pool stats via health endpoint.
3. **In-memory rate limit state loss on restart**: Rate limit counters reset on container restart, allowing a brief window of unthrottled access. *Mitigation*: Acceptable for MVP single-instance deployment; document as known limitation for scaling phase.
4. **Qdrant re-ingestion required**: Switching embeddings from OpenAI (1536d) to Gemini (768d) is a breaking change — the entire textbook must be re-ingested. *Mitigation*: This is a one-time operation; the ingestion pipeline (spec 009) already supports full re-runs. Run ingestion as part of deployment.
