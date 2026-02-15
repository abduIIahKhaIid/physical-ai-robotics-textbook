# Research: FastAPI Chat Backend

**Date**: 2026-02-14 | **Branch**: `010-fastapi-chat-backend`

## R1: SSE Streaming Strategy

**Decision**: Use `sse-starlette` (`EventSourceResponse`) for SSE streaming.

**Rationale**: `sse-starlette` provides full SSE protocol compliance (auto-formatted events, ping/keep-alive, client disconnect detection, event/id/retry fields) out of the box. Raw `StreamingResponse` with `text/event-stream` requires manual formatting and lacks disconnect detection. For a chat application forwarding LLM token streams, `EventSourceResponse` is the production-proven choice.

**Alternatives considered**:
- `StreamingResponse` with manual SSE formatting — lighter dependency but loses keep-alive pings and disconnect detection.
- WebSockets — out of scope per spec; more complex, requires client-side reconnection logic.

**SSE Event Format**:
```
event: token
data: {"content": "partial text", "citations": []}

event: done
data: {"message_id": "...", "session_id": "..."}

event: error
data: {"code": "...", "message": "..."}
```

## R2: Database Client for Neon Postgres

**Decision**: Use `asyncpg` directly with a connection pool (no ORM).

**Rationale**: The data model is simple (3 tables, basic CRUD). `asyncpg` is the lightest-weight async Postgres driver for Python, provides built-in connection pooling via `asyncpg.create_pool()`, and works natively with Neon Postgres (which supports standard `postgresql://` connection strings with SSL). No need for SQLAlchemy's overhead for this scope.

**Alternatives considered**:
- `sqlalchemy[asyncio]` + `asyncpg` — adds ORM overhead; warranted only if the schema grows complex. Can migrate to this later if needed.
- `databases` library — wrapper around asyncpg with a query-builder API; adds an abstraction layer without clear benefit for 3-table CRUD.
- `psycopg[binary]` (v3 async) — mature alternative, but asyncpg is more established in FastAPI ecosystem.

**Connection pattern**:
```python
pool = await asyncpg.create_pool(dsn=DATABASE_URL, min_size=2, max_size=10)
```
Pool created on app startup, closed on shutdown. Injected into route handlers via FastAPI dependency.

## R3: LLM and Embedding Provider — Gemini

**Decision**: Use Google Gemini (`google-genai` SDK) for both text generation and embeddings, replacing OpenAI throughout the stack.

**Rationale**: Gemini is a free/open-access LLM with generous free-tier API limits. It provides both text generation (streaming) and embedding models via a single SDK (`google-genai`), reducing dependency count. The `gemini-embedding-001` model supports flexible dimensions (128–3072) and the text generation models (`gemini-2.0-flash`) support streaming via `generate_content_stream()`.

**Text Generation Pattern**:
```python
from google import genai

client = genai.Client(api_key=GEMINI_API_KEY)

response = client.models.generate_content_stream(
    model="gemini-2.0-flash",
    contents=[system_instruction + context + user_question],
)
for chunk in response:
    yield chunk.text
```

**Embedding Pattern**:
```python
from google import genai

client = genai.Client(api_key=GEMINI_API_KEY)

result = client.models.embed_content(
    model="gemini-embedding-001",
    contents="What is Physical AI?",
    config={"output_dimensionality": 768},
)
embedding = result.embeddings[0].values  # list[float], 768 dimensions
```

**Key Model Details**:
- **Text generation**: `gemini-2.0-flash` (fast, streaming-capable)
- **Embeddings**: `gemini-embedding-001` (768 dimensions recommended, supports 128–3072)
- **SDK**: `google-genai` (unified Python SDK, `from google import genai`)
- **Auth**: API key via `GEMINI_API_KEY` env var

**Impact on Spec 009 RAG pipeline**:
The existing RAG pipeline (spec 009) uses `openai` SDK for embeddings in two places:
1. `rag/embedder/openai_embedder.py` — batch embedding during ingestion
2. `rag/retriever/query_engine.py:_retrieve_normal()` — query-time embedding
3. `rag/config.py` — `openai_api_key`, `embedding_model` defaults
4. `rag/store/collection_config.py` — vector size hardcoded to 1536

These must be refactored as part of this feature to use Gemini embeddings. The Qdrant collection must be recreated with 768-dimension vectors. This is a **breaking change** requiring re-ingestion of all textbook content.

**Alternatives considered**:
- OpenAI (`gpt-4o-mini` + `text-embedding-3-small`) — proprietary, paid, but well-tested in current codebase. Rejected in favor of open/free-tier access.
- Ollama (local models) — no API cost but requires GPU infrastructure and self-hosting. Not suitable for cloud deployment.
- Cohere (embed v3 + Command R) — strong embedding model but less ecosystem support than Gemini.

## R4: Rate Limiting Approach

**Decision**: In-memory sliding window rate limiter using `dict` + `asyncio.Lock`.

**Rationale**: The spec assumes single-container deployment for MVP. An in-memory approach is simplest, has zero external dependencies, and provides sub-millisecond rate limit checks. The spec explicitly states "in-memory sliding window for MVP, with an option to move to a distributed store later."

**Alternatives considered**:
- `slowapi` (wraps `limits` library) — popular FastAPI rate limiter, but adds a dependency and is designed for simpler per-route limits rather than tiered user/IP logic.
- Redis-backed rate limiting — needed for horizontal scaling but out of scope for single-instance MVP.

## R5: Migration Strategy

**Decision**: Raw SQL migration files, executed by a simple Python runner.

**Rationale**: The schema is 3 tables with straightforward relationships. Alembic adds significant tooling overhead (alembic.ini, env.py, revision management) that isn't justified for this scope. Plain `.sql` files numbered sequentially (`001_create_tables.sql`) are easy to review, version-control, and run. A minimal Python script applies unapplied migrations tracked by a `schema_migrations` table.

**Alternatives considered**:
- Alembic — standard choice for evolving schemas, but the overhead isn't warranted for the initial 3-table schema. Can be introduced if schema evolution becomes complex.
- Prisma — requires Node.js runtime; not appropriate for a Python project.
- Neon branching — useful for staging but doesn't replace migration files.

## R6: Integration with Spec 009 RAG Pipeline

**Decision**: Import and call `rag.retriever.query_engine.retrieve()` and `rag.response.grounding.apply_grounding_policy()` directly.

**Rationale**: The RAG pipeline is in the same repository as a Python package. Direct function calls avoid network overhead and complexity. The existing `Query` model already supports `mode="selected_text_only"` with proper validation. The `GroundedResponse` provides `system_instruction` and `context` ready for LLM prompt construction.

**Critical constraint**: The selected-text-only path in `query_engine.retrieve()` uses lazy imports — it checks mode *before* any Qdrant imports. The backend must preserve this pattern: never import or initialize Qdrant clients when handling selected-text-only requests.

**Integration flow**:
1. Backend constructs `rag.models.Query` from the HTTP request
2. Calls `retrieve(query)` — routes to selected_text or normal mode
3. Calls `apply_grounding_policy(result, question)` — returns `GroundedResponse`
4. Passes `grounded.system_instruction` + `grounded.context` + conversation history to LLM
5. Streams LLM response tokens back as SSE events

## R7: Authentication Placeholder

**Decision**: Opaque bearer tokens in `Authorization` header, generated by the backend on first session creation.

**Rationale**: Spec explicitly states "session tokens are opaque bearer tokens generated by the backend — no external identity provider integration is required for the initial version." This provides user identification for rate limiting and session association without the complexity of OAuth/SSO.

**Pattern**: Generate a UUID4-based token, store it in the `users` table, return it in the first session creation response. Client includes it in subsequent requests via `Authorization: Bearer <token>` header. Missing token = anonymous user (rate-limited by IP).
