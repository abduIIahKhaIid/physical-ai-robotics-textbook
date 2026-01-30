---
name: fastapi-chat-endpoint-scaffold
description: Scaffold FastAPI backends for RAG chat applications with request/response models, streaming support, session persistence, Qdrant vector search, Neon Postgres database, selected-text-only mode, and optional authentication. Use when starting a new chat backend, integrating ChatKit UI, adding RAG capabilities, implementing streaming chat, or building endpoints for Physical AI/Humanoid Robotics textbook chatbots with user context awareness.
---

# FastAPI Chat Endpoint Scaffold

Generate production-ready FastAPI backends for RAG-powered chat applications with database persistence, vector search, and flexible query modes.

## Core Features

This skill scaffolds:
- Health and chat endpoints (`/health`, `/chat`, `/sessions`)
- Strict Pydantic request/response contracts
- Session management with Neon Postgres
- RAG retrieval with Qdrant vector database
- Selected-text-only mode (bypass retrieval for user selections)
- Streaming response support (SSE)
- Optional authentication integration (Better Auth)
- Multilingual support (English/Urdu)

## Scaffold Procedure

### 1. Project Structure Setup

Create the following directory structure:

```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py
│   ├── config.py
│   ├── api/
│   │   ├── __init__.py
│   │   └── routes/
│   │       ├── __init__.py
│   │       ├── health.py
│   │       ├── chat.py
│   │       └── sessions.py
│   ├── models/
│   │   ├── __init__.py
│   │   └── schemas.py
│   ├── services/
│   │   ├── __init__.py
│   │   ├── database.py
│   │   ├── rag.py
│   │   └── chat.py
│   └── db/
│       ├── __init__.py
│       └── session.py
├── requirements.txt
├── .env.example
└── README.md
```

### 2. Core Files Implementation

#### `app/config.py`
Environment configuration using Pydantic Settings:
- Database URLs (Neon Postgres)
- Vector DB config (Qdrant)
- OpenAI API keys
- CORS settings
- Streaming flags

#### `app/models/schemas.py`
Define strict Pydantic models:

```python
class ChatRequest(BaseModel):
    session_id: Optional[str] = None
    message: str
    selected_text: Optional[str] = None
    lang: Optional[Literal["en", "ur"]] = "en"
    mode: Optional[Literal["normal", "selection_only"]] = "normal"

class ChatResponse(BaseModel):
    session_id: str
    reply: str
    citations: Optional[List[Citation]] = None
    used_mode: Literal["normal", "selection_only"]
    debug: Optional[Dict[str, Any]] = None
```

**Critical Contract Rules:**
- If `selected_text` is present, set `mode` to `"selection_only"` automatically
- In `selection_only` mode, bypass Qdrant retrieval entirely
- Store both `message` and `selected_text` in database for context

### 3. Service Layer

#### `app/services/database.py`
Session persistence:
- Connect to Neon Postgres using asyncpg or SQLAlchemy async
- Create sessions table schema
- Implement create/read session operations
- Store message history with timestamps

#### `app/services/rag.py`
RAG retrieval logic:
- Connect to Qdrant Cloud (free tier)
- Implement vector search with metadata filtering
- Return top-k relevant chunks with scores
- Handle empty results gracefully
- **Must skip retrieval when `mode="selection_only"`**

#### `app/services/chat.py`
LLM integration:
- Use OpenAI SDK for completion requests
- Support streaming via SSE
- Handle `selection_only` mode with constrained prompts
- Format citations from retrieved chunks
- Implement language-specific system prompts (en/ur)

### 4. API Routes

#### `app/api/routes/health.py`
```python
@router.get("/health")
async def health_check():
    return {"status": "ok", "version": "1.0.0"}
```

#### `app/api/routes/chat.py`
POST `/chat` handler:
1. Validate ChatRequest schema
2. Create or retrieve session from database
3. Determine query mode (selection_only vs normal)
4. **If selection_only:** Call LLM with selected_text only (no RAG)
5. **If normal:** Retrieve from Qdrant, then call LLM with context
6. Store user message and assistant reply in session
7. Return ChatResponse with citations (if applicable)

**Streaming variant:** Use SSE to stream tokens with `yield` pattern

#### `app/api/routes/sessions.py`
- `GET /sessions` - List user sessions (requires auth)
- `GET /sessions/{id}` - Get session history
- `DELETE /sessions/{id}` - Delete session

### 5. Database Schema

**sessions table:**
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY,
    user_id VARCHAR(255),  -- null if no auth
    created_at TIMESTAMP,
    updated_at TIMESTAMP
);

CREATE TABLE messages (
    id SERIAL PRIMARY KEY,
    session_id UUID REFERENCES sessions(id),
    role VARCHAR(20),  -- 'user' or 'assistant'
    content TEXT,
    selected_text TEXT,  -- nullable
    metadata JSONB,
    created_at TIMESTAMP
);
```

### 6. Selection-Only Mode Implementation

**Critical behavior:**
When `selected_text` is provided:
1. Set `used_mode = "selection_only"` in response
2. Do NOT query Qdrant (skip retrieval entirely)
3. Pass only `selected_text` to LLM with constrained prompt
4. If selected_text lacks context, respond: *"The selected text doesn't provide enough information to answer. Please select more context or ask without selection."*
5. Store `selected_text` in database for audit trail

**Example prompt for selection-only:**
```
You are answering based ONLY on this selected text:
---
{selected_text}
---
User question: {message}

If the selection doesn't contain the answer, say so clearly.
```

### 7. Testing Checklist

Run these tests before deployment:
- [ ] `uvicorn app.main:app --reload` starts without errors
- [ ] `/health` returns 200 with version
- [ ] `/chat` with normal mode returns reply + citations
- [ ] `/chat` with `selected_text` uses selection_only mode
- [ ] `/chat` with insufficient selected_text responds appropriately
- [ ] Streaming endpoint yields tokens correctly
- [ ] Session persistence works (messages stored in DB)
- [ ] Qdrant retrieval returns relevant chunks
- [ ] CORS allows frontend origin

### 8. Deployment Artifacts

Generate these files:

**requirements.txt:**
```
fastapi==0.115.0
uvicorn[standard]==0.32.0
pydantic==2.9.0
pydantic-settings==2.5.0
openai==1.54.0
qdrant-client==1.12.0
asyncpg==0.30.0
python-dotenv==1.0.0
```

**.env.example:**
```
NEON_DATABASE_URL=postgresql://user:pass@host/db
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=xxx
OPENAI_API_KEY=sk-xxx
CORS_ORIGINS=http://localhost:3000
ENABLE_STREAMING=true
```

**README.md:**
Include setup instructions, API documentation, and example requests

## Non-Negotiable Rules

1. **Always validate with Pydantic** - No raw dict parsing
2. **Persist all interactions** - Store user + assistant messages
3. **Respect selection_only mode** - Never retrieve when selected_text present
4. **Handle empty results** - Graceful degradation for no retrieval matches
5. **CORS configuration** - Allow specified origins only
6. **Error logging** - Log all exceptions with context
7. **Type hints everywhere** - Full typing for maintainability

## Optional Enhancements

Reference `references/advanced_features.md` for:
- Better Auth integration (signup/signin)
- User background questionnaire at signup
- Personalized content based on user profile
- Urdu translation endpoints
- Rate limiting
- Caching layer (Redis)
- Metrics/observability

## Example Usage

```bash
# Start server
uvicorn app.main:app --reload --port 8000

# Test health
curl http://localhost:8000/health

# Normal chat (with RAG)
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "mode": "normal"}'

# Selection-only chat (no RAG)
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this",
    "selected_text": "ROS 2 is a robot middleware...",
    "mode": "selection_only"
  }'
```

## Success Criteria

- ✅ Server starts and responds to requests
- ✅ Health endpoint returns 200
- ✅ Chat endpoint handles both modes correctly
- ✅ Sessions persist across requests
- ✅ RAG retrieval works (normal mode)
- ✅ Selected text bypasses RAG (selection_only mode)
- ✅ Streaming works if enabled
- ✅ Error handling is robust