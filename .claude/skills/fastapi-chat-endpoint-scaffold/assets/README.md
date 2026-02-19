# Physical AI Textbook Chat API

FastAPI backend for RAG-powered chatbot integrated with Physical AI & Humanoid Robotics textbook.

## Features

- ✅ **Dual Query Modes**: Normal (RAG-enhanced) and Selection-Only (focused answers)
- ✅ **Vector Search**: Qdrant Cloud integration for semantic retrieval
- ✅ **Session Persistence**: Neon Postgres for conversation history
- ✅ **Streaming Support**: Server-Sent Events for real-time responses
- ✅ **Multilingual**: English and Urdu support
- ✅ **Type-Safe**: Full Pydantic validation
- ⚡ **Optional**: Authentication, personalization, caching

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure Environment

```bash
cp .env.example .env
# Edit .env with your credentials
```

Required environment variables:
- `NEON_DATABASE_URL`: Postgres connection string
- `QDRANT_URL`: Qdrant Cloud endpoint
- `QDRANT_API_KEY`: Qdrant API key
- `OPENAI_API_KEY`: OpenAI API key

### 3. Initialize Database

```bash
python scripts/init_db.py
```

This creates the required tables:
- `sessions`: Session metadata
- `messages`: Message history

### 4. Start Server

```bash
uvicorn app.main:app --reload --port 8000
```

API will be available at: `http://localhost:8000`

## API Endpoints

### Health Check

```bash
GET /health
```

Response:
```json
{
  "status": "ok",
  "version": "1.0.0",
  "database": "connected",
  "vector_db": "connected"
}
```

### Chat (Normal Mode)

Uses RAG retrieval from Qdrant to enhance responses.

```bash
POST /api/chat
Content-Type: application/json

{
  "message": "What is ROS 2?",
  "lang": "en",
  "mode": "normal"
}
```

Response:
```json
{
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "reply": "ROS 2 (Robot Operating System 2) is...",
  "citations": [
    {
      "text": "ROS 2 is a middleware...",
      "source": "Module 1: The Robotic Nervous System",
      "score": 0.89
    }
  ],
  "used_mode": "normal"
}
```

### Chat (Selection-Only Mode)

Answers based ONLY on user-selected text, bypassing RAG retrieval.

```bash
POST /api/chat
Content-Type: application/json

{
  "message": "Explain this concept",
  "selected_text": "ROS 2 uses DDS for communication between nodes...",
  "mode": "selection_only"
}
```

Response:
```json
{
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "reply": "Based on the selected text, ROS 2 uses DDS...",
  "citations": null,
  "used_mode": "selection_only"
}
```

### Streaming Chat

```bash
POST /api/chat/stream
Content-Type: application/json

{
  "message": "Explain NVIDIA Isaac Sim",
  "mode": "normal"
}
```

Returns Server-Sent Events (SSE):
```
data: NVIDIA
data:  Isaac
data:  Sim
data:  is
data:  a
data:  photorealistic
...
data: [DONE]
```

### Sessions

```bash
# List user sessions
GET /api/sessions

# Get session history
GET /api/sessions/{session_id}

# Delete session
DELETE /api/sessions/{session_id}
```

## Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI app initialization
│   ├── config.py            # Environment configuration
│   ├── api/
│   │   └── routes/
│   │       ├── health.py    # Health endpoint
│   │       ├── chat.py      # Chat endpoints
│   │       └── sessions.py  # Session management
│   ├── models/
│   │   └── schemas.py       # Pydantic models
│   ├── services/
│   │   ├── database.py      # Postgres operations
│   │   ├── rag.py           # Qdrant retrieval
│   │   └── chat.py          # LLM integration
│   └── db/
│       └── session.py       # Session models
├── requirements.txt
├── .env.example
└── README.md
```

## Key Concepts

### Selection-Only Mode

When a user selects text in the UI and asks a question:

1. **Frontend** sends both `message` and `selected_text`
2. **Backend** automatically sets `mode="selection_only"`
3. **RAG retrieval is bypassed** entirely
4. **LLM receives only the selected text** as context
5. **Response is constrained** to information in the selection

This ensures focused, precise answers without hallucination.

### Session Management

Each conversation has a unique `session_id`:
- Auto-generated if not provided
- Persists message history in Postgres
- Enables conversation continuity across requests
- Supports multi-turn dialogues

### Citations

In normal mode, citations include:
- **text**: Retrieved chunk content
- **source**: Document/section reference
- **score**: Relevance score (0-1)

Use citations to help users verify information sources.

## Development

### Running Tests

```bash
pytest tests/
```

### Adding New Endpoints

1. Create route file in `app/api/routes/`
2. Define Pydantic schemas in `app/models/schemas.py`
3. Register route in `app/main.py`

### Database Migrations

```bash
# Create migration
alembic revision --autogenerate -m "Add new table"

# Apply migration
alembic upgrade head
```

## Deployment

### Docker

```dockerfile
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Environment Variables

Production checklist:
- [ ] `NEON_DATABASE_URL` configured
- [ ] `QDRANT_URL` and `QDRANT_API_KEY` set
- [ ] `OPENAI_API_KEY` configured
- [ ] `CORS_ORIGINS` restricted to production domains
- [ ] `ENABLE_DEBUG_MODE=false`

## Troubleshooting

### Database Connection Fails

```bash
# Test Neon connection
psql $NEON_DATABASE_URL
```

### Qdrant Returns No Results

```bash
# Verify collection exists
curl -X GET "$QDRANT_URL/collections" \
  -H "api-key: $QDRANT_API_KEY"
```

### OpenAI Rate Limits

Implement exponential backoff:
```python
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(stop=stop_after_attempt(3), wait=wait_exponential())
async def call_openai():
    # OpenAI API call
    pass
```

## License

MIT