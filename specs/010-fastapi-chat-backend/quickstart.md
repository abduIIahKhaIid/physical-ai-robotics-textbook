# Quickstart: FastAPI Chat Backend

## Prerequisites

- Python 3.11+
- Neon Postgres database (or local Postgres for development)
- Google Gemini API key (for embeddings + LLM) — get one free at https://ai.google.dev
- Qdrant Cloud instance (or local Qdrant for development)

## Environment Variables

```bash
# Required
DATABASE_URL=postgresql://user:pass@host/dbname?sslmode=require
GEMINI_API_KEY=AIza...

# Required for normal mode (vector retrieval)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=...

# Optional (defaults shown)
QDRANT_COLLECTION=textbook_chunks
EMBEDDING_MODEL=gemini-embedding-001
EMBEDDING_DIMENSIONS=768
LLM_MODEL=gemini-2.0-flash
CORS_ORIGINS=https://abdullahkhalid.com
RATE_LIMIT_ANON=10
RATE_LIMIT_IDENTIFIED=20
LOG_LEVEL=INFO
```

## Local Development Setup

```bash
# 1. Install dependencies
pip install -e ".[api]"

# 2. Copy env template and fill values
cp .env.example .env
# Edit .env with your credentials

# 3. Run database migrations
python -m backend.db.migrate

# 4. Start the server
uvicorn backend.main:app --reload --port 8000

# 5. Test the health endpoint
curl http://localhost:8000/api/v1/health
```

## Quick Test

```bash
# Send a chat message (normal mode)
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}' \
  --no-buffer

# Send a selected-text-only message
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this concept",
    "mode": "selected_text_only",
    "selected_text": "Physical AI systems must operate in real-time."
  }' \
  --no-buffer
```

## Running Tests

```bash
# All tests
pytest tests/

# Backend tests only
pytest tests/unit/test_backend/ tests/integration/test_backend/

# With coverage
pytest tests/ --cov=backend --cov-report=term-missing
```
