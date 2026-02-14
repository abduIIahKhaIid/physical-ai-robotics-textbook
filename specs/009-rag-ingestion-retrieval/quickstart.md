# Quickstart: RAG Ingestion & Retrieval

**Feature**: 009-rag-ingestion-retrieval
**Date**: 2026-02-14

## Prerequisites

1. Python 3.11+
2. Qdrant Cloud account (or local Qdrant via Docker)
3. OpenAI API key

## Setup

```bash
# From repo root
cd /workspaces/physical-ai-robotics-textbook

# Create virtual environment
python -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -e ".[dev]"

# Configure environment
cp .env.example .env
# Edit .env with your keys:
#   QDRANT_URL=https://your-cluster.qdrant.io:6333
#   QDRANT_API_KEY=your-api-key
#   OPENAI_API_KEY=sk-your-key
#   EMBEDDING_MODEL=text-embedding-3-small
#   QDRANT_COLLECTION=textbook_chunks
```

## Run Ingestion

```bash
# Ingest all documents
python -m rag.cli ingest --docs-dir website/docs/

# Ingest a single document
python -m rag.cli ingest-doc website/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md
```

Expected output:
```
Ingestion complete:
  Documents processed: 89
  Chunks created: ~2,500
  Chunks updated: 0
  Chunks deleted: 0
  Duration: ~3 minutes
```

## Test Retrieval

```bash
# Normal mode query
python -m rag.cli query "What are the core principles of Physical AI?"

# With module filter
python -m rag.cli query "How does ROS2 handle communication?" --module module-1

# Selected-text-only mode (for development testing)
python -m rag.cli query "Explain this concept" \
  --selected-text "Physical AI systems must operate in real-time, responding to changes in their environment as they occur." \
  --mode selected_text_only
```

## Run Tests

```bash
# All tests
pytest tests/ -v

# Unit tests only
pytest tests/unit/ -v

# Integration tests (requires .env configured)
pytest tests/integration/ -v

# Leak tests for selected-text mode
pytest tests/integration/test_selected_text_leak.py -v

# Evaluation harness (requires ingested data)
python -m rag.cli evaluate --test-set tests/evaluation/test_set.json
```

## Verify Acceptance Criteria

| Criterion | Command | Expected |
| --------- | ------- | -------- |
| SC-001: recall@5 >= 0.90 | `rag evaluate` | 18+/20 questions hit |
| SC-002: Selected-text isolation | `pytest tests/integration/test_selected_text_leak.py` | All pass, 0 Qdrant calls |
| SC-003: Ingestion < 10 min | `time rag ingest` | < 600 seconds |
| SC-004: Idempotent upserts | Run `rag ingest` twice, compare | Zero diff |
| SC-006: Query < 5s p95 | `pytest tests/integration/test_retrieval_pipeline.py` | p95 < 5000ms |
