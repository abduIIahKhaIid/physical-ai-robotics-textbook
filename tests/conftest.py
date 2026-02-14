"""Shared test fixtures for RAG pipeline tests."""

import os
import tempfile
import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import pytest

from rag.models import (
    Chunk,
    ChunkMetadata,
    Citation,
    ContentType,
    Document,
    Query,
    QueryFilters,
    ScoredChunk,
)


FIXTURES_DIR = Path(__file__).parent / "fixtures"


@pytest.fixture
def sample_chapter_path():
    return str(FIXTURES_DIR / "sample_chapter.md")


@pytest.fixture
def sample_lab_path():
    return str(FIXTURES_DIR / "sample_lab.md")


@pytest.fixture
def sample_quiz_path():
    return str(FIXTURES_DIR / "sample_quiz.md")


@pytest.fixture
def no_frontmatter_path():
    return str(FIXTURES_DIR / "no_frontmatter.md")


@pytest.fixture
def sample_document():
    return Document(
        doc_path="module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md",
        absolute_path=str(FIXTURES_DIR / "sample_chapter.md"),
        module="module-1",
        chapter="1.1-introduction-to-physical-ai",
        title="1.1 Foundations of Physical AI",
        description="Deep dive into the core principles of Physical AI.",
        tags=["physical-ai", "foundations", "robotics"],
        content_type=ContentType.PROSE,
    )


@pytest.fixture
def sample_chunk():
    return Chunk(
        id="abcdef0123456789",
        doc_path="module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md",
        text="Physical AI is built upon several core principles.",
        embedding=[0.1] * 1536,
        metadata=ChunkMetadata(
            doc_path="module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md",
            module="module-1",
            chapter="1.1-introduction-to-physical-ai",
            section_heading="Core Principles of Physical AI",
            heading_breadcrumb=["1.1 Foundations", "Core Principles"],
            chunk_index=0,
            content_type="prose",
            tags=["physical-ai", "foundations"],
            content_hash="abc123",
            title="1.1 Foundations of Physical AI",
            word_count=8,
            ingested_at="2026-02-14T00:00:00Z",
            url="/physical-ai-robotics-textbook/docs/module-1/1.1-introduction-to-physical-ai/physical-ai-foundations#core-principles-of-physical-ai",
        ),
    )


@pytest.fixture
def sample_scored_chunk(sample_chunk):
    return ScoredChunk(
        chunk_id=sample_chunk.id,
        text=sample_chunk.text,
        score=0.92,
        metadata=sample_chunk.metadata,
        citation=Citation(
            title="1.1 Foundations of Physical AI",
            section="Core Principles of Physical AI",
            url=sample_chunk.metadata.url,
            module="module-1",
            chapter="1.1-introduction-to-physical-ai",
        ),
    )


@pytest.fixture
def sample_query():
    return Query(
        question="What are the core principles of Physical AI?",
        mode="normal",
        filters=QueryFilters(top_k=5),
    )


@pytest.fixture
def sample_selected_text_query():
    return Query(
        question="Explain this concept",
        mode="selected_text_only",
        selected_text="Physical AI systems must operate in real-time.",
        source_doc_path="module-1/1.1-introduction-to-physical-ai/physical-ai-foundations.md",
        source_section="Principle 2: Real-time Operation",
    )


@pytest.fixture
def temp_docs_dir():
    """Create a temp directory with sample docs mimicking website/docs/ structure."""
    tmpdir = tempfile.mkdtemp()
    mod_dir = os.path.join(tmpdir, "module-1", "1.1-introduction-to-physical-ai")
    os.makedirs(mod_dir)
    shutil.copy(
        str(FIXTURES_DIR / "sample_chapter.md"),
        os.path.join(mod_dir, "physical-ai-foundations.md"),
    )
    shutil.copy(
        str(FIXTURES_DIR / "sample_lab.md"),
        os.path.join(mod_dir, "physical-ai-lab-exercises.md"),
    )
    yield tmpdir
    shutil.rmtree(tmpdir)


@pytest.fixture
def mock_qdrant_client():
    mock = MagicMock()
    mock.get_collections.return_value = MagicMock(collections=[])
    mock.create_collection.return_value = True
    mock.upsert.return_value = MagicMock(status="completed")
    mock.search.return_value = []
    mock.scroll.return_value = ([], None)
    mock.delete.return_value = MagicMock(status="completed")
    return mock


@pytest.fixture
def mock_openai_client():
    mock = MagicMock()
    embedding_response = MagicMock()
    embedding_obj = MagicMock()
    embedding_obj.embedding = [0.1] * 1536
    embedding_response.data = [embedding_obj]
    mock.embeddings.create.return_value = embedding_response
    return mock
