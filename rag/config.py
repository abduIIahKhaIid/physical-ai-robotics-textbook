"""Environment configuration for RAG pipeline."""

import os

from pydantic import BaseModel, Field


class Settings(BaseModel):
    """RAG pipeline configuration loaded from environment variables."""

    qdrant_url: str = Field(default="http://localhost:6333")
    qdrant_api_key: str = Field(default="")
    gemini_api_key: str = Field(default="")
    embedding_model: str = Field(default="gemini-embedding-001")
    qdrant_collection: str = Field(default="textbook_chunks")
    embedding_dimensions: int = Field(default=768)
    chunk_min_tokens: int = Field(default=200)
    chunk_max_tokens: int = Field(default=800)
    chunk_target_tokens: int = Field(default=500)
    chunk_overlap_tokens: int = Field(default=50)
    batch_size: int = Field(default=100)
    dedup_threshold: float = Field(default=0.95)
    base_url: str = Field(default="/physical-ai-robotics-textbook/docs/")


def load_settings() -> Settings:
    """Load settings from environment variables."""
    return Settings(
        qdrant_url=os.environ.get("QDRANT_URL", "http://localhost:6333"),
        qdrant_api_key=os.environ.get("QDRANT_API_KEY", ""),
        gemini_api_key=os.environ.get("GEMINI_API_KEY", ""),
        embedding_model=os.environ.get("EMBEDDING_MODEL", "gemini-embedding-001"),
        qdrant_collection=os.environ.get("QDRANT_COLLECTION", "textbook_chunks"),
        embedding_dimensions=int(os.environ.get("EMBEDDING_DIMENSIONS", "768")),
    )
