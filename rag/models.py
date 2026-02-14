"""Pydantic data models for the RAG pipeline."""

from __future__ import annotations

import enum
from typing import Optional

from pydantic import BaseModel, Field, model_validator


class ContentType(str, enum.Enum):
    """Content type classification for documents and chunks."""

    PROSE = "prose"
    CODE = "code"
    TABLE = "table"
    LAB = "lab"
    QUIZ = "quiz"
    ASSESSMENT = "assessment"


class Document(BaseModel):
    """A parsed Docusaurus markdown file."""

    doc_path: str
    absolute_path: str = ""
    module: str = ""
    chapter: str = ""
    title: str = ""
    description: str = ""
    tags: list[str] = Field(default_factory=list)
    learning_objectives: list[str] = Field(default_factory=list)
    sidebar_label: str = ""
    sidebar_position: int = 0
    content: str = ""
    content_type: ContentType = ContentType.PROSE


class ChunkMetadata(BaseModel):
    """Qdrant payload schema for a chunk."""

    doc_path: str
    module: str = ""
    chapter: str = ""
    section_heading: str = ""
    heading_breadcrumb: list[str] = Field(default_factory=list)
    chunk_index: int = 0
    content_type: str = "prose"
    tags: list[str] = Field(default_factory=list)
    content_hash: str = ""
    title: str = ""
    word_count: int = 0
    ingested_at: str = ""
    url: str = ""


class Chunk(BaseModel):
    """An indexed unit stored in Qdrant."""

    id: str = ""
    doc_path: str
    text: str
    embedding: list[float] = Field(default_factory=list)
    metadata: ChunkMetadata


class Citation(BaseModel):
    """Source citation for a retrieval result."""

    title: str
    section: str
    url: str
    module: str = ""
    chapter: str = ""


class ScoredChunk(BaseModel):
    """A chunk with similarity score and citation."""

    chunk_id: str
    text: str
    score: float
    metadata: ChunkMetadata
    citation: Citation


class QueryFilters(BaseModel):
    """Optional metadata filters for retrieval queries."""

    modules: Optional[list[str]] = None
    chapters: Optional[list[str]] = None
    content_types: Optional[list[str]] = None
    tags: Optional[list[str]] = None
    top_k: int = 5


class Query(BaseModel):
    """A user's retrieval request."""

    question: str
    mode: str = "normal"  # "normal" | "selected_text_only"
    selected_text: Optional[str] = None
    source_doc_path: Optional[str] = None
    source_section: Optional[str] = None
    filters: Optional[QueryFilters] = None

    @model_validator(mode="after")
    def validate_selected_text_mode(self) -> "Query":
        if self.mode == "selected_text_only" and not self.selected_text:
            raise ValueError(
                "selected_text is required when mode is 'selected_text_only'"
            )
        if not self.question:
            raise ValueError("question must not be empty")
        return self


class RetrievalResult(BaseModel):
    """Ranked chunks returned for a query."""

    chunks: list[ScoredChunk] = Field(default_factory=list)
    query: Query
    mode: str
    total_candidates: int = 0


class GroundedResponse(BaseModel):
    """Response with groundedness constraints applied."""

    context: str
    system_instruction: str
    citations: list[Citation] = Field(default_factory=list)
    mode: str
    sufficient_context: bool


class IngestionReport(BaseModel):
    """Summary of an ingestion run."""

    docs_processed: int = 0
    chunks_created: int = 0
    chunks_updated: int = 0
    chunks_deleted: int = 0
    duration_seconds: float = 0.0
    errors: list[str] = Field(default_factory=list)
