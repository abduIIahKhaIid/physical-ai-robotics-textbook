"""Pydantic request/response models for the chat API."""

from __future__ import annotations

from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, Field, model_validator


class QueryFilters(BaseModel):
    """Optional metadata filters for retrieval queries."""

    modules: Optional[list[str]] = None
    chapters: Optional[list[str]] = None
    content_types: Optional[list[str]] = None
    tags: Optional[list[str]] = None
    top_k: int = Field(default=5, ge=1, le=20)


class ChatRequest(BaseModel):
    """POST /api/v1/chat request body."""

    message: str = Field(..., min_length=1, max_length=4000)
    session_id: Optional[UUID] = None
    mode: str = Field(default="normal", pattern=r"^(normal|selected_text_only)$")
    selected_text: Optional[str] = Field(default=None, max_length=10000)
    source_doc_path: Optional[str] = None
    source_section: Optional[str] = None
    filters: Optional[QueryFilters] = None

    @model_validator(mode="after")
    def validate_selected_text_required(self) -> "ChatRequest":
        if self.mode == "selected_text_only" and not self.selected_text:
            raise ValueError(
                "selected_text is required when mode is 'selected_text_only'"
            )
        return self


class Citation(BaseModel):
    """Source citation in a response."""

    title: str
    section: str
    url: str
    module: str = ""
    chapter: str = ""


class TokenEvent(BaseModel):
    """SSE token event data."""

    content: str


class DoneEvent(BaseModel):
    """SSE done event data."""

    message_id: str
    session_id: str
    citations: list[Citation] = Field(default_factory=list)
    persistence_warning: bool = False


class SessionSummary(BaseModel):
    """Session list item."""

    id: UUID
    title: str
    status: str
    message_count: int = 0
    created_at: datetime
    updated_at: datetime


class SessionList(BaseModel):
    """GET /api/v1/sessions response."""

    sessions: list[SessionSummary] = Field(default_factory=list)
    total: int = 0


class MessageResponse(BaseModel):
    """Message in session history."""

    id: UUID
    role: str
    content: str
    retrieval_mode: Optional[str] = None
    created_at: datetime


class MessageList(BaseModel):
    """GET /api/v1/sessions/{id}/messages response."""

    messages: list[MessageResponse] = Field(default_factory=list)
    session_id: UUID


class HealthComponent(BaseModel):
    """Health status of a single component."""

    status: str  # "ok" or "error"
    message: str = ""


class HealthResponse(BaseModel):
    """GET /api/v1/health response."""

    status: str  # "healthy" or "degraded"
    timestamp: datetime
    components: dict[str, HealthComponent] = Field(default_factory=dict)
