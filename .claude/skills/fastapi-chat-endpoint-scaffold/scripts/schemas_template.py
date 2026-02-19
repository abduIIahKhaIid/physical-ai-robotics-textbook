"""
Pydantic Schemas for Chat API

Defines strict request/response contracts with validation.
"""

from pydantic import BaseModel, Field, validator
from typing import Optional, List, Literal, Dict, Any
from datetime import datetime
import uuid


class ChatRequest(BaseModel):
    """
    Request schema for /chat endpoint.
    
    Fields:
    - session_id: UUID for session continuity (auto-generated if null)
    - message: User's question/prompt
    - selected_text: Text selected by user (triggers selection-only mode)
    - lang: Response language (en/ur)
    - mode: Query mode (auto-set to selection_only if selected_text present)
    """
    session_id: Optional[str] = Field(
        None,
        description="Session UUID for conversation continuity"
    )
    message: str = Field(
        ...,
        min_length=1,
        max_length=5000,
        description="User's question or prompt"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=10000,
        description="Text selected by user (triggers selection-only mode)"
    )
    lang: Optional[Literal["en", "ur"]] = Field(
        "en",
        description="Response language"
    )
    mode: Optional[Literal["normal", "selection_only"]] = Field(
        "normal",
        description="Query mode (auto-set if selected_text provided)"
    )
    
    @validator('mode', always=True)
    def set_mode_based_on_selection(cls, v, values):
        """Auto-set mode to selection_only if selected_text is present"""
        if 'selected_text' in values and values['selected_text']:
            return "selection_only"
        return v
    
    class Config:
        schema_extra = {
            "example": {
                "message": "What is ROS 2?",
                "lang": "en",
                "mode": "normal"
            }
        }


class Citation(BaseModel):
    """Citation from retrieved context"""
    text: str = Field(..., description="Retrieved text chunk")
    source: str = Field(..., description="Source document/section")
    score: float = Field(..., description="Relevance score")
    
    class Config:
        schema_extra = {
            "example": {
                "text": "ROS 2 is a robot middleware that enables communication...",
                "source": "Module 1: The Robotic Nervous System",
                "score": 0.89
            }
        }


class ChatResponse(BaseModel):
    """
    Response schema for /chat endpoint.
    
    Fields:
    - session_id: UUID for session (same as request or newly generated)
    - reply: Assistant's response
    - citations: List of retrieved chunks (null in selection_only mode)
    - used_mode: Actual mode used (normal or selection_only)
    - debug: Optional debug info (only in development)
    """
    session_id: str = Field(..., description="Session UUID")
    reply: str = Field(..., description="Assistant's response")
    citations: Optional[List[Citation]] = Field(
        None,
        description="Retrieved context chunks (null in selection-only mode)"
    )
    used_mode: Literal["normal", "selection_only"] = Field(
        ...,
        description="Mode used for this response"
    )
    debug: Optional[Dict[str, Any]] = Field(
        None,
        description="Debug information (dev only)"
    )
    
    class Config:
        schema_extra = {
            "example": {
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "reply": "ROS 2 (Robot Operating System 2) is a middleware...",
                "citations": [
                    {
                        "text": "ROS 2 is a robot middleware...",
                        "source": "Module 1",
                        "score": 0.89
                    }
                ],
                "used_mode": "normal"
            }
        }


class SessionCreate(BaseModel):
    """Request to create a new session"""
    user_id: Optional[str] = Field(None, description="User ID (if authenticated)")


class SessionResponse(BaseModel):
    """Session metadata"""
    id: str
    user_id: Optional[str]
    created_at: datetime
    updated_at: datetime
    message_count: int


class Message(BaseModel):
    """Message in a session"""
    id: int
    session_id: str
    role: Literal["user", "assistant"]
    content: str
    selected_text: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime


class SessionHistory(BaseModel):
    """Complete session with message history"""
    session: SessionResponse
    messages: List[Message]


class HealthResponse(BaseModel):
    """Health check response"""
    status: Literal["ok", "degraded", "error"]
    version: str
    database: Optional[str] = None
    vector_db: Optional[str] = None
    
    class Config:
        schema_extra = {
            "example": {
                "status": "ok",
                "version": "1.0.0",
                "database": "connected",
                "vector_db": "connected"
            }
        }