"""Backend configuration loaded from environment variables."""

import json
import os

from dotenv import load_dotenv
from pydantic import BaseModel, Field

load_dotenv()


class BackendSettings(BaseModel):
    """Backend-specific settings extending RAG pipeline config."""

    database_url: str = Field(default="")
    gemini_api_key: str = Field(default="")
    cors_origins: list[str] = Field(default=["https://abdullahkhalid.com"])
    llm_model: str = Field(default="gemini-2.0-flash")
    embedding_model: str = Field(default="gemini-embedding-001")
    embedding_dimensions: int = Field(default=768)
    rate_limit_anon: int = Field(default=10)
    rate_limit_identified: int = Field(default=20)
    max_message_length: int = Field(default=4000)
    max_selected_text_length: int = Field(default=10000)
    max_history_messages: int = Field(default=10)
    better_auth_url: str = Field(default="http://localhost:4000")
    log_level: str = Field(default="INFO")
    router_timeout_s: int = Field(default=5)
    llm_timeout_s: int = Field(default=30)
    retrieval_timeout_s: int = Field(default=10)


def load_backend_settings() -> BackendSettings:
    """Load backend settings from environment variables."""
    cors_raw = os.environ.get("CORS_ORIGINS", '["https://abdullahkhalid.com"]')
    try:
        cors_origins = json.loads(cors_raw)
    except (json.JSONDecodeError, TypeError):
        cors_origins = [cors_raw]

    return BackendSettings(
        database_url=os.environ.get("DATABASE_URL", ""),
        gemini_api_key=os.environ.get("GEMINI_API_KEY", ""),
        cors_origins=cors_origins,
        llm_model=os.environ.get("LLM_MODEL", "gemini-2.0-flash"),
        embedding_model=os.environ.get("EMBEDDING_MODEL", "gemini-embedding-001"),
        embedding_dimensions=int(os.environ.get("EMBEDDING_DIMENSIONS", "768")),
        rate_limit_anon=int(os.environ.get("RATE_LIMIT_ANON", "10")),
        rate_limit_identified=int(os.environ.get("RATE_LIMIT_IDENTIFIED", "20")),
        max_message_length=int(os.environ.get("MAX_MESSAGE_LENGTH", "4000")),
        max_selected_text_length=int(
            os.environ.get("MAX_SELECTED_TEXT_LENGTH", "10000")
        ),
        max_history_messages=int(os.environ.get("MAX_HISTORY_MESSAGES", "10")),
        better_auth_url=os.environ.get("BETTER_AUTH_URL", "http://localhost:4000"),
        log_level=os.environ.get("LOG_LEVEL", "INFO"),
        router_timeout_s=int(os.environ.get("ROUTER_TIMEOUT_S", "5")),
        llm_timeout_s=int(os.environ.get("LLM_TIMEOUT_S", "30")),
        retrieval_timeout_s=int(os.environ.get("RETRIEVAL_TIMEOUT_S", "10")),
    )
