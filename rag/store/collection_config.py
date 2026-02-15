"""Qdrant collection configuration for the textbook chunks."""

from qdrant_client.models import Distance, PayloadSchemaType, VectorParams

COLLECTION_NAME = "textbook_chunks"

VECTOR_CONFIG = VectorParams(
    size=768,
    distance=Distance.COSINE,
)

PAYLOAD_INDEXES: list[dict] = [
    {"field_name": "module", "field_schema": PayloadSchemaType.KEYWORD},
    {"field_name": "chapter", "field_schema": PayloadSchemaType.KEYWORD},
    {"field_name": "content_type", "field_schema": PayloadSchemaType.KEYWORD},
    {"field_name": "tags", "field_schema": PayloadSchemaType.KEYWORD},
    {"field_name": "doc_path", "field_schema": PayloadSchemaType.KEYWORD},
]
