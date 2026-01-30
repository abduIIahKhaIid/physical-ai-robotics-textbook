# Qdrant Schema Configuration

## Collection Setup

```python
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PayloadSchemaType,
    PayloadIndexInfo, PointStruct
)

# Initialize client
client = QdrantClient(
    url="https://your-cluster.qdrant.io",
    api_key="your-api-key"
)

# Create collection
collection_name = "textbook_chunks"

client.create_collection(
    collection_name=collection_name,
    vectors_config=VectorParams(
        size=1536,  # OpenAI text-embedding-3-small dimension
        distance=Distance.COSINE
    )
)
```

## Payload Schema with Indexes

```python
# Create indexes for frequently filtered fields
# This dramatically improves query performance

# Index: module_number (for filtering by module)
client.create_payload_index(
    collection_name=collection_name,
    field_name="module_number",
    field_schema=PayloadSchemaType.INTEGER
)

# Index: page_url (for exact page lookups)
client.create_payload_index(
    collection_name=collection_name,
    field_name="page_url",
    field_schema=PayloadSchemaType.KEYWORD
)

# Index: is_selected_text (for selected-text-only mode)
client.create_payload_index(
    collection_name=collection_name,
    field_name="is_selected_text",
    field_schema=PayloadSchemaType.BOOL
)

# Index: conversation_id (for user session filtering)
client.create_payload_index(
    collection_name=collection_name,
    field_name="conversation_id",
    field_schema=PayloadSchemaType.KEYWORD
)

# Index: content_type (text, code, table, etc.)
client.create_payload_index(
    collection_name=collection_name,
    field_name="content_type",
    field_schema=PayloadSchemaType.KEYWORD
)

# Index: has_code (for code-specific queries)
client.create_payload_index(
    collection_name=collection_name,
    field_name="has_code",
    field_schema=PayloadSchemaType.BOOL
)

# Index: difficulty_level
client.create_payload_index(
    collection_name=collection_name,
    field_name="difficulty_level",
    field_schema=PayloadSchemaType.KEYWORD
)

# Index: programming_languages (for filtering by language)
client.create_payload_index(
    collection_name=collection_name,
    field_name="programming_languages",
    field_schema=PayloadSchemaType.KEYWORD
)
```

## Complete Payload Example

```python
chunk_payload = {
    # Core identifiers
    "chunk_id": "a3f2b8c9-1d4e-4f5a-9b2c-7e8d9f0a1b2c",
    "document_id": "docs/module-1/ros2-fundamentals",
    "chunk_index": 5,
    "content": "ROS 2 uses a publish-subscribe pattern...",
    "content_type": "text",
    
    # Document context
    "course_name": "Physical AI & Humanoid Robotics",
    "module_number": 1,
    "module_title": "The Robotic Nervous System",
    "week_number": 4,
    "page_title": "ROS 2 Communication Patterns",
    "page_url": "/docs/module-1/ros2-fundamentals",
    "section_hierarchy": ["Module 1", "ROS 2 Fundamentals", "Communication Patterns"],
    
    # Content metadata
    "headings": ["Communication Patterns", "Publish-Subscribe"],
    "primary_heading": "Publish-Subscribe",
    "has_code": True,
    "programming_languages": ["python"],
    "has_math": False,
    "has_images": False,
    "estimated_reading_time": 30,
    
    # Retrieval enhancement
    "keywords": ["ROS 2", "publish-subscribe", "publisher", "subscriber", "topics"],
    "learning_objectives": ["Understand publish-subscribe pattern", "Create publishers"],
    "prerequisites": ["Module 1 Week 3"],
    "difficulty_level": "beginner",
    "practical_exercise": True,
    
    # Selected text fields (default false)
    "is_selected_text": False,
    "selection_timestamp": None,
    "selection_parent_chunk": None,
    "user_id": None,
    "conversation_id": None
}
```

## Upserting Points

```python
from uuid import uuid4

def upsert_chunk(client, collection_name, chunk_payload, embedding):
    """
    Insert or update a chunk in Qdrant.
    """
    point = PointStruct(
        id=str(uuid4()),
        vector=embedding,
        payload=chunk_payload
    )
    
    client.upsert(
        collection_name=collection_name,
        points=[point]
    )
```

## Batch Upsert for Performance

```python
def batch_upsert_chunks(client, collection_name, chunks, embeddings, batch_size=100):
    """
    Batch upsert for better performance.
    """
    points = []
    
    for chunk, embedding in zip(chunks, embeddings):
        point = PointStruct(
            id=str(uuid4()),
            vector=embedding,
            payload=chunk
        )
        points.append(point)
    
    # Upsert in batches
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(
            collection_name=collection_name,
            points=batch,
            wait=True  # Wait for indexing
        )
        print(f"Uploaded batch {i // batch_size + 1}")
```

## Query Examples

### Basic Semantic Search

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

def search_chunks(client, collection_name, query_embedding, top_k=5):
    """
    Basic semantic search.
    """
    results = client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=top_k
    )
    return results
```

### Filtered Search by Module

```python
def search_by_module(client, collection_name, query_embedding, module_number, top_k=5):
    """
    Search within a specific module.
    """
    results = client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        query_filter=Filter(
            must=[
                FieldCondition(
                    key="module_number",
                    match=MatchValue(value=module_number)
                )
            ]
        ),
        limit=top_k
    )
    return results
```

### Selected Text Only Search

```python
def search_selected_text(client, collection_name, query_embedding, 
                        conversation_id, top_k=5):
    """
    Search only within user-selected text for current conversation.
    """
    from datetime import datetime, timedelta
    
    # Only search selections from last hour
    one_hour_ago = (datetime.now() - timedelta(hours=1)).isoformat()
    
    results = client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        query_filter=Filter(
            must=[
                FieldCondition(
                    key="is_selected_text",
                    match=MatchValue(value=True)
                ),
                FieldCondition(
                    key="conversation_id",
                    match=MatchValue(value=conversation_id)
                )
            ],
            should=[
                # Prefer recent selections
                FieldCondition(
                    key="selection_timestamp",
                    range={
                        "gte": one_hour_ago
                    }
                )
            ]
        ),
        limit=top_k,
        score_threshold=0.7  # Only return highly relevant results
    )
    return results
```

### Multi-Filter Search

```python
def advanced_search(client, collection_name, query_embedding, filters: dict, top_k=5):
    """
    Search with multiple filters.
    
    Example filters:
    {
        "module_number": 1,
        "has_code": True,
        "difficulty_level": "beginner",
        "programming_languages": ["python"]
    }
    """
    conditions = []
    
    if "module_number" in filters:
        conditions.append(
            FieldCondition(
                key="module_number",
                match=MatchValue(value=filters["module_number"])
            )
        )
    
    if "has_code" in filters:
        conditions.append(
            FieldCondition(
                key="has_code",
                match=MatchValue(value=filters["has_code"])
            )
        )
    
    if "difficulty_level" in filters:
        conditions.append(
            FieldCondition(
                key="difficulty_level",
                match=MatchValue(value=filters["difficulty_level"])
            )
        )
    
    if "programming_languages" in filters:
        # Match any of the specified languages
        conditions.append(
            FieldCondition(
                key="programming_languages",
                match=MatchValue(value=filters["programming_languages"][0])
            )
        )
    
    results = client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        query_filter=Filter(must=conditions) if conditions else None,
        limit=top_k
    )
    
    return results
```

## Cleanup Operations

### Delete Expired Selections

```python
from datetime import datetime, timedelta

def cleanup_expired_selections(client, collection_name, hours=24):
    """
    Delete selected text chunks older than specified hours.
    """
    cutoff_time = (datetime.now() - timedelta(hours=hours)).isoformat()
    
    client.delete(
        collection_name=collection_name,
        points_selector=Filter(
            must=[
                FieldCondition(
                    key="is_selected_text",
                    match=MatchValue(value=True)
                ),
                FieldCondition(
                    key="selection_timestamp",
                    range={"lt": cutoff_time}
                )
            ]
        )
    )
    print(f"Deleted selections older than {hours} hours")
```

### Delete by Document

```python
def delete_document_chunks(client, collection_name, document_id):
    """
    Delete all chunks from a specific document.
    Useful for re-indexing updated content.
    """
    client.delete(
        collection_name=collection_name,
        points_selector=Filter(
            must=[
                FieldCondition(
                    key="document_id",
                    match=MatchValue(value=document_id)
                )
            ]
        )
    )
    print(f"Deleted chunks from document: {document_id}")
```

## Performance Tuning

### Optimize Vector Search

```python
# Use hnsw parameters for better performance
from qdrant_client.models import HnswConfigDiff

client.update_collection(
    collection_name=collection_name,
    hnsw_config=HnswConfigDiff(
        m=16,  # Number of edges per node (higher = better recall, more memory)
        ef_construct=100,  # Size of dynamic candidate list (higher = better index)
    )
)
```

### Monitor Collection Stats

```python
def get_collection_stats(client, collection_name):
    """
    Get collection statistics.
    """
    info = client.get_collection(collection_name)
    
    print(f"Collection: {collection_name}")
    print(f"Points count: {info.points_count}")
    print(f"Vectors count: {info.vectors_count}")
    print(f"Indexed vectors: {info.indexed_vectors_count}")
    print(f"Status: {info.status}")
    
    return info
```

## Example: Complete Ingestion Pipeline

```python
from openai import OpenAI
from qdrant_client import QdrantClient

# Initialize clients
openai_client = OpenAI(api_key="your-openai-key")
qdrant_client = QdrantClient(url="https://your-cluster.qdrant.io", api_key="your-qdrant-key")

def ingest_textbook(textbook_chunks: list):
    """
    Complete ingestion pipeline.
    """
    collection_name = "textbook_chunks"
    
    # Generate embeddings in batch
    embeddings = []
    batch_size = 100
    
    for i in range(0, len(textbook_chunks), batch_size):
        batch = textbook_chunks[i:i + batch_size]
        texts = [chunk['content'] for chunk in batch]
        
        response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=texts
        )
        
        batch_embeddings = [item.embedding for item in response.data]
        embeddings.extend(batch_embeddings)
    
    # Upsert to Qdrant
    batch_upsert_chunks(
        qdrant_client,
        collection_name,
        textbook_chunks,
        embeddings,
        batch_size=100
    )
    
    print(f"Ingested {len(textbook_chunks)} chunks")
```