# Qdrant Payload Schema Reference

This document defines the standard payload schema for textbook RAG collections.

## Standard Textbook Payload Schema

### Required Fields

Every point inserted into the collection should include these fields:

```python
{
    "text": str,           # The actual text chunk content
    "chapter": str,        # Chapter identifier (e.g., "chapter-1", "introduction")
    "section": str,        # Section within chapter (e.g., "1.1", "ros-fundamentals")
    "page": int,          # Page number in original document
    "chunk_id": str,      # Unique identifier for this chunk
    "type": str,          # Content type: "text", "code", "heading", "list", "table"
}
```

### Optional Fields

Additional metadata that enhances retrieval:

```python
{
    "title": str,         # Chunk title or heading text
    "keywords": List[str], # Extracted keywords for this chunk
    "difficulty": str,    # "beginner", "intermediate", "advanced"
    "module": str,        # Course module (e.g., "module-1-ros")
    "week": int,          # Course week number
    "has_code": bool,     # Whether chunk contains code examples
    "language": str,      # Programming language if has_code=True
    "prereqs": List[str], # Prerequisites for understanding this chunk
    "related_chunks": List[str], # Related chunk_ids
}
```

### Metadata for User Context

When user authentication is enabled:

```python
{
    "difficulty_level": str,  # Personalized difficulty based on user profile
    "recommended_for": List[str], # User background types
}
```

## Payload Indexing Strategy

### High-Priority Indexes

Create indexes on fields used for filtering:

```python
indexes = {
    "chapter": "keyword",     # Fast chapter-based filtering
    "section": "keyword",     # Section-level filtering
    "page": "integer",        # Page range queries
    "type": "keyword",        # Filter by content type
    "module": "keyword",      # Module-based filtering
    "week": "integer",        # Week-based queries
    "has_code": "bool",       # Code example filtering
}
```

### When to Create Indexes

- **Always index**: Fields used in `filter` parameter of search queries
- **Don't index**: Fields only used for display (text, title)
- **Consider indexing**: Fields for analytics or potential future filtering

## Filter Query Examples

### Basic Chapter Filtering

```python
from qdrant_client import models

results = client.search(
    collection_name="textbook_chunks",
    query_vector=embedding,
    query_filter=models.Filter(
        must=[
            models.FieldCondition(
                key="chapter",
                match=models.MatchValue(value="chapter-1")
            )
        ]
    ),
    limit=5
)
```

### Multi-Condition Filtering

```python
# Find code examples in specific module
results = client.search(
    collection_name="textbook_chunks",
    query_vector=embedding,
    query_filter=models.Filter(
        must=[
            models.FieldCondition(
                key="module",
                match=models.MatchValue(value="module-3-isaac")
            ),
            models.FieldCondition(
                key="has_code",
                match=models.MatchValue(value=True)
            )
        ]
    ),
    limit=5
)
```

### Page Range Queries

```python
# Find content in page range
results = client.search(
    collection_name="textbook_chunks",
    query_vector=embedding,
    query_filter=models.Filter(
        must=[
            models.FieldCondition(
                key="page",
                range=models.Range(
                    gte=10,  # Greater than or equal to 10
                    lte=20   # Less than or equal to 20
                )
            )
        ]
    ),
    limit=5
)
```

### User-Selected Text Context

For chatbot responses based on selected text:

```python
# User selected text from specific section
results = client.search(
    collection_name="textbook_chunks",
    query_vector=embedding,
    query_filter=models.Filter(
        must=[
            models.FieldCondition(
                key="section",
                match=models.MatchValue(value="2.1")
            )
        ]
    ),
    limit=3  # Fewer results since context is already specific
)
```

## Upsert Strategies

### Batch Insertion

Efficient bulk loading:

```python
from qdrant_client import models

points = []
for idx, chunk in enumerate(chunks):
    points.append(
        models.PointStruct(
            id=idx,  # Or use UUID
            vector=embeddings[idx],
            payload={
                "text": chunk["text"],
                "chapter": chunk["chapter"],
                "section": chunk["section"],
                "page": chunk["page"],
                "chunk_id": chunk["id"],
                "type": chunk["type"],
            }
        )
    )

# Batch insert
client.upsert(
    collection_name="textbook_chunks",
    points=points,
    wait=True  # Wait for operation to complete
)
```

### Incremental Updates

Updating specific chunks:

```python
# Update metadata for existing chunks
client.set_payload(
    collection_name="textbook_chunks",
    payload={
        "difficulty": "intermediate",
        "updated_at": "2025-01-29"
    },
    points=[chunk_id_1, chunk_id_2, chunk_id_3]
)
```

### Safe Upsert Pattern

Avoid duplicates:

```python
import hashlib

def generate_chunk_id(text: str, chapter: str, section: str) -> str:
    """Generate deterministic ID for chunk."""
    content = f"{chapter}:{section}:{text[:100]}"
    return hashlib.sha256(content.encode()).hexdigest()[:16]

# Use deterministic IDs for upsert
chunk_id = generate_chunk_id(chunk_text, chapter, section)
client.upsert(
    collection_name="textbook_chunks",
    points=[
        models.PointStruct(
            id=chunk_id,  # Deterministic ID prevents duplicates
            vector=embedding,
            payload=payload
        )
    ]
)
```

## Schema Evolution

### Adding New Fields

Safe process for extending schema:

1. **Test on dev collection** with new fields
2. **Validate queries** work with optional fields
3. **Deploy to production** (existing points won't break)
4. **Backfill data** if needed

```python
# New fields are optional - old points still work
new_payload = {
    **existing_payload,
    "new_field": "value",  # Add new field
}
```

### Removing Fields

```python
# Delete specific payload keys
client.delete_payload(
    collection_name="textbook_chunks",
    keys=["deprecated_field"],
    points=[point_id]
)
```

## Performance Considerations

### Vector Dimensions

Common embedding models:

- `text-embedding-ada-002`: 1536 dimensions
- `text-embedding-3-small`: 1536 dimensions  
- `text-embedding-3-large`: 3072 dimensions
- `all-MiniLM-L6-v2`: 384 dimensions

### Collection Size Estimates

Memory calculation:

```
Memory per point ≈ (vector_size × 4 bytes) + payload_size + overhead

Example for 10,000 chunks with text-embedding-3-large:
- Vectors: 10,000 × 3072 × 4 = 122.88 MB
- Payload (~500 bytes avg): 10,000 × 500 = 4.88 MB
- Total: ~130 MB + overhead
```

### Optimization Tips

1. **Use appropriate vector dimensions**: Larger isn't always better
2. **Index only filtered fields**: Each index uses memory
3. **Batch operations**: Use batch upsert/search for efficiency
4. **Monitor collection stats**: Check `points_count` and `segments_count`
5. **Consider quantization**: For large collections, use scalar/product quantization

## Error Handling

### Common Issues

**Vector Dimension Mismatch:**
```python
try:
    client.upsert(collection_name, points)
except Exception as e:
    if "dimension" in str(e).lower():
        print(f"Vector dimension mismatch!")
        print(f"Expected: {expected_size}")
        print(f"Check embedding model output")
```

**Collection Doesn't Exist:**
```python
try:
    client.get_collection(collection_name)
except Exception as e:
    print(f"Collection not found. Run setup script first.")
```

**API Key Issues:**
```python
try:
    client = QdrantClient(url=url, api_key=api_key)
except Exception as e:
    print(f"Authentication failed. Check API key.")
```