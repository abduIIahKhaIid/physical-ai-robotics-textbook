---
name: qdrant-collection-setup
description: Create and manage Qdrant collections for textbook RAG pipelines. Use when setting up vector databases, changing embedding models, adding payload indexes, debugging retrieval issues, or managing dev/staging/prod collections. Handles vector configuration, payload schemas, indexes, safe lifecycle operations, and validation.
---

# Qdrant Collection Setup

Provision and manage Qdrant vector collections for RAG-powered textbook chatbots.

## Non-Negotiable Rules

1. **Validate collection existence before create** - Always check if a collection exists before attempting to create it; use `--validate-only` or equivalent check
2. **Use environment-specific naming** - Collections must include environment suffix (e.g., `_dev`, `_staging`) to prevent cross-environment conflicts
3. **No real API keys in code or config** - Use environment variables (`$QDRANT_API_KEY`) for all credentials; never hardcode keys in scripts or configuration files
4. **Match vector dimensions to embedding model** - Vector size must exactly match the chosen embedding model's output dimensions
5. **Backup before destructive operations** - Always create a backup before using `--recreate` or deleting collections

## When to Use This Skill

Use this skill when:

- **Initial setup**: Creating Qdrant collections for the first time
- **Model changes**: Switching embedding models (dimension changes)
- **Schema updates**: Adding new payload fields or indexes
- **Environment management**: Setting up dev/staging/prod collections
- **Troubleshooting**: Debugging retrieval latency or filter correctness
- **Migrations**: Moving between embedding models or Qdrant versions

## Quick Start

### Basic Collection Creation

```bash
# Local development (small embedding model)
python scripts/setup_collection.py \
    --collection textbook_chunks_dev \
    --vector-size 384 \
    --recreate

# Production (OpenAI embeddings)
python scripts/setup_collection.py \
    --url https://xyz.cloud.qdrant.io:6333 \
    --api-key $QDRANT_API_KEY \
    --collection textbook_chunks \
    --vector-size 1536 \
    --distance Cosine
```

### Common Vector Dimensions

- `text-embedding-ada-002`: 1536
- `text-embedding-3-small`: 1536
- `text-embedding-3-large`: 3072
- `all-MiniLM-L6-v2`: 384

## Core Implementation Workflow

### 1. First-Time Setup

For new projects starting from scratch:

```bash
# Step 1: Choose embedding model
# - Development: all-MiniLM-L6-v2 (384D, fast)
# - Production: text-embedding-3-large (3072D, best quality)

# Step 2: Create collection
python scripts/setup_collection.py \
    --collection textbook_chunks \
    --vector-size 1536 \
    --distance Cosine

# Step 3: Verify setup
python scripts/setup_collection.py \
    --collection textbook_chunks \
    --vector-size 1536 \
    --validate-only

# Collection is ready for embedding insertion
```

### 2. Changing Embedding Models

When switching models (e.g., from ada-002 to text-embedding-3-large):

```bash
# Step 1: Backup current collection
python scripts/migrate_collection.py backup \
    --collection textbook_chunks \
    --backup-name textbook_chunks_backup

# Step 2: Create new collection with new dimensions
python scripts/setup_collection.py \
    --collection textbook_chunks_v2 \
    --vector-size 3072 \
    --recreate

# Step 3: Re-embed content with new model and insert
# (This happens in your embedding pipeline, not this skill)

# Step 4: Validate new collection
python scripts/setup_collection.py \
    --collection textbook_chunks_v2 \
    --vector-size 3072 \
    --validate-only
```

### 3. Adding Payload Indexes

For faster filtered queries, create indexes on frequently-filtered fields:

```python
from qdrant_client import QdrantClient, models

client = QdrantClient(url="...", api_key="...")

# Add index for chapter filtering
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="chapter",
    field_schema=models.PayloadSchemaType.KEYWORD
)

# Add index for page range queries
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="page",
    field_schema=models.PayloadSchemaType.INTEGER
)
```

**Recommended indexes** (created automatically by setup script):
- `chapter`: keyword (chapter filtering)
- `section`: keyword (section filtering)
- `page`: integer (page range queries)
- `type`: keyword (content type filtering)

### 4. Environment Separation

Manage dev/staging/prod collections:

```bash
# Development (local)
python scripts/setup_collection.py \
    --url http://localhost:6333 \
    --collection textbook_chunks_dev \
    --vector-size 384

# Staging (cloud, matches prod config)
python scripts/setup_collection.py \
    --url https://staging.cloud.qdrant.io:6333 \
    --api-key $STAGING_KEY \
    --collection textbook_chunks_staging \
    --vector-size 1536

# Production (cloud)
python scripts/setup_collection.py \
    --url https://prod.cloud.qdrant.io:6333 \
    --api-key $PROD_KEY \
    --collection textbook_chunks \
    --vector-size 1536 \
    --skip-test  # No test data in prod
```

See `references/environment_config.md` for detailed environment management.

## Payload Schema Design

### Standard Schema

Every embedded chunk should include:

```python
payload = {
    # Required fields
    "text": "The actual chunk text...",
    "chapter": "chapter-1",
    "section": "1.1",
    "page": 42,
    "chunk_id": "unique-id",
    "type": "text",  # or "code", "heading", "list", "table"

    # Optional but recommended
    "title": "Section heading",
    "module": "module-1-ros",
    "week": 1,
    "has_code": True,
    "keywords": ["robotics", "ROS2"],
}
```

See `references/payload_schema.md` for complete schema reference and filtering examples.

## Safety Checks

### Pre-Creation Validation

The setup script automatically validates:

1. **Vector dimension**: Matches your embedding model
2. **Collection existence**: Prevents accidental overwrites (unless `--recreate`)
3. **Connection**: Verifies Qdrant is reachable
4. **Indexes**: Creates recommended payload indexes
5. **Test insertion**: Inserts and searches test vector

### Collection Health Check

```bash
# Validate existing collection
python scripts/setup_collection.py \
    --collection textbook_chunks \
    --vector-size 1536 \
    --validate-only

# Output shows:
# ✓ Collection exists
#   Vector size: 1536
#   Distance: COSINE
#   Points count: 45231
# ✅ Validation passed
```

## Troubleshooting

### Vector Dimension Mismatch

**Error**: `vector dimension mismatch, expected 1536 got 384`

**Solution**:
```bash
# Check collection config
python scripts/setup_collection.py \
    --collection textbook_chunks \
    --validate-only

# If mismatch, migrate to new collection
python scripts/migrate_collection.py migrate \
    --source textbook_chunks \
    --target textbook_chunks_v2 \
    --vector-size 1536
```

### Slow Filtered Queries

**Symptom**: Queries with filters take >1s

**Solution**: Add payload indexes
```python
# Index the filtered field
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="chapter",  # Field being filtered
    field_schema=models.PayloadSchemaType.KEYWORD
)
```

### Collection Doesn't Exist

**Error**: `Collection textbook_chunks not found`

**Solution**:
```bash
# Create the collection
python scripts/setup_collection.py \
    --collection textbook_chunks \
    --vector-size 1536
```

### API Key Authentication Failed

**Error**: `Unauthorized` or `403 Forbidden`

**Solution**:
```bash
# Verify API key is correct
echo $QDRANT_API_KEY

# Test connection
python scripts/setup_collection.py \
    --url https://xyz.cloud.qdrant.io:6333 \
    --api-key $QDRANT_API_KEY \
    --collection textbook_chunks \
    --validate-only
```

## Advanced Usage

### Custom Index Configuration

```python
# Create custom index with specific parameters
client.create_payload_index(
    collection_name="textbook_chunks",
    field_name="difficulty",
    field_schema=models.PayloadSchemaType.KEYWORD
)

# Use in queries
results = client.search(
    collection_name="textbook_chunks",
    query_vector=embedding,
    query_filter=models.Filter(
        must=[
            models.FieldCondition(
                key="difficulty",
                match=models.MatchValue(value="beginner")
            )
        ]
    )
)
```

### Collection Versioning

For zero-downtime migrations:

```bash
# Create v2 with new schema
python scripts/setup_collection.py \
    --collection textbook_chunks_v2 \
    --vector-size 3072

# Load data into v2
# (Run embedding pipeline)

# Validate v2 works
python scripts/setup_collection.py \
    --collection textbook_chunks_v2 \
    --validate-only

# Switch application to use v2
# Update config: COLLECTION_NAME=textbook_chunks_v2

# After validation, remove v1
# client.delete_collection("textbook_chunks")
```

## Reference Documentation

- **Payload Schema**: `references/payload_schema.md` - Complete payload field reference, filtering examples, and upsert strategies
- **Environment Config**: `references/environment_config.md` - Dev/staging/prod setup, Qdrant Cloud configuration, backup procedures

## Script Reference

### setup_collection.py

Main collection provisioning script.

**Required arguments:**
- `--vector-size`: Vector dimension (must match embedding model)

**Optional arguments:**
- `--url`: Qdrant server URL (default: http://localhost:6333)
- `--api-key`: API key for cloud deployments
- `--collection`: Collection name (default: textbook_chunks)
- `--distance`: Distance metric (Cosine/Euclidean/Dot, default: Cosine)
- `--recreate`: Delete and recreate if exists
- `--validate-only`: Only validate, don't create
- `--skip-test`: Skip test vector insertion

### migrate_collection.py

Collection migration and backup.

**Commands:**

```bash
# Backup collection
migrate_collection.py backup \
    --collection textbook_chunks \
    --backup-name textbook_chunks_backup

# Migrate to new schema
migrate_collection.py migrate \
    --source textbook_chunks \
    --target textbook_chunks_v2 \
    --vector-size 3072
```

## Best Practices

1. **Always validate** after creation with `--validate-only`
2. **Use environment suffixes** for collection names (dev/staging/prod)
3. **Backup before migrations** using `migrate_collection.py backup`
4. **Match vector dimensions** to your embedding model exactly
5. **Index filtered fields** for query performance
6. **Test in dev first** before applying to production
7. **Use deterministic IDs** to prevent duplicate points
8. **Monitor collection health** with regular validation checks

## Acceptance Checklist

- [ ] Collection created with correct vector dimensions matching embedding model
- [ ] Collection existence validated before and after creation
- [ ] Environment-specific naming convention applied (e.g., `_dev`, `_staging`, `_prod`)
- [ ] Payload indexes created for all filtered fields
- [ ] Test vector insertion and search verified
- [ ] No real API keys hardcoded in scripts or configuration
- [ ] Backup created before any destructive operation (`--recreate` or delete)
- [ ] Health check passes with `--validate-only`
