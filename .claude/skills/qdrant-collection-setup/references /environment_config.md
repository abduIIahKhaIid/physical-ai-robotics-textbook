# Environment Configuration Guide

Best practices for managing dev, staging, and production Qdrant environments.

## Environment Strategy

### Development (Local)

**Purpose:** Rapid iteration, testing, debugging

```bash
# Local Qdrant instance
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Not needed for local

# Collection naming
COLLECTION_NAME=textbook_chunks_dev

# Vector settings
VECTOR_SIZE=384  # Use smaller model for speed (all-MiniLM-L6-v2)
```

**Setup:**
```bash
# Run local Qdrant with Docker
docker run -p 6333:6333 qdrant/qdrant

# Create dev collection
python scripts/setup_collection.py \
    --collection textbook_chunks_dev \
    --vector-size 384 \
    --recreate
```

### Staging (Cloud)

**Purpose:** Integration testing, final validation before prod

```bash
# Qdrant Cloud (free tier)
QDRANT_URL=https://xyz-example.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-staging-api-key

# Collection naming
COLLECTION_NAME=textbook_chunks_staging

# Vector settings - match production
VECTOR_SIZE=1536  # text-embedding-3-small or ada-002
```

**Setup:**
```bash
python scripts/setup_collection.py \
    --url $QDRANT_URL \
    --api-key $QDRANT_API_KEY \
    --collection textbook_chunks_staging \
    --vector-size 1536
```

### Production (Cloud)

**Purpose:** Live user traffic, high availability

```bash
# Qdrant Cloud (paid tier recommended)
QDRANT_URL=https://xyz-production.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-production-api-key

# Collection naming
COLLECTION_NAME=textbook_chunks

# Vector settings - optimized for quality
VECTOR_SIZE=3072  # text-embedding-3-large for best quality
```

**Setup:**
```bash
python scripts/setup_collection.py \
    --url $QDRANT_URL \
    --api-key $QDRANT_API_KEY \
    --collection textbook_chunks \
    --vector-size 3072 \
    --skip-test  # Don't add test data to prod
```

## Collection Naming Conventions

### Recommended Patterns

```
# Environment suffix
textbook_chunks_dev
textbook_chunks_staging
textbook_chunks  # Production (no suffix)

# Version suffix (for migrations)
textbook_chunks_v1
textbook_chunks_v2

# Feature branches
textbook_chunks_feature_multilang
textbook_chunks_experiment_ranking
```

### Anti-patterns

```
# DON'T: Generic names
chunks
data
vectors

# DON'T: Include dates
textbook_2025_01
textbook_jan

# DON'T: Personal names
johns_collection
test_alice
```

## Configuration Management

### Environment Variables

Create `.env` files:

**.env.development:**
```bash
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=
COLLECTION_NAME=textbook_chunks_dev
EMBEDDING_MODEL=all-MiniLM-L6-v2
VECTOR_SIZE=384
```

**.env.production:**
```bash
QDRANT_URL=https://xyz.cloud.qdrant.io:6333
QDRANT_API_KEY=${PROD_API_KEY}  # From secrets manager
COLLECTION_NAME=textbook_chunks
EMBEDDING_MODEL=text-embedding-3-large
VECTOR_SIZE=3072
```

### Python Configuration

```python
# config.py
import os
from dataclasses import dataclass
from dotenv import load_dotenv

@dataclass
class QdrantConfig:
    url: str
    api_key: str
    collection_name: str
    vector_size: int
    
    @classmethod
    def from_env(cls, env: str = "development"):
        """Load config from environment."""
        load_dotenv(f".env.{env}")
        
        return cls(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY", ""),
            collection_name=os.getenv("COLLECTION_NAME"),
            vector_size=int(os.getenv("VECTOR_SIZE")),
        )

# Usage
config = QdrantConfig.from_env("production")
```

## Migration Between Environments

### Dev → Staging

```bash
# 1. Export dev data structure (not vectors)
python scripts/export_schema.py \
    --collection textbook_chunks_dev \
    --output dev_schema.json

# 2. Create staging collection with same config
python scripts/setup_collection.py \
    --url $STAGING_URL \
    --api-key $STAGING_API_KEY \
    --collection textbook_chunks_staging \
    --vector-size 1536

# 3. Re-embed and insert with staging embedding model
# (Vectors from dev won't transfer - dimension mismatch)
```

### Staging → Production

```bash
# 1. Backup current production
python scripts/migrate_collection.py backup \
    --url $PROD_URL \
    --api-key $PROD_API_KEY \
    --collection textbook_chunks \
    --backup-name textbook_chunks_backup_$(date +%Y%m%d)

# 2. Validate staging is ready
python scripts/setup_collection.py \
    --url $STAGING_URL \
    --api-key $STAGING_API_KEY \
    --collection textbook_chunks_staging \
    --validate-only

# 3. Create production collection
python scripts/setup_collection.py \
    --url $PROD_URL \
    --api-key $PROD_API_KEY \
    --collection textbook_chunks \
    --vector-size 3072 \
    --recreate  # Careful! Deletes existing

# 4. Re-embed and load production data
# (Use production embedding model)
```

## Qdrant Cloud Setup

### Free Tier Limitations

- 1 cluster
- 1 GB RAM
- Suitable for: Development, small demos, hackathons
- Not suitable for: Production with >10K chunks

### Paid Tier Recommendations

For production textbook (estimate 50K chunks):

- **Memory:** 4-8 GB RAM
- **Reason:** Vectors + indexes + overhead
- **Cost:** ~$40-80/month (varies by region)

### Creating Cloud Cluster

1. Sign up at https://cloud.qdrant.io
2. Create cluster:
   - **Region:** Choose closest to users
   - **Size:** Start with 1GB (free) or 2GB (paid)
   - **Backups:** Enable for production
3. Get credentials:
   - **Cluster URL:** Copy from dashboard
   - **API Key:** Generate and store securely

### Security Best Practices

```bash
# NEVER commit API keys
echo ".env*" >> .gitignore

# Use environment variables
export QDRANT_API_KEY="your-key-here"

# Or use secrets manager
# AWS: aws secretsmanager get-secret-value
# GCP: gcloud secrets versions access
```

## Monitoring

### Collection Health Checks

```python
# health_check.py
from qdrant_client import QdrantClient

def check_collection_health(config):
    client = QdrantClient(url=config.url, api_key=config.api_key)
    
    try:
        info = client.get_collection(config.collection_name)
        
        print(f"✅ Collection: {config.collection_name}")
        print(f"   Points: {info.points_count}")
        print(f"   Vectors: {info.config.params.vectors.size}D")
        print(f"   Status: Healthy")
        
        return True
    except Exception as e:
        print(f"❌ Collection unhealthy: {e}")
        return False
```

### Automated Checks

```bash
# Add to cron or CI/CD
*/15 * * * * python health_check.py --env production
```

## Disaster Recovery

### Regular Backups

```bash
# Weekly backup script
#!/bin/bash
DATE=$(date +%Y%m%d)
python scripts/migrate_collection.py backup \
    --url $PROD_URL \
    --api-key $PROD_API_KEY \
    --collection textbook_chunks \
    --backup-name "backup_${DATE}"
```

### Recovery Process

```bash
# If production fails, restore from backup
python scripts/migrate_collection.py migrate \
    --url $PROD_URL \
    --api-key $PROD_API_KEY \
    --source backup_20250129 \
    --target textbook_chunks \
    --vector-size 3072
```

## Environment Promotion Checklist

Before promoting to production:

- [ ] Validate vector dimensions match embedding model
- [ ] Test search queries return relevant results  
- [ ] Verify payload indexes are created
- [ ] Check collection size vs. memory limits
- [ ] Backup current production (if exists)
- [ ] Test rollback procedure
- [ ] Monitor initial traffic for errors
- [ ] Verify API key permissions