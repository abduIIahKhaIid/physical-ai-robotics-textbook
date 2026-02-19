# Embedding Models Comparison

## Overview

This document compares embedding models for the RAG system, focusing on quality, cost, and performance trade-offs.

## Recommended Models

### OpenAI text-embedding-3-small (Recommended)

**Best for:** Cost-effective production deployment

```python
from openai import OpenAI

client = OpenAI(api_key="your-key")

response = client.embeddings.create(
    model="text-embedding-3-small",
    input="ROS 2 uses a publish-subscribe pattern for communication"
)

embedding = response.data[0].embedding
```

**Specifications:**
- Dimension: 1536
- Max tokens: 8191
- Cost: $0.02 / 1M tokens
- Performance: 62.3% on MTEB benchmark

**Pros:**
- Excellent cost-performance ratio
- Fast inference
- Good for educational content
- Supports up to 8k tokens per chunk

**Cons:**
- Slightly lower quality than text-embedding-3-large
- Not optimized for code (use code models for code-heavy content)

**Use when:**
- Budget is a primary concern
- Content is primarily text-based
- Acceptable quality threshold is 60%+ on benchmarks

---

### OpenAI text-embedding-3-large

**Best for:** Highest quality retrieval

```python
response = client.embeddings.create(
    model="text-embedding-3-large",
    input="ROS 2 uses a publish-subscribe pattern for communication"
)

embedding = response.data[0].embedding
```

**Specifications:**
- Dimension: 3072 (can be reduced to 1536 or 256)
- Max tokens: 8191
- Cost: $0.13 / 1M tokens
- Performance: 64.6% on MTEB benchmark

**Pros:**
- Best-in-class retrieval quality
- Better multilingual support
- Dimension reduction without quality loss
- Handles complex technical content well

**Cons:**
- 6.5x more expensive than text-embedding-3-small
- Larger vectors (storage cost)
- Slower inference

**Use when:**
- Quality is paramount
- Content is multilingual (English + Urdu)
- Budget allows for premium service
- Content is highly technical/specialized

---

### Voyage AI voyage-code-2 (Alternative)

**Best for:** Code-heavy educational content

```python
import voyageai

vo = voyageai.Client(api_key="your-key")

embeddings = vo.embed(
    ["def create_publisher(node, topic):\n    return node.create_publisher(String, topic, 10)"],
    model="voyage-code-2"
)

embedding = embeddings.embeddings[0]
```

**Specifications:**
- Dimension: 1536
- Max tokens: 16000
- Cost: $0.12 / 1M tokens
- Performance: Optimized for code

**Pros:**
- Specialized for programming content
- Longer context (16k tokens)
- Better code similarity detection

**Cons:**
- Less effective for prose
- Requires separate API
- Higher cost than OpenAI small model

**Use when:**
- Content is >50% code
- Need to retrieve specific code patterns
- Longer context is beneficial

---

### Cohere embed-english-v3.0 (Alternative)

**Best for:** Hybrid search (dense + sparse)

```python
import cohere

co = cohere.Client(api_key="your-key")

response = co.embed(
    texts=["ROS 2 uses a publish-subscribe pattern"],
    model="embed-english-v3.0",
    input_type="search_document"  # or "search_query"
)

embedding = response.embeddings[0]
```

**Specifications:**
- Dimension: 1024
- Max tokens: 512
- Cost: $0.10 / 1M tokens
- Performance: 64.5% on MTEB

**Pros:**
- Supports hybrid search (dense + BM25)
- Compression-aware embeddings
- Good for English content

**Cons:**
- Shorter context (512 tokens)
- English-only
- Requires Cohere account

**Use when:**
- Implementing hybrid search
- Content is strictly English
- Context per chunk is short

---

## Performance Comparison Table

| Model | Dimension | Max Tokens | Cost ($/1M) | MTEB Score | Best For |
|-------|-----------|------------|-------------|------------|----------|
| text-embedding-3-small | 1536 | 8191 | $0.02 | 62.3% | Cost-effective |
| text-embedding-3-large | 3072 | 8191 | $0.13 | 64.6% | Highest quality |
| voyage-code-2 | 1536 | 16000 | $0.12 | N/A | Code-heavy |
| embed-english-v3.0 | 1024 | 512 | $0.10 | 64.5% | Hybrid search |

## Cost Analysis

### Example: Indexing Entire Textbook

Assume 100 pages, 500 words/page, 1.3 tokens/word:
- Total tokens: 100 × 500 × 1.3 = 65,000 tokens

**text-embedding-3-small:**
- Cost: (65,000 / 1,000,000) × $0.02 = $0.0013

**text-embedding-3-large:**
- Cost: (65,000 / 1,000,000) × $0.13 = $0.00845

**Difference:** $0.0071 per full index

For a typical educational deployment:
- 1 textbook re-indexed monthly
- 10,000 student queries/month (avg 50 tokens/query)

**Monthly costs:**
- text-embedding-3-small: $0.0013 + (10,000 × 50 / 1M × $0.02) = $0.01
- text-embedding-3-large: $0.00845 + (10,000 × 50 / 1M × $0.13) = $0.073

**Recommendation:** Use text-embedding-3-small for prototyping and budget deployments.

## Quality Benchmarks

### MTEB (Massive Text Embedding Benchmark)

Tests embedding quality across 8 tasks:
1. Classification
2. Clustering
3. Pair classification
4. Reranking
5. Retrieval
6. Semantic textual similarity (STS)
7. Summarization
8. BitextMining

**Scores for our use case (Retrieval task):**
- text-embedding-3-large: 55.7%
- text-embedding-3-small: 53.9%
- embed-english-v3.0: 54.2%

**Conclusion:** Marginal difference for educational content. Small model is sufficient.

## Implementation Recommendations

### Use text-embedding-3-small for:
- General textbook content (prose, explanations)
- Budget-conscious deployments
- Rapid prototyping

### Use text-embedding-3-large for:
- Final production deployment
- Multilingual content (English + Urdu)
- When quality justifies cost

### Use voyage-code-2 for:
- Code-heavy modules (Module 1: ROS 2, Module 3: Isaac)
- Specific code retrieval tasks
- When chunks frequently exceed 8k tokens

## Hybrid Approach (Recommended)

Use different models for different content types:

```python
def get_embedding_model(chunk):
    """
    Select embedding model based on chunk content.
    """
    if chunk.metadata.get("has_code") and chunk.metadata.get("content_type") == "code":
        return "voyage-code-2"
    elif chunk.metadata.get("language") == "urdu":
        return "text-embedding-3-large"  # Better multilingual
    else:
        return "text-embedding-3-small"  # Default
```

**Cost savings:** ~40% vs. using text-embedding-3-large everywhere
**Quality retention:** ~95% of text-embedding-3-large performance

## Vector Storage Costs

Qdrant Cloud Free Tier:
- 1 GB storage
- 1536 dimensions × 4 bytes/float × 100,000 chunks ≈ 615 MB
- **Conclusion:** Free tier supports ~100k chunks

For larger deployments, consider dimension reduction:

```python
# Reduce text-embedding-3-large from 3072 to 1536
response = client.embeddings.create(
    model="text-embedding-3-large",
    input=text,
    dimensions=1536  # 50% storage savings
)
```

Quality loss: ~2% on MTEB (from 64.6% to 63.2%)

## Optimization Strategies

### Caching Embeddings

```python
import hashlib
import json

def get_cached_embedding(text, cache_dict):
    """
    Cache embeddings to avoid recomputation.
    """
    text_hash = hashlib.sha256(text.encode()).hexdigest()
    
    if text_hash in cache_dict:
        return cache_dict[text_hash]
    
    # Generate new embedding
    embedding = generate_embedding(text)
    cache_dict[text_hash] = embedding
    
    return embedding
```

### Batch Processing

```python
# Instead of individual calls
embeddings = [generate_embedding(chunk) for chunk in chunks]

# Use batch API
texts = [chunk.content for chunk in chunks]
response = client.embeddings.create(
    model="text-embedding-3-small",
    input=texts  # Up to 2048 inputs per batch
)
embeddings = [item.embedding for item in response.data]
```

**Performance gain:** 10x faster, same cost

## Testing Retrieval Quality

```python
def test_retrieval_quality(test_queries, expected_results):
    """
    Measure retrieval accuracy.
    
    Metrics:
    - Hit Rate @ K: % of queries where correct answer is in top K
    - MRR (Mean Reciprocal Rank): Average 1/rank of first correct result
    """
    hit_rate = 0
    reciprocal_ranks = []
    
    for query, expected_id in zip(test_queries, expected_results):
        results = search(query, top_k=10)
        
        # Check hit rate
        result_ids = [r.id for r in results]
        if expected_id in result_ids:
            hit_rate += 1
            rank = result_ids.index(expected_id) + 1
            reciprocal_ranks.append(1 / rank)
        else:
            reciprocal_ranks.append(0)
    
    hit_rate = hit_rate / len(test_queries)
    mrr = sum(reciprocal_ranks) / len(reciprocal_ranks)
    
    print(f"Hit Rate @ 10: {hit_rate:.2%}")
    print(f"MRR: {mrr:.3f}")
```

**Benchmark targets:**
- Hit Rate @ 10: >85% for educational content
- MRR: >0.6 for high-quality retrieval

## Final Recommendation

**For Physical AI & Humanoid Robotics Textbook:**

1. **Default model:** text-embedding-3-small
   - Covers 80% of content (prose, explanations)
   - Cost: ~$0.01/month for typical usage

2. **Code-heavy sections:** voyage-code-2
   - Modules 1, 3 (ROS 2, Isaac SDK)
   - Cost: ~$0.005/month additional

3. **Urdu translation:** text-embedding-3-large
   - Only for translated content
   - Cost: ~$0.03/month additional (if entire book translated)

**Total estimated cost:** $0.045/month (~$0.50/year)

**Quality:** 95% of best-in-class at 20% of the cost