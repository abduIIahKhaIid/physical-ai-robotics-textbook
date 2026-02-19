---
name: rag-chunking-and-metadata
description: Design chunking strategy and metadata schema for indexing Docusaurus textbooks into Qdrant vector DB. Use when building RAG systems for educational content, implementing text-selection mode for chatbots, designing ingestion pipelines, or debugging poor retrieval quality. Handles MDX, Markdown, and code blocks with semantic chunking and rich metadata.
---

# RAG Chunking and Metadata Strategy

## Non-Negotiable Rules

- MUST preserve frontmatter metadata from source documents — never discard YAML frontmatter
- Chunks MUST NOT exceed configured `max_chunk_size` (default 1200 tokens)
- Every chunk payload MUST include `document_id` and `section_hierarchy`
- Never split mid-paragraph, mid-code-block, or mid-equation — respect semantic boundaries
- Every chunk MUST be classified by content type (text, code, table, list)

## Quick Start

```text
/rag-chunking-and-metadata
```

Provide the source content directory (e.g., `website/docs/`) and target Qdrant collection name. The skill will produce a chunking strategy, metadata schema, and ingestion pipeline design.

## Core Implementation Workflow

## Overview

This skill defines a production-ready chunking strategy and metadata schema for indexing Docusaurus textbooks into Qdrant, optimized for educational content with support for both full-document and selected-text-only retrieval modes.

## Core Principles

**Semantic Coherence**: Chunk boundaries respect document structure (sections, headings, paragraphs) rather than arbitrary character counts.

**Context Preservation**: Each chunk carries sufficient metadata to be understood independently during retrieval.

**Retrieval Flexibility**: Support both broad queries (course-level) and precise queries (selected text, specific code examples).

## Chunking Strategy

### Base Configuration

```python
CHUNK_CONFIG = {
    "target_size": 600,        # Target chunk size in tokens (~800 chars)
    "overlap": 100,             # Overlap between chunks in tokens
    "min_chunk_size": 200,      # Minimum viable chunk size
    "max_chunk_size": 1200,     # Hard limit before forced split
    "granularity": "section"    # Default: section | heading | paragraph
}
```

### Hierarchical Chunking Algorithm

**Level 1: Course Structure**
- Split by top-level sections (module boundaries)
- Each module becomes a parent context

**Level 2: Sections**
- Split by H2/H3 headings within modules
- Preserve heading hierarchy in metadata

**Level 3: Paragraphs**
- Final splits at paragraph boundaries
- Never split mid-paragraph unless exceeding max_chunk_size

**Special Cases:**

**Code Blocks**: Treat as atomic units. If code block exceeds max_chunk_size, keep intact and chunk by logical sections (functions, classes) with preserved syntax.

**Tables**: Keep entire table in single chunk when possible. For large tables, split by rows while preserving headers.

**Lists**: Keep list items together. Split only between major list sections.

**Math/Formulas**: Never split LaTeX blocks or equations.

### MDX-Specific Handling

```python
def chunk_mdx_content(content: str) -> List[Chunk]:
    """
    MDX files contain JSX, imports, and embedded React components.
    
    Rules:
    1. Preserve import statements as metadata, not chunk content
    2. Extract JSX component props as structured metadata
    3. Chunk the prose content between components
    4. Link component instances to their surrounding context
    """
    
    # Extract frontmatter
    frontmatter = extract_yaml_frontmatter(content)
    
    # Separate imports
    imports = extract_import_statements(content)
    
    # Parse JSX components
    components = parse_jsx_components(content)
    
    # Chunk remaining prose
    prose_chunks = semantic_split(
        remove_jsx(content),
        target_size=CHUNK_CONFIG["target_size"]
    )
    
    # Enrich chunks with component context
    for chunk in prose_chunks:
        chunk.metadata["nearby_components"] = find_nearby_components(
            chunk.position, components
        )
    
    return prose_chunks
```

## Metadata Schema

### Required Fields

Every chunk MUST include:

```python
{
    "chunk_id": "uuid-v4",                    # Unique identifier
    "document_id": "course/module/page",      # Source document path
    "chunk_index": 3,                         # Position in document (0-indexed)
    "content": "The actual text content...",  # The chunked text
    "content_type": "text|code|table|list",   # Content classification
}
```

### Document Context Fields

```python
{
    "course_name": "Physical AI & Humanoid Robotics",
    "module_number": 2,
    "module_title": "The Digital Twin",
    "week_number": 6,
    "page_title": "Gazebo Simulation Environment",
    "page_url": "/docs/module2/gazebo-intro",
    "section_hierarchy": ["Module 2", "Gazebo", "Basic Setup"],
}
```

### Content Metadata Fields

```python
{
    "headings": ["Introduction to ROS 2", "Core Concepts"],  # All parent headings
    "primary_heading": "Core Concepts",                      # Immediate parent heading
    "has_code": true,                                        # Contains code blocks
    "programming_languages": ["python", "bash"],             # Languages in code blocks
    "has_math": false,                                       # Contains LaTeX/equations
    "has_images": true,                                      # References images
    "image_paths": ["/img/ros2-architecture.png"],           # Image references
    "estimated_reading_time": 45,                            # Seconds to read
}
```

### Retrieval Enhancement Fields

```python
{
    "keywords": ["ROS 2", "nodes", "topics", "publish-subscribe"],  # Extracted key terms
    "learning_objectives": ["Understand ROS 2 nodes", "Create publishers"],  # If defined
    "prerequisites": ["Module 1"],                                   # Required background
    "difficulty_level": "intermediate",                              # beginner|intermediate|advanced
    "practical_exercise": true,                                      # Contains hands-on work
}
```

### Selected Text Mode Fields

**Critical for user-selected text queries:**

```python
{
    "is_selected_text": true,              # Marks user-selected chunks
    "selection_timestamp": "2025-01-29T10:30:00Z",
    "selection_parent_chunk": "uuid-parent",  # Links to broader context
    "user_id": "user-uuid",                # If authenticated
    "conversation_id": "conv-uuid",        # Session context
}
```

## Selected Text Only Mode

### Ingestion Strategy

When user selects text on the page, create a **temporary chunk** with high priority:

```python
def create_selected_text_chunk(
    selected_text: str,
    parent_chunk_id: str,
    user_context: dict
) -> Chunk:
    """
    Create ephemeral chunk for selected text queries.
    
    Lifecycle: Expires after 24h or when conversation ends.
    Priority: Highest during retrieval for same user/conversation.
    """
    
    chunk = Chunk(
        chunk_id=generate_uuid(),
        content=selected_text,
        metadata={
            "is_selected_text": true,
            "selection_timestamp": now(),
            "selection_parent_chunk": parent_chunk_id,
            "user_id": user_context["user_id"],
            "conversation_id": user_context["conversation_id"],
            
            # Inherit from parent
            **get_parent_metadata(parent_chunk_id),
            
            # Override priority
            "retrieval_priority": "selected_text",
        }
    )
    
    return chunk
```

### Retrieval Filter

During query execution, apply strict filtering:

```python
def build_selected_text_filter(user_context: dict) -> dict:
    """
    Qdrant filter for selected-text-only mode.
    """
    
    return {
        "must": [
            {
                "key": "is_selected_text",
                "match": {"value": true}
            },
            {
                "key": "conversation_id",
                "match": {"value": user_context["conversation_id"]}
            }
        ],
        "should": [
            # Prefer recent selections
            {
                "key": "selection_timestamp",
                "range": {
                    "gte": (now() - timedelta(hours=1)).isoformat()
                }
            }
        ]
    }
```

### Cleanup Strategy

```python
def cleanup_expired_selections():
    """
    Remove selected text chunks older than 24h.
    Run as scheduled job every 6 hours.
    """
    
    expired_timestamp = (now() - timedelta(hours=24)).isoformat()
    
    qdrant_client.delete(
        collection_name="textbook_chunks",
        points_selector={
            "filter": {
                "must": [
                    {
                        "key": "is_selected_text",
                        "match": {"value": true}
                    },
                    {
                        "key": "selection_timestamp",
                        "range": {"lt": expired_timestamp}
                    }
                ]
            }
        }
    )
```

## Implementation Checklist

**Setup Phase:**
- [ ] Define CHUNK_CONFIG based on content analysis
- [ ] Create Qdrant collection with proper schema
- [ ] Set up vector dimensions (match embedding model)
- [ ] Configure metadata indexing for filters

**Ingestion Phase:**
- [ ] Extract all MDX/Markdown files from Docusaurus
- [ ] Parse frontmatter and extract document metadata
- [ ] Apply semantic chunking algorithm
- [ ] Generate embeddings (OpenAI text-embedding-3-small or similar)
- [ ] Upload chunks with full metadata to Qdrant

**Selected Text Phase:**
- [ ] Implement frontend selection detection
- [ ] Create API endpoint for selected text ingestion
- [ ] Store with conversation_id and user_id
- [ ] Set up cleanup job for expired selections

**Retrieval Phase:**
- [ ] Implement dual-mode queries (full vs. selected)
- [ ] Apply metadata filters (module, week, difficulty)
- [ ] Re-rank results by relevance + metadata match
- [ ] Return source context for citations

## Quality Validation

### Chunk Quality Metrics

```python
def validate_chunk_quality(chunk: Chunk) -> dict:
    """
    Validate that chunks meet quality standards.
    """
    
    issues = []
    
    # Size checks
    if len(chunk.content) < CHUNK_CONFIG["min_chunk_size"]:
        issues.append("Chunk too small - may lack context")
    
    if len(chunk.content) > CHUNK_CONFIG["max_chunk_size"]:
        issues.append("Chunk too large - may dilute relevance")
    
    # Coherence checks
    if chunk.content.startswith(("and", "but", "however", "therefore")):
        issues.append("Chunk starts mid-thought - poor boundary")
    
    # Metadata completeness
    required_fields = ["course_name", "module_number", "page_title"]
    missing = [f for f in required_fields if f not in chunk.metadata]
    if missing:
        issues.append(f"Missing metadata: {missing}")
    
    return {
        "is_valid": len(issues) == 0,
        "issues": issues
    }
```

### Retrieval Quality Testing

Create test queries for each module:

```python
TEST_QUERIES = [
    {
        "query": "How do I create a ROS 2 publisher?",
        "expected_module": 1,
        "expected_keywords": ["ROS 2", "publisher", "node"]
    },
    {
        "query": "What GPU is required for Isaac Sim?",
        "expected_module": 3,
        "expected_keywords": ["NVIDIA", "RTX", "GPU"]
    }
]

def test_retrieval_quality():
    for test in TEST_QUERIES:
        results = qdrant_search(test["query"], top_k=5)
        
        # Check if top result is from expected module
        top_result = results[0]
        assert top_result.metadata["module_number"] == test["expected_module"]
        
        # Check keyword presence
        for keyword in test["expected_keywords"]:
            assert keyword.lower() in top_result.content.lower()
```

## Common Pitfalls

**Over-Chunking**: Creating chunks smaller than 200 tokens loses context. Increase target_size.

**Under-Chunking**: Chunks larger than 1200 tokens dilute relevance. Improve splitting logic.

**Metadata Bloat**: Don't store redundant data. Example: Don't store full page content in every chunk's metadata.

**Ignoring Structure**: Splitting mid-paragraph or mid-code-block breaks semantic coherence.

**Missing Selection Context**: Selected text chunks without parent_chunk_id lose broader context.

**No Cleanup**: Expired selections accumulate. Implement scheduled cleanup.

## Performance Optimization

**Batch Ingestion**: Upload chunks in batches of 100-500 to Qdrant.

**Async Embedding**: Generate embeddings asynchronously during ingestion.

**Index Metadata**: Create Qdrant payload indexes on frequently filtered fields (module_number, page_url, is_selected_text).

**Cache Embeddings**: Store computed embeddings to avoid recomputation on re-ingestion.

## Example: Complete Chunk

```python
{
    "chunk_id": "a3f2b8c9-1d4e-4f5a-9b2c-7e8d9f0a1b2c",
    "document_id": "docs/module-1/ros2-fundamentals",
    "chunk_index": 5,
    "content": "ROS 2 uses a publish-subscribe pattern for communication between nodes. Publishers send messages to topics, and subscribers receive those messages. This decoupled architecture allows for flexible system design.\n\nTo create a publisher in Python:\n\n```python\nimport rclpy\nfrom std_msgs.msg import String\n\npublisher = node.create_publisher(String, 'topic_name', 10)\n```",
    "content_type": "text",
    
    # Document Context
    "course_name": "Physical AI & Humanoid Robotics",
    "module_number": 1,
    "module_title": "The Robotic Nervous System",
    "week_number": 4,
    "page_title": "ROS 2 Communication Patterns",
    "page_url": "/docs/module-1/ros2-fundamentals",
    "section_hierarchy": ["Module 1", "ROS 2 Fundamentals", "Communication Patterns"],
    
    # Content Metadata
    "headings": ["Communication Patterns", "Publish-Subscribe"],
    "primary_heading": "Publish-Subscribe",
    "has_code": true,
    "programming_languages": ["python"],
    "has_math": false,
    "has_images": false,
    "estimated_reading_time": 30,
    
    # Retrieval Enhancement
    "keywords": ["ROS 2", "publish-subscribe", "publisher", "subscriber", "topics"],
    "learning_objectives": ["Understand publish-subscribe pattern", "Create publishers"],
    "prerequisites": ["Module 1 Week 3"],
    "difficulty_level": "beginner",
    "practical_exercise": true,
    
    # Not selected text (default)
    "is_selected_text": false
}
```

## Reference Files

**For detailed examples of chunking algorithms**: See `references/chunking-algorithms.md`

**For Qdrant schema configuration**: See `references/qdrant-schema.md`

**For embedding model comparison**: See `references/embedding-models.md`

**For MDX parsing utilities**: See `scripts/mdx_parser.py`

## Acceptance Checklist

- [ ] Chunks respect semantic boundaries (no mid-paragraph splits)
- [ ] All chunks under token limit
- [ ] Every chunk has `document_id` and `section_hierarchy` in payload
- [ ] Frontmatter metadata preserved in chunk metadata
- [ ] Content type classification applied to all chunks
- [ ] Selected-text-only mode chunks are properly isolated