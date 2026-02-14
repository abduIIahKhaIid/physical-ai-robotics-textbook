# Data Model: RAG Ingestion & Retrieval

**Feature**: 009-rag-ingestion-retrieval
**Date**: 2026-02-14

## Entities

### Document

Represents a parsed Docusaurus markdown file.

| Field | Type | Source | Required |
| ----- | ---- | ------ | -------- |
| `doc_path` | `str` | Path relative to `website/docs/` | Yes |
| `absolute_path` | `str` | Filesystem absolute path | Yes |
| `module` | `str` | Extracted from path (e.g., "module-1") | Yes |
| `chapter` | `str` | Extracted from path (e.g., "1.1-introduction-to-physical-ai") | Yes |
| `title` | `str` | Frontmatter `title` or first H1 heading | Yes |
| `description` | `str` | Frontmatter `description` or empty | No |
| `tags` | `list[str]` | Frontmatter `tags` or empty list | No |
| `learning_objectives` | `list[str]` | Frontmatter `learning-objectives` or empty | No |
| `sidebar_label` | `str` | Frontmatter `sidebar_label` or title | No |
| `sidebar_position` | `int` | Frontmatter `sidebar_position` or 0 | No |
| `content` | `str` | Markdown body after frontmatter | Yes |
| `content_type` | `ContentType` | Derived from filename pattern | Yes |

**ContentType enum**: `prose` | `code` | `table` | `lab` | `quiz` | `assessment`

**Content type detection rules**:
- Filename contains "lab" or "exercise" → `lab`
- Filename contains "quiz" → `quiz`
- Filename contains "capstone" or "assessment" or "evaluation" → `assessment`
- All others → `prose`

### Chunk

Represents an indexed unit stored in Qdrant.

| Field | Type | Description | Required |
| ----- | ---- | ----------- | -------- |
| `id` | `str` | SHA-256(`doc_path::chunk_index`)[:16] | Yes |
| `doc_path` | `str` | Parent document path | Yes |
| `text` | `str` | Chunk content (rendering syntax stripped) | Yes |
| `embedding` | `list[float]` | 1536-dim vector | Yes |
| `metadata` | `ChunkMetadata` | Qdrant payload | Yes |

### ChunkMetadata (Qdrant Payload)

| Field | Type | Qdrant Index | Description |
| ----- | ---- | ------------ | ----------- |
| `doc_path` | `str` | No | Source file path |
| `module` | `str` | Keyword | Module identifier |
| `chapter` | `str` | Keyword | Chapter identifier |
| `section_heading` | `str` | No | Nearest heading above chunk |
| `heading_breadcrumb` | `list[str]` | No | Full heading hierarchy |
| `chunk_index` | `int` | No | 0-based position in document |
| `content_type` | `str` | Keyword | prose/code/table/lab/quiz/assessment |
| `tags` | `list[str]` | Keyword | From frontmatter |
| `content_hash` | `str` | No | SHA-256 of chunk text |
| `title` | `str` | No | Document title |
| `word_count` | `int` | No | Word count of chunk |
| `ingested_at` | `str` | No | ISO 8601 timestamp |
| `url` | `str` | No | GitHub Pages URL with anchor |

### Query

| Field | Type | Required | Description |
| ----- | ---- | -------- | ----------- |
| `question` | `str` | Yes | User's question |
| `mode` | `str` | Yes | "normal" or "selected_text_only" |
| `selected_text` | `str` | If mode=selected | User's highlighted text |
| `source_doc_path` | `str` | If mode=selected | Document containing selection |
| `source_section` | `str` | If mode=selected | Section heading of selection |
| `filters` | `QueryFilters` | No | Optional metadata filters |

### QueryFilters

| Field | Type | Default | Description |
| ----- | ---- | ------- | ----------- |
| `modules` | `list[str]` | None | Filter by module(s) |
| `chapters` | `list[str]` | None | Filter by chapter(s) |
| `content_types` | `list[str]` | None | Filter by content type(s) |
| `tags` | `list[str]` | None | Filter by tag(s) |
| `top_k` | `int` | 5 | Number of results |

### RetrievalResult

| Field | Type | Description |
| ----- | ---- | ----------- |
| `chunks` | `list[ScoredChunk]` | Ranked chunks |
| `query` | `Query` | Original query |
| `mode` | `str` | Query mode used |
| `total_candidates` | `int` | Total before top-k |

### ScoredChunk

| Field | Type | Description |
| ----- | ---- | ----------- |
| `chunk_id` | `str` | Chunk identifier |
| `text` | `str` | Chunk content |
| `score` | `float` | Similarity score (0–1) |
| `metadata` | `ChunkMetadata` | Full payload |
| `citation` | `Citation` | Formatted citation |

### Citation

| Field | Type | Description |
| ----- | ---- | ----------- |
| `title` | `str` | Document title |
| `section` | `str` | Section heading |
| `url` | `str` | Full URL with anchor |
| `module` | `str` | Module identifier |
| `chapter` | `str` | Chapter identifier |

### GroundedResponse

| Field | Type | Description |
| ----- | ---- | ----------- |
| `context` | `str` | Formatted chunk texts for LLM |
| `system_instruction` | `str` | Grounding instruction |
| `citations` | `list[Citation]` | Sources used |
| `mode` | `str` | Query mode |
| `sufficient_context` | `bool` | Whether context is adequate |

### IngestionReport

| Field | Type | Description |
| ----- | ---- | ----------- |
| `docs_processed` | `int` | Number of documents |
| `chunks_created` | `int` | New chunks inserted |
| `chunks_updated` | `int` | Existing chunks overwritten |
| `chunks_deleted` | `int` | Stale chunks removed |
| `duration_seconds` | `float` | Total time |
| `errors` | `list[str]` | Error messages (non-fatal) |

## Relationships

```
Document (1) ──produces──> (*) Chunk
Chunk (1) ──has──> (1) ChunkMetadata
Chunk (1) ──has──> (1) Citation
Query (1) ──produces──> (1) RetrievalResult
RetrievalResult (1) ──contains──> (*) ScoredChunk
ScoredChunk (1) ──references──> (1) Chunk
GroundedResponse (1) ──derived from──> (1) RetrievalResult
IngestionReport (1) ──summarizes──> (*) Document processing
```
