# Feature Specification: RAG Ingestion & Retrieval for Docusaurus Textbook

**Feature Branch**: `009-rag-ingestion-retrieval`
**Created**: 2026-02-14
**Status**: Draft
**Input**: User description: "Build RAG ingestion + retrieval design for indexing Docusaurus markdown into Qdrant: chunking strategy, metadata schema, embedding strategy, retrieval + response policy. Must include 'selected text only mode' requirements and acceptance tests."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Full-Book Question Answering (Priority: P1)

A student reading the Physical AI & Humanoid Robotics textbook types a question into the RoboTutor chatbot (e.g., "What is the difference between forward and inverse kinematics?"). The system retrieves relevant passages from across the entire textbook, synthesizes an answer grounded in the source material, and returns it with citations pointing back to specific chapters and sections.

**Why this priority**: This is the core value proposition. Without full-book retrieval, the chatbot cannot answer general questions and has zero utility.

**Independent Test**: Can be fully tested by submitting 10 representative questions covering each module and verifying that retrieved chunks are relevant and answers are factually grounded in the textbook content.

**Acceptance Scenarios**:

1. **Given** the textbook content has been ingested into the vector store, **When** a student asks "What are the core principles of Physical AI?", **Then** the system retrieves chunks from Module 1.1 (Foundations of Physical AI) and returns an answer citing that chapter.
2. **Given** the textbook content has been ingested, **When** a student asks a cross-module question like "How does embodied cognition relate to sim-to-real transfer?", **Then** the system retrieves chunks from both Module 1.3 and Module 3.4 and synthesizes a coherent answer referencing both.
3. **Given** the textbook content has been ingested, **When** a student asks a question completely outside the textbook scope (e.g., "What is the capital of France?"), **Then** the system responds that the question is outside the textbook's coverage and suggests related topics it can help with.

---

### User Story 2 - Selected Text Only Mode (Priority: P1)

A student highlights a specific passage in a chapter (e.g., a paragraph about bipedal walking gait patterns) and asks a clarifying question. The system restricts its retrieval and response exclusively to the content of the highlighted text, without pulling in information from other chapters or sections.

**Why this priority**: This is a critical differentiation feature. Students need targeted explanations of specific passages they find confusing. Mixing in unrelated content degrades trust and learning outcomes. Co-equal priority with P1 because the spec explicitly requires it.

**Independent Test**: Can be fully tested by selecting a specific paragraph, asking a question about it, and verifying the response references only the selected content — never information from other sections.

**Acceptance Scenarios**:

1. **Given** a student has selected a paragraph about "Principle 2: Real-time Operation" from Chapter 1.1, **When** they ask "Why can't I pause the system?", **Then** the response is derived solely from the selected text and does not reference content from other chapters.
2. **Given** a student has selected a code snippet from a lab exercise, **When** they ask "Explain this code line by line", **Then** the response explains only the selected code, not unrelated code from other labs.
3. **Given** a student has selected a paragraph and asks a question unrelated to the selection (e.g., selects text about kinematics but asks about sensors), **Then** the system informs the student their question doesn't appear related to the selected text and offers to either answer from the selection context or switch to full-book mode.

---

### User Story 3 - Content Ingestion Pipeline (Priority: P2)

A course maintainer adds or updates a chapter in the Docusaurus textbook. The ingestion pipeline processes the new/updated markdown, chunks it according to the defined strategy, generates embeddings, and upserts the chunks into the vector store with full metadata. Previously indexed chunks for updated content are replaced, not duplicated.

**Why this priority**: The pipeline must exist for Stories 1 and 2 to function, but the maintainer-facing workflow is secondary to the student-facing experience.

**Independent Test**: Can be tested by adding a new markdown file to `website/docs/`, running the pipeline, and querying the vector store to confirm the content appears with correct metadata.

**Acceptance Scenarios**:

1. **Given** a new chapter markdown file is added to `website/docs/module-1/`, **When** the ingestion pipeline runs, **Then** the file is chunked, embedded, and stored in the vector store with metadata including module, chapter, section, and document path.
2. **Given** an existing chapter is updated with new content, **When** the ingestion pipeline runs, **Then** old chunks for that chapter are replaced with new ones (no stale duplicates remain).
3. **Given** a markdown file contains code blocks, tables, and nested headings, **When** the ingestion pipeline processes it, **Then** code blocks are kept intact within chunks (not split mid-block), and heading hierarchy is preserved in metadata.

---

### User Story 4 - Retrieval Quality Diagnostics (Priority: P3)

A developer or course maintainer can run a retrieval evaluation harness that tests a set of known question-answer pairs against the system, reporting precision, recall, and answer groundedness scores.

**Why this priority**: Essential for ongoing quality assurance but not required for initial launch.

**Independent Test**: Can be tested by running the evaluation harness against a curated test set and verifying it produces a structured report with per-question scores.

**Acceptance Scenarios**:

1. **Given** a test set of 20 question-answer pairs exists, **When** the evaluation harness runs, **Then** it reports retrieval precision@5, recall@5, and mean reciprocal rank for each question.
2. **Given** the evaluation harness has run, **When** a developer reviews results, **Then** questions with poor retrieval scores are flagged with the expected vs. actual retrieved chunks for debugging.

---

### Edge Cases

- What happens when a markdown file has no frontmatter? The system MUST still ingest the file, deriving metadata from the file path and heading structure.
- What happens when a student selects text that spans multiple sections or headings? The system MUST treat the entire selection as the retrieval scope, including all content within the selection boundary.
- What happens when the selected text is extremely short (fewer than 10 words)? The system MUST still respond based on the selection but MAY note that a longer selection would enable a more detailed answer.
- How does the system handle markdown-specific syntax (LaTeX math, Mermaid diagrams, admonitions)? The system MUST strip rendering-specific syntax during ingestion while preserving the semantic content (e.g., keep math expressions readable, extract text from admonitions).
- What happens when two chunks have near-identical content (e.g., repeated definitions across modules)? The system MUST deduplicate at retrieval time by filtering chunks with similarity above a configurable threshold.
- What happens when the vector store is empty or unreachable? The system MUST return a clear error message to the user rather than failing silently.

## Requirements *(mandatory)*

### Functional Requirements

#### Ingestion Pipeline

- **FR-001**: System MUST parse Docusaurus markdown files from the `website/docs/` directory tree, extracting frontmatter metadata (title, description, tags, sidebar_label, sidebar_position, learning-objectives) and body content.
- **FR-002**: System MUST chunk content using a heading-aware strategy that respects document structure — chunks MUST NOT split across heading boundaries (H1, H2, H3), code blocks, or table boundaries.
- **FR-003**: System MUST produce chunks within a target size range of 200–800 tokens, with a soft overlap of 50–100 tokens between adjacent chunks from the same section to preserve context continuity.
- **FR-004**: System MUST generate a unique, deterministic chunk ID derived from the document path and chunk position, enabling idempotent upserts (re-running ingestion on the same content produces the same IDs).
- **FR-005**: System MUST attach structured metadata to each chunk, including at minimum: document path, module identifier, chapter/section identifier, heading hierarchy (breadcrumb), chunk position within document, content type (prose, code, table, lab-exercise, quiz), frontmatter tags, and a content hash for change detection.
- **FR-006**: System MUST generate vector embeddings for each chunk using a configured embedding model with a consistent dimensionality across all chunks.
- **FR-007**: System MUST upsert chunks into the vector store. When re-ingesting a document, chunks with matching IDs MUST be overwritten, and chunks from the previous version that no longer exist MUST be deleted.
- **FR-008**: System MUST preserve code blocks as atomic units — a code block MUST NOT be split across multiple chunks. If a code block exceeds the maximum chunk size, it becomes its own chunk.

#### Retrieval & Response

- **FR-009**: System MUST support a "normal mode" query that searches across the full vector store using semantic similarity, returning the top-K most relevant chunks (configurable, default K=5).
- **FR-010**: System MUST support a "selected text only mode" query that restricts the response to ONLY the content provided in the user's text selection. In this mode, the system MUST NOT retrieve additional chunks from the vector store.
- **FR-011**: In selected text only mode, the system MUST receive the selected text as a separate input field alongside the user's question, and MUST use only that text as context for generating a response.
- **FR-012**: System MUST return source citations with every response, including the document title, section heading, and a navigable reference (document path or URL) for each source chunk used.
- **FR-013**: System MUST enforce a groundedness constraint: responses MUST be derived from retrieved/provided content. If the retrieved context is insufficient to answer the question, the system MUST state this explicitly rather than generating unsupported content.
- **FR-014**: System MUST support metadata-based filtering on retrieval queries, allowing callers to restrict search to specific modules, chapters, or content types (e.g., "only search lab exercises in Module 2").

#### Metadata Schema

- **FR-015**: Each chunk stored in the vector store MUST include the following payload fields: `doc_path` (string), `module` (string, e.g., "module-1"), `chapter` (string, e.g., "1.1-introduction-to-physical-ai"), `section_heading` (string), `heading_breadcrumb` (array of strings), `chunk_index` (integer), `content_type` (enum: prose | code | table | lab | quiz | assessment), `tags` (array of strings), `content_hash` (string), `title` (string), `word_count` (integer), `ingested_at` (ISO timestamp).
- **FR-016**: System MUST create payload indexes on `module`, `chapter`, `content_type`, and `tags` fields to enable efficient filtered retrieval.

### Key Entities

- **Document**: A single Docusaurus markdown file with frontmatter and body content. Identified by its path relative to `website/docs/`. Key attributes: path, module, chapter, frontmatter metadata.
- **Chunk**: A contiguous segment of a document, produced by the chunking strategy. Key attributes: unique ID, embedding vector, text content, metadata payload, position within parent document.
- **Query**: A user's question, optionally accompanied by selected text. Key attributes: question text, mode (normal | selected-text-only), selected text (if applicable), metadata filters (optional).
- **Retrieval Result**: A ranked set of chunks returned for a query, with similarity scores. Key attributes: chunks, scores, citations.
- **Ingestion Run**: A single execution of the ingestion pipeline, processing one or more documents. Key attributes: timestamp, documents processed, chunks created/updated/deleted.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: For a curated test set of 20 representative questions, the system retrieves at least one relevant chunk in the top-5 results for 90% or more of the questions (recall@5 >= 0.90).
- **SC-002**: In selected text only mode, 100% of responses reference ONLY the provided selection — zero responses include information from outside the selected text.
- **SC-003**: The ingestion pipeline processes the full textbook (approximately 80+ markdown files) in under 10 minutes on a standard development machine.
- **SC-004**: Re-ingesting an unchanged document produces zero new or modified chunks (idempotent operation verified by content hash).
- **SC-005**: Users interacting with the chatbot rate the relevance of answers at 4 or higher on a 5-point scale for 80% of queries (measured via optional feedback mechanism).
- **SC-006**: The system returns a response (including retrieval and generation) within 5 seconds for 95% of queries.

## Assumptions

- The textbook content in `website/docs/` follows the established frontmatter schema (title, description, tags, learning-objectives, sidebar_label, sidebar_position) as documented in `module-3/frontmatter-schema.md`. Files without frontmatter will be handled gracefully using path-derived metadata.
- The existing ChatbotTeaser component (RoboTutor) on the homepage will be the primary UI entry point, extended to support text selection mode on chapter pages.
- The embedding model, vector store, and LLM for response generation will be hosted externally (cloud APIs). The spec does not prescribe specific vendors — these are implementation decisions.
- The ingestion pipeline will run as an offline/batch process (not real-time on every git push), triggered manually or via CI.
- Content is English-only for the initial version.
- The vector store supports payload filtering and nearest-neighbor search with cosine similarity.
