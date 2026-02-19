# Routing Guide: Skills & Agents

This guide maps tasks to the correct skill or agent. Consult it when you need to know which automation tool to use.

## Quick Lookup: "I need to..."

| Task | Use | Type |
|------|-----|------|
| Create a new textbook chapter | `docusaurus-chapter-generator` | Skill |
| Update sidebar from course outline | `course-outline-to-sidebar` | Skill |
| Write hardware requirements docs | `hardware-requirements-writer` | Skill |
| Design RAG chunking strategy | `rag-chunking-and-metadata` | Skill |
| Set up a Qdrant collection | `qdrant-collection-setup` | Skill |
| Define Neon DB schema/migrations | `neon-schema-and-migrations` | Skill |
| Scaffold a FastAPI chat endpoint | `fastapi-chat-endpoint-scaffold` | Skill |
| Embed ChatKit widget in Docusaurus | `chatkit-embed-and-token-flow` | Skill |
| Define personalization rules | `personalization-rules-engine` | Skill |
| Add Urdu translation toggle | `urdu-translation-toggle-ux` | Skill |
| Write/expand multiple chapters | `curriculum-author-agent` | Agent |
| Review technical writing accuracy | `technical-accuracy-reviewer-agent` | Agent |
| Fix Docusaurus build/sidebar/routing | `docusaurus-architect-agent` | Agent |
| Build RAG pipeline (chunking→retrieval) | `rag-pipeline-engineer-agent` | Agent |
| Wire ChatKit + FastAPI end-to-end | `chatkit-fastapi-integrator-agent` | Agent |
| Validate build/deploy readiness | `qa-validation-agent` | Agent |

## Skills Reference (10)

### docusaurus-chapter-generator
- **Triggers**: "create a chapter", "add a lesson page", "generate chapter content", "write a Docusaurus page"
- **Example**: `/docusaurus-chapter-generator Module 1, Week 2, Lesson 1.2 — Introduction to ROS 2`
- **Related agent**: `curriculum-author-agent` (for multi-chapter work)

### course-outline-to-sidebar
- **Triggers**: "create sidebar from course outline", "set up docs structure", "update course navigation", "sync sidebar with chapters"
- **Example**: `/course-outline-to-sidebar` (provide outline as input)
- **Related agent**: `docusaurus-architect-agent` (for sidebar debugging)

### hardware-requirements-writer
- **Triggers**: "document hardware needs", "create equipment list", "write setup requirements", "hardware procurement guide"
- **Example**: `/hardware-requirements-writer` for ROS 2 + Isaac Sim course
- **Related agent**: `curriculum-author-agent` (when embedded in chapter content)

### rag-chunking-and-metadata
- **Triggers**: "design chunking strategy", "indexing textbook content", "metadata schema for RAG", "debug retrieval quality"
- **Example**: `/rag-chunking-and-metadata` with source directory `website/docs/`
- **Related agent**: `rag-pipeline-engineer-agent` (for full pipeline implementation)

### qdrant-collection-setup
- **Triggers**: "set up vector database", "create Qdrant collection", "add payload indexes", "manage dev/staging/prod collections"
- **Example**: `/qdrant-collection-setup` for new collection
- **Related agent**: `rag-pipeline-engineer-agent` (for retrieval integration)

### neon-schema-and-migrations
- **Triggers**: "set up database", "add tables for chat", "create migration", "define schema for sessions/messages"
- **Example**: `/neon-schema-and-migrations` init
- **Related agent**: `chatkit-fastapi-integrator-agent` (for backend integration)

### fastapi-chat-endpoint-scaffold
- **Triggers**: "scaffold chat backend", "create streaming endpoint", "add RAG chat API", "new FastAPI endpoint"
- **Example**: `/fastapi-chat-endpoint-scaffold`
- **Related agent**: `chatkit-fastapi-integrator-agent` (for full frontend-backend wiring)

### chatkit-embed-and-token-flow
- **Triggers**: "embed chat widget", "add ChatKit to Docusaurus", "implement selected text capture", "set up token flow"
- **Example**: `/chatkit-embed-and-token-flow`
- **Related agent**: `chatkit-fastapi-integrator-agent` (for end-to-end integration)

### personalization-rules-engine
- **Triggers**: "define personalization rules", "set up onboarding profiles", "chapter content variants", "beginner/advanced modes"
- **Example**: `/personalization-rules-engine` for beginner profile
- **Related agent**: `curriculum-author-agent` (for personalized chapter content)

### urdu-translation-toggle-ux
- **Triggers**: "add Urdu translation", "translation toggle UI", "RTL rendering support", "language switching"
- **Example**: `/urdu-translation-toggle-ux` for a chapter
- **Related agent**: `docusaurus-architect-agent` (for Docusaurus integration issues)

## Agents Reference (6)

### curriculum-author-agent
- **Delegates when**: Multi-chapter authoring, curriculum alignment verification, expanding outlines into full lesson content
- **Spec coverage**: 005-module-1-chapters, 006-module-2-lessons, 007-author-module-3, 008-humanoid-vla-concepts
- **Related skills**: `docusaurus-chapter-generator`, `hardware-requirements-writer`

### technical-accuracy-reviewer-agent
- **Delegates when**: Reviewing robotics/AI technical writing for correctness, tightening definitions, flagging overclaims or terminology errors
- **Spec coverage**: All content specs (005-008) during review phase
- **Related skills**: None (review-only agent)

### docusaurus-architect-agent
- **Delegates when**: Sidebar/IA changes, broken links, MDX compilation errors, doc routing/slug issues, deploy/CI fixes
- **Spec coverage**: 001-docusaurus-book-site, 001-homepage-redesign, 001-sidebar-ia-redesign
- **Related skills**: `course-outline-to-sidebar`

### rag-pipeline-engineer-agent
- **Delegates when**: Building or modifying RAG system — chunking design, embeddings, Qdrant schema, retrieval logic, evaluation harness
- **Spec coverage**: 009-rag-ingestion-retrieval
- **Related skills**: `rag-chunking-and-metadata`, `qdrant-collection-setup`

### chatkit-fastapi-integrator-agent
- **Delegates when**: End-to-end chatbot integration — ChatKit embedding, FastAPI endpoints, sessions, persistence, streaming, CORS
- **Spec coverage**: 010-fastapi-chat-backend, 011-embed-chatkit-ui
- **Related skills**: `fastapi-chat-endpoint-scaffold`, `chatkit-embed-and-token-flow`, `neon-schema-and-migrations`

### qa-validation-agent
- **Delegates when**: Validating acceptance criteria, build/link integrity, deploy readiness, backend health, selected-text-only mode correctness
- **Spec coverage**: Pre-merge validation for all specs
- **Related skills**: All validation skills as needed

## Spec-to-Agent Routing

| Spec Range | Primary Agent | Supporting Skills |
|------------|---------------|-------------------|
| 001-004 | `docusaurus-architect-agent` | `course-outline-to-sidebar` |
| 005-008 | `curriculum-author-agent` + `technical-accuracy-reviewer-agent` | `docusaurus-chapter-generator`, `hardware-requirements-writer` |
| 009 | `rag-pipeline-engineer-agent` | `rag-chunking-and-metadata`, `qdrant-collection-setup` |
| 010-011 | `chatkit-fastapi-integrator-agent` | `fastapi-chat-endpoint-scaffold`, `chatkit-embed-and-token-flow`, `neon-schema-and-migrations` |
| 012+ | Varies by feature | Varies |
| Pre-merge | `qa-validation-agent` | All validation skills as needed |

## Maintenance

When adding a new skill or agent, update this file:
1. Add a row to the **Quick Lookup** table
2. Add a subsection to the appropriate **Reference** section (Skills or Agents)
3. Update the **Spec-to-Agent Routing** table if the new skill/agent serves a specific spec range
4. Verify no orphan entries exist (every entry maps to an actual file)
