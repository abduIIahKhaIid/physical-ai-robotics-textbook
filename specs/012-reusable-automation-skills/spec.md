# Feature Specification: Reusable Automation Skills

**Feature Branch**: `012-reusable-automation-skills`
**Created**: 2026-02-17
**Status**: Draft
**Input**: User description: "Create reusable intelligence: a set of skills and subagents that automate repetitive work (chapter creation, sidebar updates, RAG chunking/indexing, FastAPI endpoint scaffolding). Document when to invoke them."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Invoke a Skill to Generate a Textbook Chapter (Priority: P1)

A course author needs to produce a new Docusaurus chapter page that follows the project's writing conventions, learning-objective structure, and MDX formatting. Instead of manually copying a chapter template and filling sections by hand, the author invokes the `docusaurus-chapter-generator` skill, which produces a complete, consistent chapter aligned to the module/week/lesson hierarchy.

**Why this priority**: Chapter creation is the most frequent repetitive workflow in the project. Automating it with a reusable skill eliminates formatting drift and reduces authoring time per chapter.

**Independent Test**: Can be fully tested by invoking the skill with a module, week, and lesson identifier and verifying the output matches the chapter template structure, contains all required sections (objectives, key terms, explanation, lab, recap, quiz), and compiles in Docusaurus without errors.

**Acceptance Scenarios**:

1. **Given** a skill definition exists at `.claude/skills/docusaurus-chapter-generator/SKILL.md`, **When** the author triggers the skill with lesson metadata, **Then** a complete chapter file is produced with all mandatory sections populated and no template placeholders remaining.
2. **Given** the generated chapter, **When** the Docusaurus build runs, **Then** the chapter compiles without MDX or link errors.
3. **Given** the skill is listed in the system prompt, **When** the author asks "how do I create a new chapter?", **Then** the assistant identifies and recommends the correct skill by name.

---

### User Story 2 - Scaffold a FastAPI Chat Endpoint (Priority: P1)

A backend developer needs to add a new streaming chat endpoint with Qdrant retrieval, Neon Postgres session persistence, and SSE response format. Instead of manually wiring models, routes, and database calls, they invoke the `fastapi-chat-endpoint-scaffold` skill, which produces a working endpoint scaffold following the project's established patterns.

**Why this priority**: Backend endpoint scaffolding involves multiple files and strict conventions (SSE format, Pydantic models, async patterns). Getting these wrong causes integration failures downstream.

**Independent Test**: Can be tested by invoking the skill and verifying the scaffold includes request/response models, a streaming route, session persistence hooks, and Qdrant retrieval integration — all matching the patterns in the existing `rag/` and backend code.

**Acceptance Scenarios**:

1. **Given** the skill definition exists at `.claude/skills/fastapi-chat-endpoint-scaffold/SKILL.md`, **When** the developer triggers the skill, **Then** a scaffold is produced with route, models, streaming handler, and database integration stubs.
2. **Given** the scaffold, **When** compared against the existing backend conventions, **Then** it uses `sse_starlette.EventSourceResponse` with dict yields, Pydantic v2 models, and async database calls.

---

### User Story 3 - Delegate Complex Work to a Specialized Subagent (Priority: P1)

A developer working on a multi-step task (e.g., ingesting new textbook content into Qdrant, fixing a Docusaurus build error, or reviewing a chapter for technical accuracy) needs the right specialized agent to handle the work autonomously. The system routes the task to the correct subagent based on the task type.

**Why this priority**: Subagents encapsulate domain expertise and multi-step workflows that are too complex for a single skill invocation. Correct routing prevents wasted effort and context pollution.

**Independent Test**: Can be tested by presenting a task description matching each agent's domain and verifying the system identifies the correct agent.

**Acceptance Scenarios**:

1. **Given** 6 subagent definitions exist in `.claude/agents/`, **When** a task involves RAG chunking, embedding, or retrieval logic, **Then** the system routes to `rag-pipeline-engineer-agent`.
2. **Given** a task involves Docusaurus sidebar changes, broken links, or MDX compilation, **When** delegated, **Then** the system routes to `docusaurus-architect-agent`.
3. **Given** a task involves reviewing robotics/AI technical writing for correctness, **When** delegated, **Then** the system routes to `technical-accuracy-reviewer-agent`.

---

### User Story 4 - Discover Available Skills and Agents (Priority: P2)

A new contributor or the course author needs to understand what automation is available, when to use each skill vs. agent, and how to invoke them. They consult a central routing guide that lists every skill and agent with trigger conditions and examples.

**Why this priority**: Without discoverability, skills and agents go unused. A routing guide is the bridge between "automation exists" and "automation gets used."

**Independent Test**: Can be tested by verifying a routing document exists, lists all skills and agents, includes trigger conditions, and matches the actual files in `.claude/skills/` and `.claude/agents/`.

**Acceptance Scenarios**:

1. **Given** the routing guide exists, **When** a contributor reads it, **Then** they can identify which skill or agent to use for any of the 10 documented workflows.
2. **Given** the routing guide, **When** compared against the actual `.claude/skills/` and `.claude/agents/` directories, **Then** every skill and agent has an entry with no orphans or missing entries.

---

### User Story 5 - Update Sidebar from Course Outline Changes (Priority: P2)

A curriculum designer modifies the course outline (adding a new week or reordering lessons). Instead of manually editing `sidebars.js` and creating folder structures, they invoke the `course-outline-to-sidebar` skill, which synchronizes the sidebar configuration and file/folder structure with the updated outline.

**Why this priority**: Sidebar/IA mismatches cause broken navigation and are tedious to fix manually across nested structures.

**Independent Test**: Can be tested by modifying a course outline input and verifying the skill produces an updated `sidebars.js` and the corresponding directory structure.

**Acceptance Scenarios**:

1. **Given** the skill definition exists, **When** invoked with an updated course outline, **Then** the sidebar configuration and folder structure are updated to match.
2. **Given** the updated sidebar, **When** the Docusaurus build runs, **Then** navigation renders correctly with no broken links.

---

### Edge Cases

- What happens when a skill is invoked with missing or invalid parameters (e.g., a chapter skill called without a module identifier)?
- How does the system behave when a subagent encounters a task that spans multiple agent domains (e.g., a RAG pipeline change that also requires Docusaurus sidebar updates)?
- What happens when a skill's template or asset files are deleted or corrupted?
- How are skill version conflicts handled when the underlying project conventions change but the skill definition hasn't been updated?

## Requirements *(mandatory)*

### Functional Requirements

#### Skills (10 Required)

- **FR-001**: System MUST include a `docusaurus-chapter-generator` skill that produces complete textbook chapter pages aligned to module/week/lesson structure with all required pedagogical sections.
- **FR-002**: System MUST include a `course-outline-to-sidebar` skill that converts course outlines into Docusaurus sidebar configuration and file/folder structures.
- **FR-003**: System MUST include a `hardware-requirements-writer` skill that generates hardware requirements documentation for robotics course materials.
- **FR-004**: System MUST include a `rag-chunking-and-metadata` skill that defines chunking strategy and metadata schema for indexing textbook content into vector databases.
- **FR-005**: System MUST include a `qdrant-collection-setup` skill that creates and manages vector store collections with proper configuration, indexes, and lifecycle operations.
- **FR-006**: System MUST include a `neon-schema-and-migrations` skill that defines database schema and migration workflows for chatbot sessions, messages, and users.
- **FR-007**: System MUST include a `fastapi-chat-endpoint-scaffold` skill that scaffolds streaming chat endpoints with retrieval, session persistence, and authentication support.
- **FR-008**: System MUST include a `chatkit-embed-and-token-flow` skill that implements chat widget embedding in Docusaurus sites with secure token/session flow.
- **FR-009**: System MUST include a `personalization-rules-engine` skill that defines deterministic personalization rules for chapter content based on user profiles.
- **FR-010**: System MUST include a `urdu-translation-toggle-ux` skill that implements translation toggle UI with RTL rendering, caching, and API integration.

#### Subagents (6 Required)

- **FR-011**: System MUST include a `curriculum-author-agent` subagent responsible for writing, expanding, and standardizing course textbook chapters aligned to weekly breakdowns and learning outcomes.
- **FR-012**: System MUST include a `technical-accuracy-reviewer-agent` subagent responsible for reviewing robotics/AI technical writing for correctness, tightening definitions, and flagging overclaims.
- **FR-013**: System MUST include a `docusaurus-architect-agent` subagent responsible for sidebar/IA changes, doc routing, MDX integration issues, build failures, and deploy fixes.
- **FR-014**: System MUST include a `rag-pipeline-engineer-agent` subagent responsible for chunking/metadata design, embeddings, Qdrant schema, retrieval logic, and evaluation.
- **FR-015**: System MUST include a `chatkit-fastapi-integrator-agent` subagent responsible for end-to-end chatbot integration: ChatKit embedding, FastAPI endpoints, sessions, persistence, and retrieval wiring.
- **FR-016**: System MUST include a `qa-validation-agent` subagent responsible for validating acceptance criteria, build/link integrity, deploy readiness, and backend health checks.

#### Storage and Structure

- **FR-017**: All skills MUST be stored at `.claude/skills/<skill-name>/SKILL.md` with supporting assets, scripts, and references in subdirectories.
- **FR-018**: All subagents MUST be stored at `.claude/agents/<agent-name>.md` as project-level agent definitions.
- **FR-019**: Each skill MUST include a YAML frontmatter block with `name` and `description` fields, followed by 5 structured sections: Title (`# <Skill Name>`), Non-Negotiable Rules, Quick Start, Core Implementation Workflow, and Acceptance Checklist.
- **FR-020**: No skill or agent definition file MUST contain real secrets, API keys, or credentials. Placeholder values MUST be used in all examples.

#### Invocation and Routing

- **FR-021**: All 10 skills MUST be manually invocable by the user. Model auto-invocation follows Claude Code defaults (the assistant may suggest or invoke a skill when its description matches the user's request).
- **FR-022**: All 6 subagents MUST be available for delegation by the assistant when a task matches the agent's domain description, subject to user approval of the Task tool.
- **FR-023**: A routing guide MUST exist that maps each skill and agent to its trigger conditions, listing when to use each one with concrete examples.
- **FR-024**: Skills and agents MUST be discoverable through the system prompt (listed in the available skills/agents sections visible to the assistant).

#### Consistency and Quality

- **FR-025**: Every skill MUST follow the same SKILL.md template structure for consistency across the pack.
- **FR-026**: Every subagent MUST include: a description of its responsibility, the tools it has access to, and non-negotiable rules for its domain.
- **FR-027**: Skill output MUST match current repository conventions (e.g., Pydantic v2, async patterns, ESM imports, Docusaurus 3.x MDX format).

### Key Entities

- **Skill**: A reusable, manually-invoked automation unit stored as a SKILL.md file with optional supporting assets. Encapsulates domain knowledge for a specific repeatable workflow (e.g., chapter generation, endpoint scaffolding).
- **Subagent**: A specialized autonomous agent definition that handles complex multi-step tasks within a specific domain. Delegates work that requires sustained context and multi-tool coordination.
- **Routing Guide**: A central reference document that maps task types to the appropriate skill or agent, with trigger conditions and usage examples.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 10 skills are present at `.claude/skills/<name>/SKILL.md` and discoverable in the system prompt when Claude Code loads the project.
- **SC-002**: All 6 subagents are present at `.claude/agents/<name>.md` and available for task delegation.
- **SC-003**: A routing guide exists and covers 100% of skills and agents with trigger conditions and examples.
- **SC-004**: Every skill follows the standardized SKILL.md template structure (YAML frontmatter + 4 required sections).
- **SC-005**: Zero secrets or real credentials appear in any skill or agent definition file.
- **SC-006**: A new contributor can identify the correct skill or agent for a given task within 60 seconds by consulting the routing guide.
- **SC-007**: Each skill, when invoked for its intended purpose, produces output that compiles/runs without errors in the current project environment.

## Clarifications

### Session 2026-02-17

- Q: Are all 10 skills required, or should some (personalization, Urdu translation, hardware-requirements) be marked optional since their downstream features aren't yet implemented? → A: All 10 skills are required. No optional tier — everything ships.

## Assumptions

- Claude Code's `.claude/skills/` and `.claude/agents/` directories are the canonical storage locations for skills and subagents respectively.
- Skills are surfaced in the system prompt automatically by Claude Code when placed in `.claude/skills/`.
- Subagents are surfaced automatically when placed in `.claude/agents/`.
- The routing guide will be a documentation artifact (not executable code) that helps users and the assistant select the right tool.
- All 10 skills and 6 agents already exist in the repository in draft form and need validation, standardization, and documentation — not creation from scratch.
