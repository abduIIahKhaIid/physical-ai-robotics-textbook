# Tasks: Reusable Automation Skills

**Input**: Design documents from `/specs/012-reusable-automation-skills/`
**Prerequisites**: plan.md (required), spec.md (required), research.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1–US5)
- All paths are relative to repository root

---

## Phase 1: Setup (Fix Structural Issues)

**Purpose**: Fix the trailing-space directory name bug across all skill subdirectories. This MUST complete before any content edits because file paths change.

- [x] T001 Rename all trailing-space subdirectories in `.claude/skills/chatkit-embed-and-token-flow/` — use `git mv` to rename `"assets "` → `assets`, `"scripts "` → `scripts`, `"references "` → `references`
- [x] T002 [P] Rename all trailing-space subdirectories in `.claude/skills/course-outline-to-sidebar/` — use `git mv` to rename `"assets "` → `assets`, `"scripts "` → `scripts`, `"references "` → `references`
- [x] T003 [P] Rename all trailing-space subdirectories in `.claude/skills/docusaurus-chapter-generator/` — use `git mv` to rename `"assets "` → `assets`, `"references "` → `references`
- [x] T004 [P] Rename all trailing-space subdirectories in `.claude/skills/fastapi-chat-endpoint-scaffold/` — use `git mv` to rename `"assets "` → `assets`, `"scripts "` → `scripts`, `"references "` → `references`
- [x] T005 [P] Rename all trailing-space subdirectories in `.claude/skills/hardware-requirements-writer/` — use `git mv` to rename `"assets "` → `assets`, `"references "` → `references`
- [x] T006 [P] Rename all trailing-space subdirectories in `.claude/skills/neon-schema-and-migrations/` — use `git mv` to rename `"assets "` → `assets`, `"scripts "` → `scripts`, `"references "` → `references`, `"migrations "` → `migrations`
- [x] T007 [P] Rename all trailing-space subdirectories in `.claude/skills/personalization-rules-engine/` — use `git mv` to rename `"scripts "` → `scripts`, `"references "` → `references`
- [x] T008 [P] Rename all trailing-space subdirectories in `.claude/skills/qdrant-collection-setup/` — use `git mv` to rename `"scripts "` → `scripts`, `"references "` → `references`
- [x] T009 [P] Rename all trailing-space subdirectories in `.claude/skills/rag-chunking-and-metadata/` — use `git mv` to rename `"scripts "` → `scripts`, `"references "` → `references`
- [x] T010 [P] Rename all trailing-space subdirectories in `.claude/skills/urdu-translation-toggle-ux/` — use `git mv` to rename `"assets "` → `assets`, `"scripts "` → `scripts`, `"references "` → `references`
- [x] T011 Verify no trailing-space directories remain — run `find .claude/skills -maxdepth 3 -type d -name "* "` and confirm zero results

**Checkpoint**: All subdirectory paths are clean. All subsequent tasks can reference paths without trailing spaces.

---

## Phase 2: Foundational (Standard Templates)

**Purpose**: Establish the canonical SKILL.md and agent templates that all standardization tasks will reference. No user story work can begin until templates are defined.

- [x] T012 Create the canonical SKILL.md template file at `specs/012-reusable-automation-skills/skill-template.md` with these exact sections: YAML frontmatter (`name`, `description`), `# Title`, `## Non-Negotiable Rules`, `## Quick Start`, `## Core Implementation Workflow`, `## Acceptance Checklist`. Include inline comments explaining each section's purpose. Reference: plan.md Decision 1.
- [x] T013 [P] Create the canonical agent template file at `specs/012-reusable-automation-skills/agent-template.md` with these exact sections: YAML frontmatter (`name`, `description`, `model: inherit`), body with `Mission:`, `Non-negotiables:`, `Operating procedure (every time):`, `Output requirements:`. Include inline comments. Reference: plan.md Decision 2.

**Checkpoint**: Templates are locked. All standardization tasks use these as the reference.

---

## Phase 3: User Story 1 — Invoke a Skill to Generate a Textbook Chapter (Priority: P1) 🎯 MVP

**Goal**: The `docusaurus-chapter-generator` skill is standardized, has all required sections, includes a practical chapter template asset, and produces valid Docusaurus chapters.

**Independent Test**: Invoke `/docusaurus-chapter-generator Module 1, Week 2, Lesson 1.2` and verify: output has all pedagogical sections, no placeholders, compiles in Docusaurus.

### Implementation for User Story 1

- [x] T014 [US1] Standardize `.claude/skills/docusaurus-chapter-generator/SKILL.md` — read the existing file, compare against template from T012, add missing `## Non-Negotiable Rules` section (include: must follow Docusaurus 3.x MDX, must include all pedagogical sections, no real credentials), add missing `## Quick Start` section (show example invocation with module/week/lesson args), rename any mismatched section headings to match template (e.g., ensure `## Core Implementation Workflow` exists), append `## Acceptance Checklist` with items: `[ ] Chapter has Learning Objectives section`, `[ ] Chapter has Key Terms section`, `[ ] Chapter has Main Explanation section`, `[ ] Chapter has Lab/Exercise section (when applicable)`, `[ ] Chapter has Recap section`, `[ ] Chapter has Quiz section`, `[ ] No template placeholders remain`, `[ ] Frontmatter has sidebar_position and title`. Preserve all existing procedural content — do NOT rewrite the workflow steps.
- [x] T015 [US1] Verify template asset exists at `.claude/skills/docusaurus-chapter-generator/assets/chapter-template.md` — read the file and confirm it contains all required pedagogical sections (Learning Objectives, Key Terms, Introduction, Main Explanation, Deeper Notes, Lab/Exercise, Recap, Quiz). If any section is missing, add it following the pattern in the existing file.

**Checkpoint**: `docusaurus-chapter-generator` is fully compliant with the standard template and has a usable chapter template asset.

---

## Phase 4: User Story 2 — Scaffold a FastAPI Chat Endpoint (Priority: P1)

**Goal**: The `fastapi-chat-endpoint-scaffold` skill is standardized and the remaining 8 skills are also brought into compliance with the template.

**Independent Test**: Invoke `/fastapi-chat-endpoint-scaffold` and verify: output includes Pydantic v2 models, SSE streaming route using `sse_starlette.EventSourceResponse` with dict yields, async database calls, session persistence hooks.

### Implementation for User Story 2

- [x] T016 [US2] Standardize `.claude/skills/fastapi-chat-endpoint-scaffold/SKILL.md` — read existing file, compare against template from T012, add missing `## Quick Start` section (show minimal invocation example), rename `## Scaffold Procedure` to `## Core Implementation Workflow` if needed, append `## Acceptance Checklist` with items: `[ ] Route uses sse_starlette.EventSourceResponse with dict yields`, `[ ] Pydantic v2 models for request/response`, `[ ] Async database calls via asyncpg`, `[ ] Session persistence hooks present`, `[ ] No real secrets or API keys`, `[ ] .env.example uses placeholder values only`. Preserve all existing procedural content.
- [x] T017 [P] [US2] Standardize `.claude/skills/course-outline-to-sidebar/SKILL.md` — add missing `## Non-Negotiable Rules` section (include: must produce valid Docusaurus sidebar config, must create matching folder structure, ESM export format), add missing `## Quick Start` section (show example with sample outline input), ensure `## Core Implementation Workflow` heading exists, append `## Acceptance Checklist`. Preserve existing content.
- [x] T018 [P] [US2] Standardize `.claude/skills/hardware-requirements-writer/SKILL.md` — add missing `## Non-Negotiable Rules` section (include: must cite specific hardware specs, must include cost estimates, no obsolete hardware), add missing `## Quick Start` section, ensure `## Core Implementation Workflow` heading exists, append `## Acceptance Checklist`. Preserve existing content.
- [x] T019 [P] [US2] Standardize `.claude/skills/rag-chunking-and-metadata/SKILL.md` — add missing `## Non-Negotiable Rules` section (include: must preserve frontmatter metadata, chunks must not exceed token limits, must include document_id and section hierarchy in payload), add missing `## Quick Start` section, ensure `## Core Implementation Workflow` heading exists, append `## Acceptance Checklist`. Preserve existing content.
- [x] T020 [P] [US2] Standardize `.claude/skills/qdrant-collection-setup/SKILL.md` — rename `## When to Use This Skill` to standard heading if needed, add missing `## Non-Negotiable Rules` section (include: must validate collection existence before create, must use environment-specific naming, no real API keys), ensure `## Core Implementation Workflow` heading exists, append `## Acceptance Checklist`. Preserve existing content.
- [x] T021 [P] [US2] Standardize `.claude/skills/neon-schema-and-migrations/SKILL.md` — add missing `## Non-Negotiable Rules` section (include: must use asyncpg for queries, must support rollback, no raw credentials), consolidate fragmented workflow sections into `## Core Implementation Workflow`, append `## Acceptance Checklist`. Preserve existing content.
- [x] T022 [P] [US2] Standardize `.claude/skills/chatkit-embed-and-token-flow/SKILL.md` — this skill already has all 4 required sections (confirmed in audit). Only append `## Acceptance Checklist` with items: `[ ] Widget renders in Docusaurus page`, `[ ] Session ID stored in sessionStorage`, `[ ] Token flow connects to FastAPI backend`, `[ ] Selected text mode supported`, `[ ] No real secrets in output`. Preserve all existing content.
- [x] T023 [P] [US2] Standardize `.claude/skills/personalization-rules-engine/SKILL.md` — rename `## Core Principles` to `## Non-Negotiable Rules`, ensure `## Core Implementation Workflow` heading exists, append `## Acceptance Checklist`. Preserve existing content.
- [x] T024 [P] [US2] Standardize `.claude/skills/urdu-translation-toggle-ux/SKILL.md` — add missing `## Quick Start` section (show minimal toggle integration example), ensure `## Core Implementation Workflow` heading exists, append `## Acceptance Checklist`. Preserve existing content.
- [x] T025 [US2] Verify all 10 SKILL.md files have consistent structure — for each of the 10 skills, run a section-heading check: grep for `## Non-Negotiable Rules`, `## Quick Start`, `## Core Implementation Workflow`, `## Acceptance Checklist` and confirm all 4 headings exist in every SKILL.md. Report any failures.

**Checkpoint**: All 10 skills are structurally compliant with the standard template (FR-019, FR-025).

---

## Phase 5: User Story 3 — Delegate Complex Work to Specialized Subagents (Priority: P1)

**Goal**: All 6 subagent definition files are standardized with consistent structure: frontmatter (name, description, model), body (Mission, Non-negotiables, Operating procedure, Output requirements).

**Independent Test**: Present a RAG-related task description and verify the system recommends `rag-pipeline-engineer-agent`; present a Docusaurus build error and verify it recommends `docusaurus-architect-agent`.

### Implementation for User Story 3

- [x] T026 [US3] Standardize `.claude/agents/curriculum-author-agent.md` — read existing file, compare against template from T013, verify frontmatter has `name`, `description`, `model: inherit`. Verify body sections: `Mission:`, `Non-negotiables:`, `Operating procedure (every time):`, `Output requirements:`. This agent is already well-structured — only add any missing section headings. Preserve all existing content.
- [x] T027 [P] [US3] Standardize `.claude/agents/technical-accuracy-reviewer-agent.md` — verify frontmatter, verify/add `Output requirements:` section if missing. Rename `## Anti-handwavy rewrite rules` or similar to include under `Non-negotiables:` if not already present. Preserve all existing content.
- [x] T028 [P] [US3] Standardize `.claude/agents/docusaurus-architect-agent.md` — verify frontmatter, verify/add `Output requirements:` section if missing. Ensure `Non-negotiables:` section exists (may be labeled `Docusaurus-specific rules of thumb` — if so, add `Non-negotiables:` heading above it). Preserve all existing content.
- [x] T029 [P] [US3] Standardize `.claude/agents/rag-pipeline-engineer-agent.md` — verify frontmatter, FIX TRUNCATION: the file is truncated at line 74 mid-sentence ("prints retrieved chunk IDs + metada"). Complete the truncated sentence, add missing `Output requirements:` section, and add any missing quality bar content. Preserve all existing content before the truncation point.
- [x] T030 [P] [US3] Standardize `.claude/agents/chatkit-fastapi-integrator-agent.md` — verify frontmatter, verify/add `Output requirements:` section if missing. Verify `Non-negotiables:` section exists. Preserve all existing content.
- [x] T031 [P] [US3] Standardize `.claude/agents/qa-validation-agent.md` — verify frontmatter, verify/add `Output requirements:` section if missing. Verify `Non-negotiables:` section exists. Preserve all existing content.
- [x] T032 [US3] Verify all 6 agent files have consistent structure — for each agent, confirm frontmatter contains `name`, `description`, `model`. Confirm body contains `Mission:`, `Non-negotiables:`, `Operating procedure`, `Output requirements:`. Report any failures.

**Checkpoint**: All 6 agents are structurally compliant (FR-018, FR-026).

---

## Phase 6: User Story 4 — Discover Available Skills and Agents (Priority: P2)

**Goal**: A routing guide exists at `.claude/ROUTING.md` that maps every skill and agent to its trigger conditions with examples. A beginner can identify the right tool for any task within 60 seconds.

**Independent Test**: Read ROUTING.md and verify: it lists all 10 skills and 6 agents, each with trigger conditions and example invocations; the spec-to-agent routing table covers all spec ranges.

### Implementation for User Story 4

- [x] T033 [US4] Create `.claude/ROUTING.md` with Quick Lookup table — write the `## Quick Lookup: "I need to..."` section containing a 16-row Markdown table mapping common tasks to the correct skill or agent name and type (Skill/Agent). Include all 10 skills and 6 agents. Reference: plan.md Decision 3 for format.
- [x] T034 [US4] Add Skills Reference section to `.claude/ROUTING.md` — append `## Skills Reference (10)` with one subsection per skill. Each entry must include: **Triggers** (2–4 natural language phrases that should invoke this skill), **Example** (exact `/skill-name args` invocation), **Related agent** (which agent handles multi-step versions of this skill's domain). Cover all 10 skills.
- [x] T035 [US4] Add Agents Reference section to `.claude/ROUTING.md` — append `## Agents Reference (6)` with one subsection per agent. Each entry must include: **Delegates when** (2–3 sentence description of task types), **Spec coverage** (which spec numbers this agent primarily serves), **Related skills** (which skills complement this agent's work). Cover all 6 agents.
- [x] T036 [US4] Add Spec-to-Agent Routing table to `.claude/ROUTING.md` — append `## Spec-to-Agent Routing` with a Markdown table mapping spec ranges to primary agents and supporting skills. Must include rows for: 001–004 (Docusaurus), 005–008 (Curriculum), 009 (RAG), 010–011 (ChatKit/FastAPI), Pre-merge (QA). Reference: plan.md Decision 3.
- [x] T037 [US4] Add Maintenance section to `.claude/ROUTING.md` — append `## Maintenance` with instructions: "When adding a new skill or agent, update this file: add a row to Quick Lookup, add a subsection to the appropriate Reference section, update the Spec-to-Agent table if applicable."
- [x] T038 [US4] Cross-validate ROUTING.md against actual files — list all `.claude/skills/*/SKILL.md` and `.claude/agents/*.md` files, then verify every file has a corresponding entry in ROUTING.md. Report any orphans (files without routing entries) or ghosts (routing entries without files).

**Checkpoint**: Routing guide is complete, accurate, and self-documenting (FR-023, SC-003, SC-006).

---

## Phase 7: User Story 5 — Update Sidebar from Course Outline Changes (Priority: P2)

**Goal**: Useful template/asset files exist within skills that need them, covering the most common use cases.

**Independent Test**: Verify that `course-outline-to-sidebar` has an outline template, `neon-schema-and-migrations` has a migration template, and `docusaurus-chapter-generator` has a chapter template.

### Implementation for User Story 5

- [x] T039 [US5] Verify and improve `.claude/skills/course-outline-to-sidebar/assets/outline-template.txt` — read the existing template, ensure it contains a realistic sample course outline (modules, weeks, lessons) that can be used as input to the skill. If it's a placeholder, replace with a concrete 2-module example matching this project's structure.
- [x] T040 [P] [US5] Verify and improve `.claude/skills/course-outline-to-sidebar/assets/frontmatter-template.md` — ensure the frontmatter template includes `sidebar_position`, `title`, and `slug` fields with placeholder values matching Docusaurus 3.x conventions.
- [x] T041 [P] [US5] Verify `.claude/skills/neon-schema-and-migrations/assets/schema.prisma` — read the existing Prisma schema, ensure it includes the core tables (sessions, messages, users) matching the project's Neon Postgres schema from specs 010. If outdated, update to match current conventions.
- [x] T042 [P] [US5] Verify `.claude/skills/neon-schema-and-migrations/migrations/` contains at least one example migration — confirm the existing `20260129135924_help.sql` is a valid example. If it's a placeholder, replace with a realistic initial schema migration.
- [x] T043 [P] [US5] Verify `.claude/skills/fastapi-chat-endpoint-scaffold/assets/.env.example` — read the file and confirm all values are placeholders (no real secrets). Must include: `OPENAI_API_KEY=sk-xxx`, `QDRANT_URL=https://xxx.cloud.qdrant.io`, `QDRANT_API_KEY=xxx`, `DATABASE_URL=postgresql+asyncpg://user:pass@host/db`, `NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.neon.tech/db`.

**Checkpoint**: Template assets are practical and realistic, not empty placeholders (SC-007).

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Security scan, consistency verification, and final validation.

### Security Scan (FR-020, SC-005)

- [x] T044 Run secrets scan across all skill files — grep for patterns `API_KEY=(?!.*xxx)`, `SECRET=`, `password=(?!.*pass)`, `Bearer `, `sk-(?!xxx)` in all `.claude/skills/*/SKILL.md` and all files in `assets/`, `scripts/`, `references/` subdirectories. Report any real credentials found. Zero findings required to pass.
- [x] T045 [P] Run secrets scan across all agent files — grep for the same patterns in all `.claude/agents/*.md` files. Zero findings required.

### Structural Consistency (FR-025, SC-004)

- [x] T046 Final consistency audit — for each of the 10 SKILL.md files, verify: (1) YAML frontmatter contains `name` matching directory name, (2) YAML frontmatter contains `description`, (3) file contains `## Non-Negotiable Rules`, (4) file contains `## Quick Start`, (5) file contains `## Core Implementation Workflow`, (6) file contains `## Acceptance Checklist`. Produce a 10-row pass/fail table. All must pass.
- [x] T047 [P] Final agent consistency audit — for each of the 6 agent `.md` files, verify: (1) YAML frontmatter contains `name`, `description`, `model`, (2) body contains `Mission:`, (3) body contains `Non-negotiables:`, (4) body contains `Operating procedure`, (5) body contains `Output requirements:`. Produce a 6-row pass/fail table. All must pass.

### Discoverability Validation (FR-024, SC-001, SC-002)

- [x] T048 Verify skill discoverability — list all skill directories with `ls .claude/skills/` and confirm exactly 10 skills are present. Verify each SKILL.md has valid YAML frontmatter by checking the `---` delimiters parse correctly.
- [x] T049 [P] Verify agent discoverability — list all agent files with `ls .claude/agents/` and confirm exactly 6 `.md` files are present. Verify each has valid YAML frontmatter.

### Demo Run (SC-007)

- [ ] T050 Demo: invoke 3 skills and verify output — run these 3 skills in sequence: (1) `/docusaurus-chapter-generator Module 1, Week 2, Lesson 1.2 — Introduction to Robot Operating System` and verify chapter markdown is produced with all required sections, (2) `/fastapi-chat-endpoint-scaffold` and verify scaffold includes Pydantic v2 models + SSE route, (3) `/course-outline-to-sidebar` with the outline template from assets and verify sidebar config is produced. Document results.
- [ ] T051 Demo: delegate to 2 agents and verify routing — (1) submit task "Review chapter 1.1 for technical accuracy in robotics definitions" and confirm it routes to `technical-accuracy-reviewer-agent`, (2) submit task "Validate Docusaurus build passes with no broken links" and confirm it routes to `qa-validation-agent`. Document results.

**Checkpoint**: All validation passes. RI pack is complete, documented, and usable.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately. **BLOCKS all other phases.**
- **Phase 2 (Templates)**: Depends on Phase 1. **BLOCKS Phases 3–7.**
- **Phase 3 (US1 — Chapter Skill)**: Depends on Phase 2. Can run in parallel with Phases 4–5.
- **Phase 4 (US2 — FastAPI + All Skills)**: Depends on Phase 2. Can run in parallel with Phases 3, 5.
- **Phase 5 (US3 — Agents)**: Depends on Phase 2. Can run in parallel with Phases 3, 4.
- **Phase 6 (US4 — Routing Guide)**: Depends on Phases 3, 4, 5 (needs final skill/agent content).
- **Phase 7 (US5 — Template Assets)**: Depends on Phase 1. Can run in parallel with Phases 3–5.
- **Phase 8 (Polish)**: Depends on ALL previous phases.

### User Story Dependencies

- **US1 (Chapter Skill)**: Independent after Phase 2
- **US2 (FastAPI + All Skills)**: Independent after Phase 2 — largest phase, 10 tasks
- **US3 (Agents)**: Independent after Phase 2
- **US4 (Routing Guide)**: Depends on US1 + US2 + US3 (needs final content to reference)
- **US5 (Template Assets)**: Independent after Phase 1

### Parallel Opportunities

```text
After Phase 1 completes:
  Phase 2 (Templates)
    ↓
  ┌─────────────────────────────────────────────────┐
  │  Phase 3 (US1)  ║  Phase 4 (US2)  ║  Phase 5 (US3)  ║  Phase 7 (US5)  │
  │  T014–T015      ║  T016–T025      ║  T026–T032      ║  T039–T043      │
  └─────────────────────────────────────────────────┘
    ↓
  Phase 6 (US4 — Routing Guide): T033–T038
    ↓
  Phase 8 (Polish): T044–T051
```

Within Phase 4, tasks T017–T024 are all parallelizable (different SKILL.md files, no dependencies).
Within Phase 5, tasks T027–T031 are all parallelizable (different agent files, no dependencies).

---

## Implementation Strategy

### MVP Scope

**MVP = Phase 1 + Phase 2 + Phase 3 (US1)**

After completing just these phases, you have:
- Clean directory structure (no trailing spaces)
- A locked template for skills and agents
- One fully compliant skill (`docusaurus-chapter-generator`) with practical chapter template

This is a useful, testable increment that proves the template works.

### Incremental Delivery

1. **Increment 1**: MVP (Phases 1–3) — 15 tasks
2. **Increment 2**: All skills standardized (Phase 4) — 10 tasks
3. **Increment 3**: All agents standardized (Phase 5) — 7 tasks
4. **Increment 4**: Routing guide + template assets (Phases 6–7) — 11 tasks
5. **Increment 5**: Validation + demo (Phase 8) — 8 tasks

### Summary

| Metric | Count |
|--------|-------|
| Total tasks | 51 |
| Phase 1 (Setup) | 11 |
| Phase 2 (Templates) | 2 |
| Phase 3 (US1) | 2 |
| Phase 4 (US2) | 10 |
| Phase 5 (US3) | 7 |
| Phase 6 (US4) | 6 |
| Phase 7 (US5) | 5 |
| Phase 8 (Polish) | 8 |
| Parallelizable tasks | 34 (67%) |
| User stories | 5 |
