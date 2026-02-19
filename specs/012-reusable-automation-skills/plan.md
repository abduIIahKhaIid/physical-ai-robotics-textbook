# Implementation Plan: Reusable Automation Skills

**Branch**: `012-reusable-automation-skills` | **Date**: 2026-02-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/012-reusable-automation-skills/spec.md`

## Summary

Validate, standardize, and document the existing 10 skills and 6 subagents as a cohesive "Reusable Intelligence Pack." All skills and agents already exist in draft form. This plan covers: fixing structural issues (directory names with spaces), standardizing SKILL.md and agent `.md` formats to match official Claude Code conventions, creating a routing guide, and validating discoverability. No new runtime code or product behavior changes.

## Technical Context

**Language/Version**: Markdown (skills/agents are Markdown definition files, not executable code)
**Primary Dependencies**: Claude Code v2.1+ (skill/agent discovery), `.claude/` directory conventions
**Storage**: `.claude/skills/` (skills), `.claude/agents/` (agents)
**Testing**: Manual validation (directory listing, Docusaurus build, skill invocation smoke tests)
**Target Platform**: Claude Code CLI (local development environment)
**Project Type**: Configuration/documentation (no source code changes)
**Performance Goals**: N/A (definition files, not runtime code)
**Constraints**: No secrets in any file; follow Claude Code frontmatter conventions
**Scale/Scope**: 10 skills + 6 agents + 1 routing guide = 17 files to validate/standardize

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Documentation-First | PASS | This feature IS documentation — skills and agents are instructional files |
| II. Selected Text Only Mode | N/A | No RAG behavior changes |
| III. Test-First | PASS | Validation checklist defined in spec; no runtime code to TDD |
| IV. Secure Architecture | PASS | FR-020 requires zero secrets; will be verified |
| V. Scalable Cloud Infrastructure | N/A | No runtime services affected |
| VI. Modular Component Design | PASS | Each skill/agent is independently deployable |

No violations. Gate passes.

## Project Structure

### Documentation (this feature)

```text
specs/012-reusable-automation-skills/
├── plan.md              # This file
├── research.md          # Phase 0: conventions research
├── tasks.md             # Phase 2 output (/sp.tasks)
└── checklists/
    └── requirements.md  # Spec quality checklist
```

### Source Code (repository root)

```text
.claude/
├── agents/                              # 6 agent definitions
│   ├── chatkit-fastapi-integrator-agent.md
│   ├── curriculum-author-agent.md
│   ├── docusaurus-architect-agent.md
│   ├── qa-validation-agent.md
│   ├── rag-pipeline-engineer-agent.md
│   └── technical-accuracy-reviewer-agent.md
├── skills/                              # 10 skill directories
│   ├── chatkit-embed-and-token-flow/
│   │   ├── SKILL.md
│   │   ├── assets/                      # FIX: currently "assets /" with trailing space
│   │   ├── scripts/                     # FIX: currently "scripts /" with trailing space
│   │   └── references/                  # FIX: currently "references /" with trailing space
│   ├── course-outline-to-sidebar/
│   │   ├── SKILL.md
│   │   ├── assets/
│   │   ├── scripts/
│   │   └── references/
│   ├── docusaurus-chapter-generator/
│   │   ├── SKILL.md
│   │   ├── assets/
│   │   └── references/
│   ├── fastapi-chat-endpoint-scaffold/
│   │   ├── SKILL.md
│   │   ├── assets/
│   │   ├── scripts/
│   │   └── references/
│   ├── hardware-requirements-writer/
│   │   ├── SKILL.md
│   │   ├── assets/
│   │   └── references/
│   ├── neon-schema-and-migrations/
│   │   ├── SKILL.md
│   │   ├── assets/
│   │   ├── migrations/
│   │   ├── scripts/
│   │   └── references/
│   ├── personalization-rules-engine/
│   │   ├── SKILL.md
│   │   ├── assets/
│   │   ├── scripts/
│   │   └── references/
│   ├── qdrant-collection-setup/
│   │   ├── SKILL.md
│   │   ├── scripts/
│   │   └── references/
│   ├── rag-chunking-and-metadata/
│   │   ├── SKILL.md
│   │   ├── scripts/
│   │   └── references/
│   └── urdu-translation-toggle-ux/
│       ├── SKILL.md
│       ├── assets/
│       ├── scripts/
│       └── references/
└── ROUTING.md                           # NEW: routing guide
```

**Structure Decision**: No new directories created. All work is within the existing `.claude/` directory tree. The only new file is `.claude/ROUTING.md`. Subdirectory names with trailing spaces must be renamed to remove spaces.

## Phase 0: Research

### Research Findings

#### Decision 1: SKILL.md Standard Template Structure

**Decision**: Standardize on 5 sections matching Claude Code conventions and existing project patterns.

**Rationale**: Official Claude Code docs show skills as Markdown with YAML frontmatter + free-form instructions. The project's existing skills already use a pattern of: Overview/intro → Rules → Quick Start → Workflow. Formalizing this as a standard template ensures consistency.

**Standard SKILL.md Template**:

```yaml
---
name: <skill-name>                    # Required: lowercase-kebab-case, matches directory name
description: <one-paragraph summary>  # Required: loaded into system prompt for discovery
---

# <Skill Title>

<One-sentence purpose statement.>

## Non-Negotiable Rules

- <Hard constraint 1>
- <Hard constraint 2>
- <No secrets, real credentials, or API keys in outputs>

## Quick Start

<Minimal copy-paste example showing the most common invocation.>

## Core Implementation Workflow

<Step-by-step procedure the assistant follows when this skill is invoked.>

1. <Step 1>
2. <Step 2>
3. <Step 3>

## Acceptance Checklist

- [ ] <Output meets criterion 1>
- [ ] <Output meets criterion 2>
- [ ] <No template placeholders remain>
```

**Alternatives considered**:
- Matching official docs examples (minimal, no prescribed sections) — rejected because it allows too much drift between skills
- Adding `inputs:` and `outputs:` YAML fields — rejected because Claude Code doesn't use these natively; they'd be ignored

#### Decision 2: Agent Definition Standard Format

**Decision**: Standardize on official Claude Code agent frontmatter with `name`, `description`, `model`, and optionally `tools`.

**Rationale**: Official docs require only `name` and `description`. Adding `model: inherit` explicitly documents the inheritance. The body follows the pattern: Mission → Non-negotiables → Operating procedure → Output requirements.

**Standard Agent Template**:

```yaml
---
name: <agent-name>                    # Required: lowercase-kebab-case
description: "<trigger description>"  # Required: Claude reads this to decide delegation
model: inherit                        # Explicit: inherits parent model
---

You are <AgentName>.

Mission:
<One-sentence mission statement.>

Non-negotiables:
- <Hard rule 1>
- <Hard rule 2>

Operating procedure (every time):
1) <Step 1>
2) <Step 2>
3) <Step 3>

Output requirements:
<What the agent must produce when done.>
```

**Alternatives considered**:
- Adding `tools:` restrictions to each agent — rejected for now; agents need broad tool access for their complex multi-step tasks. Can be tightened later per-agent as needed.
- Adding `maxTurns:` — deferred; let default behavior govern until we observe issues.

#### Decision 3: Routing Guide Format and Location

**Decision**: Place routing guide at `.claude/ROUTING.md`. Format as a task→skill/agent lookup table with trigger patterns and examples.

**Rationale**: Files in `.claude/` are auto-loaded by Claude Code into the system context. Placing the routing guide here ensures the assistant can reference it without manual file reads. A simple lookup-table format is scannable by both humans and the AI.

**Structure**:

```markdown
# Routing Guide: Skills & Agents

## Quick Lookup: "I need to..."

| Task | Use | Type |
|------|-----|------|
| Create a new chapter/lesson | docusaurus-chapter-generator | Skill |
| Update sidebar from outline | course-outline-to-sidebar | Skill |
| ... | ... | ... |

## Skills Reference (10)

### docusaurus-chapter-generator
- **Triggers**: "create a chapter", "add a lesson", "generate chapter content"
- **Example**: `/docusaurus-chapter-generator Module 3, Week 9, Lesson 3.1`
- **Related agent**: curriculum-author-agent (for multi-chapter work)

[repeat for each skill]

## Agents Reference (6)

### curriculum-author-agent
- **Delegates when**: Multi-chapter authoring, curriculum alignment verification
- **Spec coverage**: 005-module-1-chapters through 008-humanoid-vla-concepts
- **Related skills**: docusaurus-chapter-generator, hardware-requirements-writer

[repeat for each agent]

## Spec-to-Agent Routing

| Spec Range | Primary Agent | Supporting Skills |
|------------|---------------|-------------------|
| 001-004    | docusaurus-architect-agent | course-outline-to-sidebar |
| 005-008    | curriculum-author-agent + technical-accuracy-reviewer-agent | docusaurus-chapter-generator, hardware-requirements-writer |
| 009        | rag-pipeline-engineer-agent | rag-chunking-and-metadata, qdrant-collection-setup |
| 010-011    | chatkit-fastapi-integrator-agent | fastapi-chat-endpoint-scaffold, chatkit-embed-and-token-flow, neon-schema-and-migrations |
| Pre-merge  | qa-validation-agent | (all validation skills as needed) |
```

**Alternatives considered**:
- Placing in `docs/` — rejected; not auto-loaded by Claude Code
- Embedding routing rules in CLAUDE.md — rejected; CLAUDE.md is already long (200+ lines)

#### Decision 4: Directory Name Fix Strategy

**Decision**: Rename directories with trailing spaces (e.g., `assets /` → `assets/`) using `git mv` to preserve history.

**Rationale**: The space in directory names (e.g., `.claude/skills/chatkit-embed-and-token-flow/assets /`) is a bug that can cause path resolution issues on some platforms. Using `git mv` preserves file history.

**Affected skills** (directories with trailing space before `/`):
- chatkit-embed-and-token-flow: `assets /`, `scripts /`, `references /`
- course-outline-to-sidebar: `assets /`, `scripts /`, `references /`
- docusaurus-chapter-generator: `assets /`, `references /`
- fastapi-chat-endpoint-scaffold: `assets /`, `scripts /`, `references /`
- hardware-requirements-writer: `assets /`, `references /`
- neon-schema-and-migrations: `assets /`, `scripts /`, `references /`, `migrations /`
- personalization-rules-engine: `scripts /`, `references /`
- qdrant-collection-setup: `scripts /`, `references /`
- rag-chunking-and-metadata: `scripts /`, `references /`
- urdu-translation-toggle-ux: `assets /`, `scripts /`, `references /`

**Note**: `personalization-rules-engine/assets/` subdirectories (`config_templates/`, `lab_variants/`) appear to NOT have the space issue.

#### Decision 5: Invocation Policy

**Decision**: All 10 skills are manual-only (user must invoke or request). All 6 agents are auto-delegated by the assistant when task matches, subject to user approval of the Task tool.

**Rationale**: Skills produce artifacts (files, scaffolds) that the user should explicitly request. Agents handle complex multi-step tasks that the assistant naturally delegates. Both follow FR-021 and FR-022 from the spec. No `disable-model-invocation: true` frontmatter needed because skills are already loaded as slash commands (user invokes via `/skill-name`).

**Clarification on auto-invocation**: Claude Code can auto-invoke skills when it detects the description matches the user's request. This is the default behavior and is acceptable — the spec's FR-021 "manually invocable" means available for manual invocation, not restricted to manual-only. We will NOT add `disable-model-invocation: true` unless the user requests it.

## Phase 1: Implementation Design

### Work Breakdown

The implementation consists of 5 sequential work areas:

#### Area 1: Fix Directory Names (prerequisite for all other work)

Rename all subdirectories with trailing spaces. This must be done first because subsequent edits to files within those directories need correct paths.

**Approach**: Use `git mv` for each affected directory. Process all 10 skills in a single pass.

**Risk**: File moves in git could cause merge conflicts if other branches touch the same files. Mitigation: this branch is isolated, and the files are skill definitions not touched by other feature branches.

#### Area 2: Standardize SKILL.md Files (10 files)

For each of the 10 skills, ensure the SKILL.md matches the standard template:

1. **Frontmatter**: Must have `name` and `description` fields. Remove any extra fields not in the official Claude Code spec.
2. **Sections**: Ensure all 5 standard sections exist: title, Non-Negotiable Rules, Quick Start, Core Implementation Workflow, Acceptance Checklist.
3. **Content**: Verify no real secrets or credentials. Verify output guidance matches current repo conventions (Pydantic v2, ESM, Docusaurus 3.x MDX, async patterns).
4. **No content rewrite**: Do NOT rewrite the procedural content of skills unless it contradicts current conventions. Only add missing sections, fix formatting, and ensure structural compliance.

**Per-skill compliance delta** (based on audit):

| Skill | Has Frontmatter | Missing Sections | Needs Fixes |
|-------|----------------|-------------------|-------------|
| docusaurus-chapter-generator | name, description | Acceptance Checklist | Add checklist |
| course-outline-to-sidebar | name, description | Acceptance Checklist | Add checklist |
| hardware-requirements-writer | name, description | Acceptance Checklist | Add checklist |
| rag-chunking-and-metadata | name, description | Acceptance Checklist | Add checklist |
| qdrant-collection-setup | name, description | Acceptance Checklist | Add checklist |
| neon-schema-and-migrations | name, description | Acceptance Checklist | Add checklist |
| fastapi-chat-endpoint-scaffold | name, description | Acceptance Checklist | Add checklist |
| chatkit-embed-and-token-flow | name, description | Acceptance Checklist | Add checklist |
| personalization-rules-engine | name, description | Acceptance Checklist | Add checklist |
| urdu-translation-toggle-ux | name, description | Acceptance Checklist | Add checklist |

All skills have correct frontmatter. All need an "Acceptance Checklist" section appended. Some may need section headings renamed to match the standard (e.g., "Core Features" → "Core Implementation Workflow" if currently named differently).

#### Area 3: Standardize Agent Files (6 files)

For each of the 6 agents, ensure the `.md` matches the standard template:

1. **Frontmatter**: Must have `name`, `description`, `model`. Verify `model: inherit` is explicit.
2. **Body structure**: Verify Mission, Non-negotiables, Operating procedure, Output requirements sections exist.
3. **Content**: Verify no real secrets or credentials.

**Per-agent compliance delta**:

| Agent | Has name | Has description | Has model | Missing Sections |
|-------|----------|----------------|-----------|-----------------|
| curriculum-author-agent | Yes | Yes | Yes (inherit) | None — well-structured |
| technical-accuracy-reviewer-agent | Yes | Yes | Yes (inherit) | Verify output requirements |
| docusaurus-architect-agent | Yes | Yes | Yes (inherit) | Verify output requirements |
| rag-pipeline-engineer-agent | Yes | Yes | Yes (inherit) | Verify output requirements |
| chatkit-fastapi-integrator-agent | Yes | Yes | Yes (inherit) | Verify output requirements |
| qa-validation-agent | Yes | Yes | Yes (inherit) | Verify output requirements |

Agents are closer to compliance. Main work is verifying each has all 4 body sections.

#### Area 4: Create Routing Guide

Create `.claude/ROUTING.md` following the structure defined in Research Decision 3:
- Quick Lookup table (16 rows: 10 skills + 6 agents)
- Detailed Skills Reference (10 entries)
- Detailed Agents Reference (6 entries)
- Spec-to-Agent routing table

#### Area 5: Validation & Demo Run

Create a validation checklist that verifies:

1. **File presence**: All 10 skills and 6 agents exist at correct paths
2. **No trailing-space directories**: Verify rename completed
3. **Frontmatter compliance**: All files have required fields
4. **Section compliance**: All files have required sections
5. **No secrets scan**: grep for common secret patterns (API_KEY=, SECRET=, password=, Bearer, sk-)
6. **Discoverability**: Start Claude Code and verify skills appear in `/` autocomplete and agents appear in Task tool
7. **Routing guide accuracy**: Every skill and agent in the repo has a corresponding entry in ROUTING.md

**Demo run checklist** (invoke each skill/agent once to verify it loads):

| Item | Invocation | Expected |
|------|-----------|----------|
| docusaurus-chapter-generator | `/docusaurus-chapter-generator Module 1, Week 1, Lesson 1.1` | Chapter markdown produced |
| course-outline-to-sidebar | `/course-outline-to-sidebar` with sample outline | Sidebar config produced |
| hardware-requirements-writer | `/hardware-requirements-writer` for ROS 2 course | Hardware doc produced |
| rag-chunking-and-metadata | `/rag-chunking-and-metadata` for a sample chapter | Chunking strategy output |
| qdrant-collection-setup | `/qdrant-collection-setup` new collection | Setup instructions output |
| neon-schema-and-migrations | `/neon-schema-and-migrations` init | Schema SQL output |
| fastapi-chat-endpoint-scaffold | `/fastapi-chat-endpoint-scaffold` | Endpoint scaffold produced |
| chatkit-embed-and-token-flow | `/chatkit-embed-and-token-flow` | Widget embed code produced |
| personalization-rules-engine | `/personalization-rules-engine` beginner profile | Rules config output |
| urdu-translation-toggle-ux | `/urdu-translation-toggle-ux` for a chapter | Translation toggle code |
| curriculum-author-agent | Task: "Write chapter for Module 1 Week 2" | Agent produces chapter |
| technical-accuracy-reviewer-agent | Task: "Review chapter 1.1 for accuracy" | Agent produces review |
| docusaurus-architect-agent | Task: "Fix sidebar broken link" | Agent investigates and fixes |
| rag-pipeline-engineer-agent | Task: "Design chunking for Module 3" | Agent produces design |
| chatkit-fastapi-integrator-agent | Task: "Wire ChatKit to new endpoint" | Agent produces integration |
| qa-validation-agent | Task: "Validate build readiness" | Agent runs checks |

## Complexity Tracking

No complexity violations. All work is within existing structures — no new abstractions, no new runtime dependencies.

## Post-Design Constitution Re-Check

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Documentation-First | PASS | All changes are documentation/configuration |
| II. Selected Text Only Mode | N/A | No RAG behavior changes |
| III. Test-First | PASS | Validation checklist serves as test plan |
| IV. Secure Architecture | PASS | FR-020 secrets scan included in validation |
| V. Scalable Cloud Infrastructure | N/A | No runtime changes |
| VI. Modular Component Design | PASS | Each skill/agent remains independent |

## Implementation Order

```
Area 1: Fix Directory Names
  ↓
Area 2: Standardize SKILL.md Files (can be parallelized across skills)
  ↓
Area 3: Standardize Agent Files (can be parallelized across agents)
  ↓
Area 4: Create Routing Guide (depends on Areas 2-3 for final content)
  ↓
Area 5: Validation & Demo Run (depends on all above)
```

Areas 2 and 3 can run in parallel after Area 1 completes.

## Risks

1. **Directory rename breaks git history** — Low risk. Using `git mv` preserves history. Worst case: files show as deleted+added in some git UIs.
2. **Skill content drift** — Medium risk. Standardizing sections may inadvertently lose nuanced instructions if we're not careful to preserve existing content. Mitigation: only add missing sections; do not rewrite existing content.
3. **Routing guide staleness** — Medium risk over time. As new specs are added, the routing guide needs updating. Mitigation: add a "Maintenance" section to ROUTING.md reminding contributors to update it when adding skills/agents.
