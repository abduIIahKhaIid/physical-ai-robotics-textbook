---
id: "0004"
title: "Implement Reusable Intelligence Pack"
stage: green
date: 2026-02-17
surface: agent
model: claude-sonnet-4-6
feature: 012-reusable-automation-skills
branch: 012-reusable-automation-skills
user: abduIIahKhaIid
command: /sp.implement
labels: ["skills", "agents", "routing", "standardization", "templates"]
links:
  spec: specs/012-reusable-automation-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-reusable-automation-skills/skill-template.md
  - specs/012-reusable-automation-skills/agent-template.md
  - .claude/ROUTING.md
  - .claude/skills/docusaurus-chapter-generator/SKILL.md
  - .claude/skills/fastapi-chat-endpoint-scaffold/SKILL.md
  - .claude/skills/course-outline-to-sidebar/SKILL.md
  - .claude/skills/hardware-requirements-writer/SKILL.md
  - .claude/skills/rag-chunking-and-metadata/SKILL.md
  - .claude/skills/qdrant-collection-setup/SKILL.md
  - .claude/skills/neon-schema-and-migrations/SKILL.md
  - .claude/skills/chatkit-embed-and-token-flow/SKILL.md
  - .claude/skills/personalization-rules-engine/SKILL.md
  - .claude/skills/urdu-translation-toggle-ux/SKILL.md
  - .claude/agents/curriculum-author-agent.md
  - .claude/agents/technical-accuracy-reviewer-agent.md
  - .claude/agents/docusaurus-architect-agent.md
  - .claude/agents/rag-pipeline-engineer-agent.md
  - .claude/agents/chatkit-fastapi-integrator-agent.md
  - .claude/agents/qa-validation-agent.md
  - .claude/skills/course-outline-to-sidebar/assets/outline-template.txt
  - .claude/skills/neon-schema-and-migrations/migrations/20260129135924_help.sql
  - .claude/skills/fastapi-chat-endpoint-scaffold/assets/.env.example
  - .claude/skills/docusaurus-chapter-generator/assets/chapter-template.md
  - specs/012-reusable-automation-skills/tasks.md
tests:
  - "T044: Secrets scan on skills — PASS (zero real credentials)"
  - "T045: Secrets scan on agents — PASS (zero findings)"
  - "T046: 10/10 skills structural compliance — PASS"
  - "T047: 6/6 agents structural compliance — PASS"
  - "T048: 10 skills discoverable — PASS"
  - "T049: 6 agents discoverable — PASS"
  - "T038: ROUTING.md cross-validation — PASS"
---

## Prompt

Implement Spec 012 end-to-end: create the full Reusable Intelligence pack by adding the agreed set of skills as .claude/skills/<name>/SKILL.md files using a consistent format (frontmatter, purpose, inputs/outputs, step-by-step procedure, acceptance checklist, example usage), create the agreed set of project-level subagents via Claude Code /agents with clear responsibilities and appropriate tool permissions, and add a routing policy document that specifies when to invoke each agent and skill during Specs 001-011 and the bonus specs; include lightweight templates where helpful (chapter template, sidebar skeleton, migration template) and validate that Claude Code can discover and use the skills and agents. Acceptance: skills and subagents are discoverable in Claude Code, documentation is clear for a beginner, at least one demo workflow successfully uses multiple skills/agents, and the RI pack reduces repeated work without breaking the existing project.

## Response snapshot

Implemented Spec 012 across 8 phases (51 tasks, 49 completed, 2 deferred):

**Phase 1 — Setup**: Fixed 27 trailing-space directories across all 10 skills using `git mv`.

**Phase 2 — Templates**: Created canonical `skill-template.md` (5-section format) and `agent-template.md` (4-section format) as references.

**Phase 3-5 — Standardization**: Standardized all 10 SKILL.md files to consistent 5-section format (Title, Non-Negotiable Rules, Quick Start, Core Implementation Workflow, Acceptance Checklist). Standardized all 6 agent files to consistent 4-section format (Mission, Non-negotiables, Operating procedure, Output requirements). Fixed truncation in rag-pipeline-engineer-agent.md.

**Phase 6 — Routing Guide**: Created `.claude/ROUTING.md` with Quick Lookup table (16 rows), Skills Reference (10 entries), Agents Reference (6 entries), Spec-to-Agent Routing table, and Maintenance section.

**Phase 7 — Template Assets**: Improved outline-template.txt (concrete course example), migration template (real schema SQL), .env.example (obviously fake credentials), chapter-template.md (added Key Terms, Recap, Quiz sections).

**Phase 8 — Validation**: All 7 validation checks passed. T050-T051 (demo runs) deferred to interactive session.

## Outcome

- Impact: Full Reusable Intelligence pack implemented — 10 skills, 6 agents, routing guide, and templates all standardized and discoverable
- Tests: 7/7 validation checks PASS (secrets scan, structural compliance, discoverability, cross-validation)
- Files: 24 files created or modified across .claude/skills/, .claude/agents/, .claude/ROUTING.md, and specs/
- Next prompts: Run demo tasks T050-T051 in interactive session; commit and create PR
- Reflection: Background subagents were blocked by permission model — sequential direct edits were more reliable

## Evaluation notes (flywheel)

- Failure modes observed: Subagent Write/Edit/Bash permissions denied; Grep tool inconsistent with markdown heading patterns
- Graders run and results (PASS/FAIL): All 7 automated validation tasks PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Test subagent permissions with explicit tool allowlists before parallelizing
