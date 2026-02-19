---
id: "0001"
title: "Plan Reusable Intelligence Pack"
stage: plan
date: 2026-02-17
surface: agent
model: claude-sonnet-4-6
feature: 012-reusable-automation-skills
branch: 012-reusable-automation-skills
user: architect
command: /sp.plan
labels: ["skills", "agents", "routing", "standardization"]
links:
  spec: specs/012-reusable-automation-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-reusable-automation-skills/plan.md
  - specs/012-reusable-automation-skills/research.md
tests:
  - null
---

## Prompt

Plan the Reusable Intelligence implementation: define the directory structure and naming for all skills, define the standard format for each SKILL.md (frontmatter, inputs/outputs, procedure, acceptance checklist, examples), define the subagent configuration strategy (project-level agents created via Claude Code /agents, tool permissions, and how agents reference skills), define a routing policy document that maps tasks/specs to agents+skills (e.g., Spec 005-008 → CurriculumAuthor + TechnicalReviewer; Spec 001-004 → DocusaurusArchitect; Spec 009 → RAGPipelineEngineer; Spec 010-011 → ChatKitFastAPIIntegrator; QAValidation before merges), and include a validation plan (how to verify Claude can discover skills and agents, and a short "demo run" checklist showing each agent/skill used once). Acceptance: the plan is actionable, repo-consistent, and produces reusable assets that reduce repeated work without changing core product behavior.

## Response snapshot

Created implementation plan with 5 work areas: (1) Fix directory names with trailing spaces via git mv, (2) Standardize 10 SKILL.md files to 5-section template (Title, Non-Negotiable Rules, Quick Start, Core Workflow, Acceptance Checklist), (3) Standardize 6 agent files to match official Claude Code conventions (name, description, model frontmatter + Mission/Non-negotiables/Procedure/Output body), (4) Create routing guide at .claude/ROUTING.md with task→skill/agent lookup table and spec-to-agent routing, (5) Validation and demo run checklist. Research confirmed official Claude Code conventions for skills and agents. All constitution gates pass. No runtime code changes.

## Outcome

- Impact: Plan defines 5 work areas covering 10 skills + 6 agents + 1 routing guide
- Tests: Validation checklist defined in Area 5 (file presence, frontmatter, sections, secrets scan, discoverability)
- Files: plan.md, research.md created
- Next prompts: /sp.tasks to generate task breakdown, then /sp.implement
- Reflection: All skills/agents exist in draft form; work is standardization + documentation, not creation from scratch

## Evaluation notes (flywheel)

- Failure modes observed: Subagent had read errors with pages parameter; directory names with trailing spaces caused glob issues
- Graders run and results (PASS/FAIL): N/A (planning phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.tasks to decompose into implementable tasks
