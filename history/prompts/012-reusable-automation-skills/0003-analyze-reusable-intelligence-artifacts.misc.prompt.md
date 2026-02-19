---
id: "0003"
title: "Analyze Reusable Intelligence Artifacts"
stage: misc
date: 2026-02-17
surface: agent
model: claude-sonnet-4-6
feature: 012-reusable-automation-skills
branch: 012-reusable-automation-skills
user: architect
command: /sp.analyze
labels: ["analysis", "cross-artifact", "validation", "skills", "agents"]
links:
  spec: specs/012-reusable-automation-skills/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-reusable-automation-skills/spec.md
  - specs/012-reusable-automation-skills/plan.md
  - specs/012-reusable-automation-skills/tasks.md
  - specs/012-reusable-automation-skills/research.md
tests:
  - null
---

## Prompt

Analyze Spec 012 artifacts (spec/plan/tasks) for usability and completeness: verify every listed skill exists with correct naming and consistent frontmatter, verify each skill has clear inputs/outputs and an actionable procedure and acceptance checklist, verify subagents cover the required responsibilities without overlap gaps, verify routing policy is concrete (maps common tasks/specs to the right agent/skill), verify nothing introduces secrets or environment-specific assumptions, and verify the RI pack aligns with the current repo structure and the hackathon requirements. Acceptance: analysis outputs Critical/Major/Minor findings, identifies any missing skills/agents or unclear invocation rules, and provides a clear "Ready to implement: Yes/No" decision based on whether the RI pack is actually reusable and discoverable.

## Response snapshot

Produced a 9-finding analysis report across spec.md, plan.md, tasks.md, and constitution.md. 0 CRITICAL, 2 HIGH (spec-plan inconsistencies in FR-021 auto-invocation policy and FR-019 section count mismatch), 5 MEDIUM (constitution test-first tension, FR-026 tools clause conflict, edge case gaps, FR duplication, subjective SC-006), 2 LOW. Coverage: 27/27 FRs mapped to tasks (100%), 7/7 SCs mapped (100%), 0/4 edge cases covered. Filesystem audit confirmed: rag-pipeline-engineer-agent truncated at L75, 0/10 skills have Acceptance Checklist, 5/6 agents lack Non-negotiables section, ROUTING.md does not exist. Decision: Ready to implement YES — after resolving 2 HIGH findings (5-minute spec edits).

## Outcome

- Impact: Cross-artifact analysis identified 2 HIGH alignment issues blocking clean implementation; all other findings are non-blocking
- Tests: N/A (read-only analysis phase)
- Files: No files modified (read-only analysis)
- Next prompts: Fix I1 (FR-021) and I2 (FR-019) in spec.md, then /sp.implement
- Reflection: 100% FR and SC coverage is strong; the 2 HIGHs are spec-plan drift from iterative refinement, not design flaws

## Evaluation notes (flywheel)

- Failure modes observed: Filesystem audit for trailing-space directories was inconclusive via `ls` (spaces hidden); recommend `find -name "* "` verification before Phase 1
- Graders run and results (PASS/FAIL): N/A (analysis phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Fix I1+I2 in spec.md, then run /sp.implement starting with Phase 1 MVP
