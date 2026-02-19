# Research: Reusable Automation Skills

**Date**: 2026-02-17 | **Branch**: 012-reusable-automation-skills

## 1. Claude Code Skills Convention (Official)

**Source**: code.claude.com/docs/en/skills.md

- **Discovery**: Auto-discovered from `.claude/skills/` directories. Descriptions loaded into system prompt up to 2% of context budget.
- **Format**: YAML frontmatter (`---`) + Markdown body. Only `name` and `description` are recommended.
- **Optional frontmatter fields**: `argument-hint`, `disable-model-invocation`, `user-invocable`, `allowed-tools`, `model`, `context`, `agent`, `hooks`
- **String substitutions**: `$ARGUMENTS`, `$ARGUMENTS[N]`, `${CLAUDE_SESSION_ID}`, `` !`command` ``
- **Invocation**: By default, both user-invocable (via `/name`) and model-invocable (Claude auto-detects from description)
- **Backward compatibility**: `.claude/commands/` still works; skills take precedence if names collide
- **Standard**: Follows [Agent Skills](https://agentskills.io) open standard

## 2. Claude Code Agents (Subagents) Convention (Official)

**Source**: code.claude.com/docs/en/sub-agents.md

- **Discovery**: Read at session start from `.claude/agents/`. Restart or use `/agents` to reload mid-session.
- **Delegation**: Claude reads `description` and auto-delegates matching tasks via Task tool.
- **Format**: YAML frontmatter + Markdown body (becomes system prompt for subagent)
- **Required fields**: `name`, `description`
- **Optional fields**: `tools`, `disallowedTools`, `model`, `permissionMode`, `maxTurns`, `skills`, `mcpServers`, `memory`, `hooks`
- **Scope priority**: CLI flag > project `.claude/agents/` > personal `~/.claude/agents/` > plugins
- **Key difference from skills**: Agents run in isolated context windows with their own system prompt; skills run inline in main conversation

## 3. Current State Audit Summary

### Skills (10/10 present)

All 10 skills exist at `.claude/skills/<name>/SKILL.md` with:
- Correct `name` and `description` frontmatter
- Supporting assets, scripts, and references subdirectories
- **Issue**: All subdirectory names have trailing spaces (e.g., `assets /` instead of `assets/`)
- **Gap**: No "Acceptance Checklist" section in any skill
- **Gap**: Section naming varies (some use "Core Features", others "Core Implementation Workflow")

### Agents (6/6 present)

All 6 agents exist at `.claude/agents/<name>.md` with:
- Correct `name`, `description`, `model: inherit` frontmatter
- Well-structured Mission/Non-negotiables/Operating procedure sections
- **Gap**: Some agents may lack explicit "Output requirements" section

### Routing Guide

No routing guide exists. `.claude/ROUTING.md` does not exist.

## 4. Decisions Made

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | SKILL.md template: 5 sections (Title, Non-Negotiable Rules, Quick Start, Core Workflow, Acceptance Checklist) | Balances structure with flexibility; matches existing patterns |
| 2 | Agent template: frontmatter (name, description, model) + body (Mission, Non-negotiables, Procedure, Output requirements) | Matches official docs; all agents already close to this |
| 3 | Routing guide at `.claude/ROUTING.md` | Auto-loaded by Claude Code; scannable by humans and AI |
| 4 | Fix directory trailing spaces via `git mv` | Preserves history; fixes cross-platform path issues |
| 5 | Default invocation policy (skills: user+model invocable; agents: auto-delegated via Task tool) | Matches Claude Code defaults; no special frontmatter needed |
