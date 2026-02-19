---
name: <skill-name>                    # Required: lowercase-kebab-case, matches directory name
description: <one-paragraph summary>  # Required: loaded into system prompt for discovery
---

# <Skill Title>

<One-sentence purpose statement.>

## Non-Negotiable Rules

<!-- Hard constraints the assistant MUST follow when executing this skill. -->

- <Hard constraint 1>
- <Hard constraint 2>
- No secrets, real credentials, or API keys in outputs

## Quick Start

<!-- Minimal copy-paste example showing the most common invocation. -->

```text
/<skill-name> <example arguments>
```

Expected output: <brief description of what the skill produces>

## Core Implementation Workflow

<!-- Step-by-step procedure the assistant follows when this skill is invoked. -->

1. <Step 1>
2. <Step 2>
3. <Step 3>

## Acceptance Checklist

<!-- Verify output quality before delivering to the user. -->

- [ ] <Output meets criterion 1>
- [ ] <Output meets criterion 2>
- [ ] No template placeholders remain
- [ ] No real secrets or credentials in output
