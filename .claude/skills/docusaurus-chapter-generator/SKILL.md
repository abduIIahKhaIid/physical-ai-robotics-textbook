---
name: docusaurus-chapter-generator
description: Generate complete, consistent Docusaurus chapter pages (Markdown/MDX) aligned with course structure (module → week → lesson) and project writing conventions. Use when creating new chapter/lesson pages for a textbook, rewriting existing chapters to match standard structure, or producing multiple chapters quickly with consistent formatting. Triggers on requests like "create a chapter", "add a lesson page", "generate chapter content", or "write a Docusaurus page".
---

# Docusaurus Chapter Generator

Generate professional, pedagogically-sound chapter pages for Docusaurus-based educational courses following a modular structure.

## Non-Negotiable Rules

- All generated chapters MUST be valid Docusaurus 3.x MDX/Markdown
- Every chapter MUST include all pedagogical sections: Learning Objectives, Key Terms, Main Content, Recap, Practice Exercises, Quiz
- Never include real API keys, passwords, or secrets in code examples — use placeholder values only
- Every file MUST have `sidebar_position` and `title` in frontmatter
- Use second person ("you will learn"), active voice, present tense throughout

## Quick Start

```text
/docusaurus-chapter-generator Module 1, Week 2, Lesson 1.2 — Introduction to ROS 2
```

Expected output: A complete chapter `.md` file with all pedagogical sections, proper frontmatter, and no template placeholders.

## Standard Chapter Structure

Every chapter must follow this exact structure:

```markdown
---
sidebar_position: N
title: "Chapter Title"
---

# Chapter Title

## Learning Objectives

By the end of this chapter, you will:
- [Concrete, measurable objective 1]
- [Concrete, measurable objective 2]
- [Concrete, measurable objective 3]

## Introduction

[2-3 paragraphs providing context, motivation, and chapter overview]

## [Main Section 1]

[Content organized with clear headers, examples, and explanations]

### [Subsection if needed]

[Detailed content with code examples where appropriate]

## [Main Section 2]

[Continue pattern...]

## Key Takeaways

- **[Concept 1]**: [Brief explanation]
- **[Concept 2]**: [Brief explanation]
- **[Concept 3]**: [Brief explanation]

## Practice Exercises

### Exercise 1: [Title]

**Objective**: [What student will accomplish]

**Instructions**:
1. [Step-by-step instructions]
2. [...]

**Hints**:
- [Helpful guidance without giving away solution]

### Exercise 2: [Title]

[Follow same pattern...]

## Further Reading

- [Resource 1 with link and brief description]
- [Resource 2 with link and brief description]

## Next Steps

[1-2 sentences connecting to the next chapter/lesson]
```

## Frontmatter Guidelines

- `sidebar_position`: Integer for chapter ordering within the section
- `title`: Concise, descriptive title (use sentence case)
- Optional fields when needed:
  - `description`: Brief meta description for SEO
  - `tags`: Array of relevant tags for categorization

## Content Quality Standards

### Writing Style
- Use clear, accessible language appropriate for the target audience
- Employ active voice and present tense
- Write in second person ("you will learn") for direct engagement
- Break complex concepts into digestible chunks
- Use consistent terminology throughout

### Code Examples
- Include syntax highlighting with language tags: ```python, ```javascript, etc.
- Provide complete, runnable examples when possible
- Add inline comments explaining key concepts
- Show both correct usage and common mistakes (clearly labeled)

### Visual Elements
Use Docusaurus admonitions for emphasis:

```markdown
:::tip
Helpful advice or best practices
:::

:::warning
Important caveats or common pitfalls
:::

:::info
Supplementary information
:::

:::danger
Critical information about errors or risks
:::
```

### Learning Objectives
- Start with action verbs (Bloom's taxonomy: understand, apply, analyze, create)
- Make objectives specific and measurable
- Align with chapter content and exercises
- Limit to 3-5 objectives per chapter

### Exercises
- Progress from simple to complex
- Include diverse exercise types: conceptual questions, coding challenges, analysis tasks
- Provide scaffolding through hints without revealing full solutions
- Ensure exercises reinforce learning objectives

## File Organization

Place chapters in the appropriate directory structure:

```
docs/
├── module-1/
│   ├── week-1/
│   │   ├── lesson-1.md
│   │   ├── lesson-2.md
│   │   └── lesson-3.md
│   └── week-2/
│       └── ...
├── module-2/
│   └── ...
```

## Generating Multiple Chapters

When creating multiple related chapters:

1. **Ensure consistency**: Use the same terminology, voice, and formatting
2. **Create logical flow**: Each chapter should build on previous content
3. **Cross-reference appropriately**: Link to related chapters using relative paths
4. **Maintain scope**: Keep each chapter focused on a single major concept
5. **Balance depth**: Similar complexity and length across chapters in the same module

## Common Patterns

### Linking to Other Chapters
```markdown
For more on this topic, see [Arrays and Lists](../week-2/lesson-1.md).
```

### Including Code Blocks with Titles
```markdown
```python title="example.py"
def hello_world():
    print("Hello, World!")
```
\```
```

### Creating Tabs for Multiple Languages
```markdown
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs>
  <TabItem value="python" label="Python">
    ```python
    print("Hello")
    ```
  </TabItem>
  <TabItem value="javascript" label="JavaScript">
    ```javascript
    console.log("Hello");
    ```
  </TabItem>
</Tabs>
```

## Core Implementation Workflow

1. **Gather requirements**: Confirm chapter topic, target audience, module/week placement
2. **Research if needed**: Review related content to ensure consistency
3. **Generate structure**: Create the chapter following the standard template
4. **Add content**: Fill in sections with clear explanations and examples
5. **Create exercises**: Develop practice problems aligned with objectives
6. **Review**: Check for clarity, accuracy, and adherence to standards
7. **Place file**: Save in correct directory with appropriate frontmatter

## Before Delivery

Verify each chapter includes:
- [ ] Proper frontmatter with sidebar_position and title
- [ ] 3-5 clear learning objectives
- [ ] Introduction explaining context and motivation
- [ ] Well-structured main content with headers
- [ ] Code examples with syntax highlighting
- [ ] Key takeaways summarizing main points
- [ ] At least 2-3 practice exercises with hints
- [ ] Further reading resources
- [ ] Next steps connecting to following content

## Acceptance Checklist

- [ ] Chapter has Learning Objectives section
- [ ] Chapter has Key Terms section
- [ ] Chapter has Main Explanation content with structured headers
- [ ] Chapter has Lab/Exercise section (when applicable)
- [ ] Chapter has Recap section
- [ ] Chapter has Quiz section
- [ ] No template placeholders remain
- [ ] Frontmatter has `sidebar_position` and `title`