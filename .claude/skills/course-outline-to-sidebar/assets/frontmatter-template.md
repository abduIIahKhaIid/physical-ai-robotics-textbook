# MDX Frontmatter Templates

## Basic Chapter Template

```mdx
---
title: Chapter Title
sidebar_position: 1
---

# Chapter Title

[Introduction paragraph]

## Content Section

[Main content]
```

## Complete Chapter Template with All Fields

```mdx
---
title: Complete Guide to React Hooks
sidebar_position: 3
sidebar_label: React Hooks
description: Learn how to use React Hooks for state management and side effects
keywords:
  - react
  - hooks
  - useState
  - useEffect
tags:
  - react
  - javascript
  - frontend
slug: /module-3/react-hooks
---

# Complete Guide to React Hooks

React Hooks revolutionize how we write React components...

## Learning Objectives

After completing this chapter, you will be able to:

- Understand the purpose and benefits of React Hooks
- Use useState for state management
- Implement useEffect for side effects
- Create custom hooks

## Prerequisites

- Basic understanding of React components
- JavaScript ES6+ knowledge
- Familiarity with functional programming

## Introduction

[Content here...]

## Main Content

### Section 1

[Content...]

### Section 2

[Content...]

## Hands-on Examples

### Example 1: Simple Counter

[Code and explanation...]

### Example 2: Data Fetching

[Code and explanation...]

## Common Pitfalls

1. **Pitfall 1**: Description and solution
2. **Pitfall 2**: Description and solution

## Summary

Key takeaways from this chapter:

- Point 1
- Point 2
- Point 3

## Additional Resources

- [Official React Docs](https://react.dev)
- [Hooks API Reference](https://react.dev/reference/react)

## Practice Exercises

1. Exercise 1 description
2. Exercise 2 description
3. Exercise 3 description

## Next Steps

In the next chapter, we'll explore...
```

## Index Page Template

```mdx
---
title: Course Name
sidebar_position: 1
slug: /
---

# Course Name

Welcome to [Course Name]!

## Overview

This course provides comprehensive coverage of...

## What You'll Learn

By the end of this course, you will:

- Skill 1
- Skill 2
- Skill 3

## Course Structure

The course is organized into [X] modules:

### Module 1: [Title]
Brief description of module 1

### Module 2: [Title]
Brief description of module 2

### Module 3: [Title]
Brief description of module 3

## Prerequisites

Before starting this course, you should have:

- Prerequisite 1
- Prerequisite 2
- Prerequisite 3

## How to Use This Course

1. Start with the introduction
2. Follow the modules in order
3. Complete the hands-on exercises
4. Review the summary at the end of each chapter

## Getting Help

If you encounter issues:

- Check the FAQ section
- Review the additional resources
- Contact support at [email]

## Let's Get Started!

Ready to begin? Start with [Introduction](./introduction.mdx).
```

## Introduction Page Template

```mdx
---
title: Introduction
sidebar_position: 2
---

# Introduction

## Course Overview

This course covers...

## Learning Path

The course follows a structured learning path:

1. **Weeks 1-2**: Foundation concepts
2. **Weeks 3-5**: Core skills
3. **Weeks 6-8**: Advanced topics
4. **Weeks 9-10**: Capstone project

## Teaching Philosophy

Our approach focuses on:

- Hands-on learning
- Real-world applications
- Progressive complexity
- Continuous practice

## Course Materials

You'll need:

- Software/tools list
- Hardware requirements (if any)
- Textbook references
- Supplementary resources

## Assessment

Your progress will be evaluated through:

- Weekly exercises (30%)
- Module projects (40%)
- Final capstone (30%)

## Time Commitment

Expect to spend:

- 5-7 hours per week on lectures
- 3-5 hours on exercises
- 10-15 hours on projects

## Ready to Learn?

Let's dive into [Module 1](./module-1/week-1/1-first-chapter.mdx)!
```

## Module Overview Template

```mdx
---
title: Module 1 Overview
sidebar_position: 1
---

# Module 1: [Module Title]

## Module Objectives

In this module, you will:

- Objective 1
- Objective 2
- Objective 3

## Topics Covered

- Topic 1
- Topic 2
- Topic 3
- Topic 4

## Duration

This module spans [X] weeks and includes [Y] chapters.

## Module Structure

### Week 1-2: [Topic]
Introduction to foundational concepts

### Week 3-5: [Topic]
Deep dive into advanced techniques

## Prerequisites

Before starting this module:

- Previous module completion (if applicable)
- Required background knowledge
- Software setup

## Project

This module includes a hands-on project where you'll...

## Let's Begin

Start with [Week 1-2: First Topic](./week-1-2/1-first-chapter.mdx).
```

## Frontmatter Field Reference

### Required Fields

```yaml
title: Chapter Title              # Display title
sidebar_position: 1              # Order in sidebar (1, 2, 3...)
```

### Optional Fields

```yaml
sidebar_label: Short Label       # Override sidebar display text
description: Brief description   # Meta description for SEO
keywords:                        # SEO keywords
  - keyword1
  - keyword2
tags:                           # Content tags for categorization
  - tag1
  - tag2
slug: /custom-url-path          # Custom URL (overrides default)
hide_title: false               # Hide the H1 title
hide_table_of_contents: false   # Hide TOC sidebar
custom_edit_url: https://...    # Custom edit URL
```

### Advanced Fields

```yaml
image: ./banner.png             # Social media preview image
draft: false                    # Draft status (exclude from build)
unlisted: false                 # Unlisted (accessible but not in nav)
toc_min_heading_level: 2        # Minimum heading level in TOC
toc_max_heading_level: 3        # Maximum heading level in TOC
displayed_sidebar: tutorialSidebar  # Which sidebar to show
pagination_next: null           # Disable next page link
pagination_prev: null           # Disable previous page link
```

## Best Practices

1. **Always include title and sidebar_position**
2. **Use description for SEO** (150-160 characters)
3. **Add keywords** for better discoverability
4. **Keep slugs stable** once published
5. **Use tags consistently** across related content
6. **Provide sidebar_label** for long titles
7. **Set appropriate TOC levels** for readability