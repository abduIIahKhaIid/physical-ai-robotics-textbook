# Naming Conventions and Slug Rules

Comprehensive guide to consistent naming across the documentation structure.

## File and Folder Naming

### Kebab-case Standard

All files and folders use kebab-case (lowercase with hyphens):

```
✅ GOOD:
- robotic-nervous-system/
- 1-introduction.mdx
- physical-ai-basics.mdx

❌ BAD:
- RoboticNervousSystem/
- 1_Introduction.mdx
- Physical AI Basics.mdx
```

### Module Folders

Format: `module-{number}-{slug}`

```
module-1-robotic-nervous-system/
module-2-digital-twin/
module-3-ai-robot-brain/
```

### Week Folders

Format: `week-{start}` or `week-{start}-{end}`

```
week-1/          # Single week
week-1-2/        # Weeks 1-2
week-3-5/        # Weeks 3-5
```

### Chapter Files

Format: `{position}-{slug}.mdx`

```
1-introduction.mdx
2-core-concepts.mdx
3-hands-on-practice.mdx
```

**Position numbering:**
- Start at 1 within each week folder
- Use sequential integers
- No gaps in numbering

## Slug Generation

### Slugification Algorithm

```python
def slugify(text: str) -> str:
    """Convert text to URL-friendly slug."""
    # 1. Remove content in parentheses
    text = re.sub(r'\([^)]*\)', '', text)
    
    # 2. Convert to lowercase
    text = text.lower()
    
    # 3. Replace special chars with hyphens
    text = re.sub(r'[^a-z0-9]+', '-', text)
    
    # 4. Remove leading/trailing hyphens
    text = text.strip('-')
    
    # 5. Collapse multiple hyphens
    text = re.sub(r'-+', '-', text)
    
    return text
```

### Examples

```
"Introduction to ROS 2" → "introduction-to-ros-2"
"The Robotic Nervous System (ROS 2)" → "robotic-nervous-system"
"AI/ML Fundamentals" → "ai-ml-fundamentals"
"Setup & Configuration" → "setup-configuration"
"Week 1-2: Getting Started" → "week-1-2-getting-started"
```

### Special Cases

**Acronyms**: Keep as lowercase
```
"ROS 2" → "ros-2"
"NVIDIA Isaac" → "nvidia-isaac"
"URDF Format" → "urdf-format"
```

**Numbers**: Preserve in slug
```
"Week 3-5" → "week-3-5"
"Module 2" → "module-2"
```

**Symbols**: Remove or convert
```
"C++ Basics" → "c-basics"
"API & SDK" → "api-sdk"
"Cost: $100" → "cost-100"
```

## Document IDs

Document IDs match the file path relative to `docs/` without extension:

```
File: docs/module-1-topic/week-1/1-intro.mdx
ID:   module-1-topic/week-1/1-intro
```

### ID Requirements

1. **Uniqueness**: No duplicate IDs across all docs
2. **Consistency**: Match exact file path structure
3. **Stability**: Avoid changing IDs (breaks links)

## Frontmatter Title vs Label

### title (in frontmatter)

Original, human-readable title:

```yaml
---
title: Introduction to ROS 2
---
```

### sidebar_label (optional override)

Shorter version for sidebar:

```yaml
---
title: Introduction to ROS 2 - Complete Guide
sidebar_label: ROS 2 Intro
---
```

## Category Labels

Category labels in `_category_.json` and `sidebars.ts`:

### Standard Format

```
Module {n}: {Title}
Week {n}: {Topic}
Weeks {n}-{m}: {Topic}
```

### Examples

```json
{
  "label": "Module 1: The Robotic Nervous System"
}
```

```typescript
{
  type: 'category',
  label: 'Week 1-2: Introduction to Physical AI',
}
```

## Consistency Rules

### Cross-reference Alignment

Ensure slugs match across:
1. File/folder names
2. Document IDs in sidebars.ts
3. Category labels
4. Internal links

### Validation Checklist

- [ ] All files use kebab-case
- [ ] No spaces in file/folder names
- [ ] Position numbers are sequential
- [ ] Slugs are lowercase
- [ ] No special characters except hyphens
- [ ] Module/week/chapter hierarchy is clear

## Common Mistakes

### ❌ Mixed Case

```
Module-1-Topic/  # Should be: module-1-topic/
Week_1/          # Should be: week-1/
```

### ❌ Spaces

```
module 1/        # Should be: module-1/
my chapter.mdx   # Should be: my-chapter.mdx
```

### ❌ Underscores

```
module_1/        # Should be: module-1/
1_intro.mdx      # Should be: 1-intro.mdx
```

### ❌ Special Characters

```
c++_basics.mdx   # Should be: c-basics.mdx
api&sdk.mdx      # Should be: api-sdk.mdx
```

### ❌ Inconsistent Numbering

```
week-1/
week-3/          # Missing week-2
week-4/
```

## Migration Strategy

When restructuring existing content:

1. **Create mapping file**: Document old → new paths
2. **Update sidebars.ts**: Change document IDs
3. **Set up redirects**: Preserve old URLs
4. **Update internal links**: Find and replace references
5. **Validate**: Run validation script

Example redirect configuration:

```javascript
// docusaurus.config.js
module.exports = {
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          redirects: [
            {
              from: '/old-path/chapter',
              to: '/module-1/week-1/1-chapter',
            },
          ],
        },
      },
    ],
  ],
};
```