---
name: personalization-rules-engine
description: Define deterministic personalization rules for chapters based on onboarding profile (beginner/intermediate/advanced, hardware tier, language). Produces stable UX and safe content transformations. Use when implementing chapter personalization (Spec 014), designing onboarding questions (Spec 013), or ensuring personalization is predictable and testable for Physical AI & Humanoid Robotics course content.
---

# Personalization Rules Engine

## Overview

Create a rule-based personalization layer that transforms chapter presentation using a stored user profile while maintaining technical accuracy and providing deterministic, testable output.

## Non-Negotiable Rules

1. **Deterministic**: Same profile + same chapter → same output every time
2. **Truthful**: Never rewrite technical truth; only adjust depth, pacing, examples, and tooling paths
3. **Reversible**: Always allow users to revert to default content
4. **Explainable**: Users must understand why they're seeing personalized content
5. **Safe**: All transformations preserve technical accuracy and safety

## Quick Start

```python
# 1. Load user profile from onboarding
profile = {
    "experience_level": "beginner",
    "hardware_tier": "low",
    "focus": "ros2",
    "language": "en"
}

# 2. Apply transformation rules
from scripts.apply_rules import apply_personalization_rules

personalized_content = apply_personalization_rules(
    chapter_content,
    profile,
    rules_config="config/personalization_rules.json"
)

# 3. Add UI indicators
add_personalization_badge(profile)
```

## Profile Schema

### Core Attributes (Required)

```typescript
interface UserProfile {
  experience_level: "beginner" | "intermediate" | "advanced";
  hardware_tier: "low" | "mid" | "high";
  language: "en" | "ur";
}
```

### Optional Attributes

```typescript
interface ExtendedProfile extends UserProfile {
  focus?: "simulation" | "ros2" | "isaac" | "humanoid" | "vla";
  gpu_available?: boolean;
  gpu_model?: string;  // e.g., "RTX 4070 Ti"
  ram_gb?: number;
  has_jetson?: boolean;
  learning_style?: "hands_on" | "theory_first" | "balanced";
  time_commitment?: "full_time" | "part_time" | "weekend";
}
```

For complete onboarding question design, see `references/onboarding_questions.md`.

## Transformation Primitives

### Content Depth

- **`expand_explanations`**: Add detailed explanations for complex concepts
- **`add_glossary_hints`**: Insert inline glossary tooltips
- **`hide_advanced_blocks`**: Collapse advanced/optional sections
- **`simplify_language`**: Use simpler terminology for beginners

### Lab Path Selection

- **`swap_lab_variant`**: Choose hardware-appropriate lab exercises
  - `low_gpu`: Gazebo-only path, CPU simulations
  - `mid_gpu`: Mixed Gazebo/Isaac Sim
  - `high_gpu`: Full Isaac Sim + acceleration
  
- **`add_cloud_alternative`**: Show AWS/cloud alternatives for resource-intensive labs

### Navigation & Guidance

- **`insert_prereq_links`**: Add "Review this first" links for beginners
- **`add_recommended_next`**: Suggest next lesson based on focus area
- **`reorder_optional_reading`**: Prioritize readings by focus area

### Language Support

- **`translate_content`**: Apply Urdu translation
- **`preserve_code_blocks`**: Keep code/commands in English
- **`add_bilingual_glossary`**: Include English terms in parentheses

## Rule Definition

Rules are stored in `config/personalization_rules.json`:

```json
{
  "rules": [
    {
      "id": "beginner_expand",
      "condition": {
        "experience_level": "beginner"
      },
      "transformations": [
        "expand_explanations",
        "add_glossary_hints",
        "insert_prereq_links"
      ],
      "priority": 10
    },
    {
      "id": "low_hardware_path",
      "condition": {
        "hardware_tier": "low"
      },
      "transformations": [
        {"type": "swap_lab_variant", "variant": "low_gpu"},
        "add_cloud_alternative"
      ],
      "priority": 20
    },
    {
      "id": "isaac_focus",
      "condition": {
        "focus": "isaac"
      },
      "transformations": [
        {"type": "highlight_sections", "sections": ["isaac", "perception"]},
        {"type": "reorder_optional_reading", "prioritize": "isaac_docs"}
      ],
      "priority": 5
    }
  ]
}
```

## Core Implementation Workflow

### Step 1: Define Rules Configuration

Create `config/personalization_rules.json` with rule definitions. Map each profile attribute to transformation primitives and set priority levels (higher = applied first).

### Step 2: Implement Transformation Functions

Use `scripts/apply_rules.py` to implement each primitive. Each function should:
- Take content and context parameters
- Return transformed content
- Preserve technical accuracy
- Be idempotent (applying twice = applying once)

### Step 3: Add UI Components

Implement frontend components (see `references/ui_patterns.md`):

```jsx
// Personalization indicator
<PersonalizationBadge profile={userProfile} />

// Toggle to default content
<button onClick={showDefaultContent}>
  View Original Version
</button>

// Explanation of personalization
<div className="personalization-info">
  Content customized for: {profile.experience_level} + 
  {profile.hardware_tier} hardware
</div>
```

### Step 4: Testing & Validation

1. **Determinism test**: Same input → same output (use `scripts/test_determinism.py`)
2. **Accuracy test**: Technical content remains correct
3. **Reversion test**: Can switch back to default
4. **Performance test**: Personalization applies in <500ms

## Common Patterns

### Pattern 1: Beginner with Low Hardware

```json
{
  "experience_level": "beginner",
  "hardware_tier": "low"
}
```

**Transformations applied:**
- Expand all explanations
- Use Gazebo-only labs
- Add glossary hints
- Insert prerequisite links
- Show cloud alternatives

### Pattern 2: Advanced with High Hardware

```json
{
  "experience_level": "advanced",
  "hardware_tier": "high"
}
```

**Transformations applied:**
- Hide beginner explanations
- Use full Isaac Sim labs
- Show advanced references
- Enable GPU-accelerated paths

### Pattern 3: Language Personalization

```json
{
  "experience_level": "intermediate",
  "language": "ur"
}
```

**Transformations applied:**
- Translate prose to Urdu
- Preserve code blocks in English
- Add bilingual glossary
- Keep technical terms in English with Urdu context

## Acceptance Checklist

✅ **Determinism**: Run test suite with `scripts/test_determinism.py`

✅ **UI Clarity**: Every personalized chapter shows:
   - "Personalized for: [profile summary]"
   - Toggle button to view default content
   
✅ **Technical Accuracy**: All transformations validated by subject matter expert

✅ **No Breaking Changes**: 
   - No broken links
   - No missing sections
   - All code examples still work
   
✅ **Performance**: Personalization applies in <500ms

## Troubleshooting

**Problem**: Personalized content has broken links

**Solution**: Use `scripts/validate_rules.py --check-links` to verify all transformed content

---

**Problem**: Users can't understand why content changed

**Solution**: Always include clear UI badge explaining the personalization applied

---

**Problem**: Transformations conflict with each other

**Solution**: Use priority levels in rules. Higher priority rules apply first and can't be overridden.

---

**Problem**: Translation breaks code blocks

**Solution**: Use `preserve_code_blocks` transformation to keep all code in English

## Resources

### scripts/

- **`apply_rules.py`**: Core rule application engine
- **`validate_rules.py`**: Validate rule configuration
- **`test_determinism.py`**: Test personalization determinism

### references/

- **`onboarding_questions.md`**: Complete onboarding question design
- **`ui_patterns.md`**: React/Frontend implementation patterns
- **`rule_examples.md`**: Comprehensive rule examples for each course module

### assets/

- **`lab_variants/`**: Alternative lab instructions for different hardware tiers
  - `low_gpu/`: Gazebo-only labs
  - `mid_gpu/`: Mixed approach
  - `high_gpu/`: Full Isaac Sim labs
  
- **`config_templates/`**: Template rule configurations
  - `beginner_rules.json`
  - `advanced_rules.json`
  - `hardware_specific_rules.json`