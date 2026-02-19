# Rule Examples for Physical AI Course Modules

This document provides comprehensive personalization rule examples for each module of the Physical AI & Humanoid Robotics course.

## Module 1: The Robotic Nervous System (ROS 2)

### Beginner Rules

```json
{
  "id": "ros2_beginner",
  "condition": {
    "experience_level": "beginner"
  },
  "transformations": [
    "expand_explanations",
    "add_glossary_hints",
    {
      "type": "add_examples",
      "examples": [
        "ros2_hello_world",
        "simple_publisher_subscriber"
      ]
    },
    "insert_prereq_links"
  ],
  "priority": 10
}
```

**Effect:**
- Adds detailed explanations of nodes, topics, and services
- Tooltips for terms like "rclpy", "QoS", "middleware"
- Extra code examples with step-by-step comments
- Links to Python basics if needed

### Advanced Rules

```json
{
  "id": "ros2_advanced",
  "condition": {
    "experience_level": "advanced"
  },
  "transformations": [
    "hide_beginner_explanations",
    {
      "type": "add_advanced_content",
      "content": "ros2_performance_tuning"
    },
    {
      "type": "suggest_readings",
      "readings": ["ros2_design_docs", "dds_internals"]
    }
  ],
  "priority": 10
}
```

**Effect:**
- Hides basic Python/programming explanations
- Adds sections on DDS configuration, QoS tuning
- Links to ROS 2 design documents
- Suggests advanced performance optimization techniques

---

## Module 2: The Digital Twin (Gazebo & Unity)

### Low Hardware Rules

```json
{
  "id": "simulation_low_hardware",
  "condition": {
    "hardware_tier": "low"
  },
  "transformations": [
    {
      "type": "swap_lab_variant",
      "variant": "low_gpu",
      "labs": ["gazebo_intro", "physics_simulation"]
    },
    "add_cloud_alternative",
    {
      "type": "optimize_settings",
      "settings": {
        "reduce_physics_rate": true,
        "disable_shadows": true,
        "lower_texture_quality": true
      }
    }
  ],
  "priority": 20
}
```

**Effect:**
- Uses Gazebo-only labs (skips Unity/Isaac Sim)
- Reduces physics update rates (10Hz instead of 100Hz)
- Disables GPU-intensive features
- Shows AWS RoboMaker setup guide
- Provides optimized .world files

### High Hardware Rules

```json
{
  "id": "simulation_high_hardware",
  "condition": {
    "hardware_tier": "high"
  },
  "transformations": [
    {
      "type": "swap_lab_variant",
      "variant": "high_gpu",
      "labs": ["unity_integration", "isaac_sim_intro"]
    },
    {
      "type": "enable_features",
      "features": ["real_time_ray_tracing", "high_fidelity_physics"]
    },
    {
      "type": "suggest_advanced_labs",
      "labs": ["multi_robot_simulation", "photorealistic_rendering"]
    }
  ],
  "priority": 20
}
```

**Effect:**
- Includes Unity integration labs
- Enables RTX ray tracing in Unity
- Uses high-resolution textures and meshes
- Adds multi-robot coordination labs
- Suggests Isaac Sim advanced features

---

## Module 3: The AI-Robot Brain (NVIDIA Isaac™)

### Without Jetson

```json
{
  "id": "isaac_no_jetson",
  "condition": {
    "has_jetson": false
  },
  "transformations": [
    {
      "type": "focus_on_sim",
      "modules": ["isaac_sim_only"]
    },
    {
      "type": "add_note",
      "note": "These concepts will be fully tested when you get Jetson hardware"
    },
    {
      "type": "suggest_alternatives",
      "alternatives": ["x86_deployment", "docker_containers"]
    }
  ],
  "priority": 15
}
```

**Effect:**
- Focuses on Isaac Sim simulation
- Skips hardware-specific deployment sections
- Shows x86 alternatives for testing
- Encourages cloud deployment

### With Jetson

```json
{
  "id": "isaac_with_jetson",
  "condition": {
    "has_jetson": true
  },
  "transformations": [
    {
      "type": "add_hardware_labs",
      "labs": ["jetson_deployment", "edge_optimization", "real_sensor_integration"]
    },
    {
      "type": "highlight_sections",
      "sections": ["isaac_ros", "deployment", "optimization"]
    },
    "add_jetson_setup_guide"
  ],
  "priority": 15
}
```

**Effect:**
- Adds Jetson-specific deployment labs
- Shows how to optimize models for edge
- Includes real RealSense camera integration
- Adds power/thermal management sections

---

## Module 4: Vision-Language-Action (VLA)

### Focused on VLA

```json
{
  "id": "vla_focus",
  "condition": {
    "focus": "vla"
  },
  "transformations": [
    {
      "type": "expand_sections",
      "sections": ["llm_integration", "whisper_setup", "action_planning"]
    },
    {
      "type": "add_extra_projects",
      "projects": [
        "multi_modal_robot_control",
        "voice_commanded_navigation",
        "visual_question_answering"
      ]
    },
    {
      "type": "reorder_content",
      "priority_sections": ["vla", "cognitive_planning", "multi_modal"]
    }
  ],
  "priority": 5
}
```

**Effect:**
- Expands VLA content significantly
- Adds extra projects on multimodal control
- Moves VLA content earlier in chapter order
- Suggests latest research papers

---

## Language-Specific Rules

### Urdu Translation

```json
{
  "id": "urdu_content",
  "condition": {
    "language": "ur"
  },
  "transformations": [
    "translate_content",
    "preserve_code_blocks",
    "add_bilingual_glossary",
    {
      "type": "use_rtl_layout",
      "exceptions": ["code", "math", "diagrams"]
    }
  ],
  "priority": 100
}
```

**Effect:**
- Translates all prose to Urdu
- Keeps code, commands, file paths in English
- Uses right-to-left text direction
- Adds glossary with English terms in parentheses

**Example:**

```markdown
# ROS 2 کا تعارف (Introduction to ROS 2)

ROS 2 ایک جدید ربوٹک فریم ورک ہے جو Nodes، Topics، اور Services استعمال کرتا ہے۔

<details>
<summary>Node (نوڈ)</summary>
ایک نوڈ ایک executable process ہے جو کام انجام دیتا ہے۔
</summary>
</details>

```python
# Code blocks remain in English
import rclpy
from rclpy.node import Node
```

**اصطلاحات (Glossary):**
- Node (نوڈ) - Executable process
- Topic (ٹاپک) - Communication channel
```

---

## Combined Rules Examples

### Beginner + Low Hardware + Urdu

```json
{
  "rules": [
    {
      "id": "urdu_beginner_low_hw",
      "condition": {
        "experience_level": "beginner",
        "hardware_tier": "low",
        "language": "ur"
      },
      "transformations": [
        "translate_content",
        "expand_explanations",
        "add_glossary_hints",
        {"type": "swap_lab_variant", "variant": "low_gpu"},
        "add_cloud_alternative",
        "insert_prereq_links"
      ],
      "priority": 100
    }
  ]
}
```

**Effect:**
- Urdu content with maximum explanations
- Gazebo-only labs
- Cloud setup instructions in Urdu
- Bilingual glossary
- Step-by-step beginner guides

### Advanced + High Hardware + ROS 2 Focus

```json
{
  "rules": [
    {
      "id": "advanced_high_ros2",
      "condition": {
        "experience_level": "advanced",
        "hardware_tier": "high",
        "focus": "ros2"
      },
      "transformations": [
        "hide_beginner_explanations",
        {"type": "swap_lab_variant", "variant": "high_gpu"},
        {"type": "highlight_sections", "sections": ["ros2", "rclpy", "services"]},
        {"type": "add_advanced_content", "content": "ros2_advanced_patterns"},
        {"type": "suggest_readings", "readings": ["ros2_dds_deep_dive"]}
      ],
      "priority": 100
    }
  ]
}
```

**Effect:**
- Concise, advanced content
- Full GPU-accelerated labs
- Extra ROS 2 content
- DDS internals documentation
- Advanced design pattern examples

---

## Time-Based Personalization

### Part-Time Learners

```json
{
  "id": "part_time_pacing",
  "condition": {
    "time_commitment": "part_time"
  },
  "transformations": [
    {
      "type": "add_weekly_plan",
      "hours_per_week": 8
    },
    {
      "type": "suggest_quick_labs",
      "max_duration": "2_hours"
    },
    {
      "type": "add_progress_tracker",
      "checkpoints": "weekly"
    }
  ],
  "priority": 5
}
```

**Effect:**
- Shows "Week 1-2 Plan" breakdowns
- Prioritizes shorter labs
- Adds weekly checkpoint quizzes
- Suggests time-saving shortcuts

---

## Learning Style Rules

### Hands-On Learners

```json
{
  "id": "hands_on_style",
  "condition": {
    "learning_style": "hands_on"
  },
  "transformations": [
    {
      "type": "reorder_content",
      "order": ["lab", "theory", "references"]
    },
    {
      "type": "expand_labs",
      "add_extra_challenges": true
    },
    "minimize_theory_blocks"
  ],
  "priority": 5
}
```

**Effect:**
- Labs appear before theory
- More hands-on exercises
- Theory sections are collapsible
- Extra "Try This" challenges

### Theory-First Learners

```json
{
  "id": "theory_first_style",
  "condition": {
    "learning_style": "theory_first"
  },
  "transformations": [
    {
      "type": "reorder_content",
      "order": ["theory", "examples", "lab"]
    },
    {
      "type": "expand_theory",
      "add_math_derivations": true
    },
    {
      "type": "add_conceptual_diagrams",
      "count": "extra"
    }
  ],
  "priority": 5
}
```

**Effect:**
- Theory sections expanded
- Mathematical derivations shown
- More conceptual diagrams
- Labs appear after understanding concepts

---

## Rule Priority Guidelines

- **100**: Language/translation (highest priority)
- **50**: Hardware constraints
- **20**: Experience level
- **10**: Content expansion/simplification
- **5**: Focus area emphasis
- **1**: Style/preference tweaks

Higher priority rules apply first and can't be overridden by lower priority rules.
