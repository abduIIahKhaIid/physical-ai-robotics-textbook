---
name: hardware-requirements-writer
description: Write or update Hardware Requirements sections for Physical AI & Humanoid Robotics textbooks, course materials, or procurement documentation. Use when creating hardware setup guides, system requirements pages, procurement checklists, or environment configuration documentation for courses involving ROS 2, Isaac Sim, Gazebo, Unity, edge AI deployment (Jetson), or robotics labs. Triggers on requests to document hardware needs, create equipment lists, write setup requirements, or guide hardware procurement.
---

# Hardware Requirements Writer

## Non-Negotiable Rules

- MUST cite specific hardware model numbers and specifications — no vague "high-performance GPU" claims
- MUST include cost estimates (or price ranges) where available
- MUST distinguish between minimum and recommended specifications
- No obsolete or discontinued hardware recommendations
- No real procurement credentials or vendor account details

## Overview

This skill helps create clear, actionable hardware requirements documentation for Physical AI and Humanoid Robotics courses. It addresses the unique challenge of documenting requirements for computationally intensive workflows combining physics simulation, visual perception, and generative AI.

## Core Implementation Workflow

### 1. Identify the Target Audience and Context

Before writing, determine:
- **Course level**: Beginner, intermediate, advanced
- **Budget constraints**: Educational institution, hobbyist, enterprise
- **Deployment model**: On-premise lab, cloud-native, hybrid
- **Primary use case**: Simulation-only, sim-to-real, physical robot deployment

### 2. Structure the Documentation

Use this recommended structure:

```markdown
# Hardware Requirements

## Overview
[Brief explanation of why specific hardware is needed]

## Minimum vs Recommended Specifications
[Clear tier system]

## Component Breakdown
### Simulation Workstation
### Edge Computing Kit (if applicable)
### Sensors and Peripherals (if applicable)
### Robot Hardware (if applicable)

## Cost Analysis
[Budget tiers with justifications]

## Alternative Configurations
[Cloud options, budget alternatives]

## Procurement Checklist
[Actionable shopping list]
```

### 3. Apply Domain-Specific Guidelines

**For Simulation Workstations (Isaac Sim, Gazebo, Unity):**
- Always specify GPU model with VRAM (not just "high-end GPU")
- Explain *why* each spec matters (e.g., "64GB RAM prevents crashes during scene rendering")
- Include OS requirements (Ubuntu version for ROS 2 compatibility)
- Mention compatibility limitations (e.g., "MacBooks cannot run Isaac Sim")

**For Edge Computing (Jetson deployment):**
- Specify exact Jetson model (Orin Nano 8GB vs Orin NX 16GB)
- Include required peripherals (RealSense camera, IMU, microphone)
- Explain role differentiation (training vs inference)
- Provide power/connectivity requirements

**For Robot Hardware:**
- Offer tiered options (budget proxy, miniature, premium)
- Explain what software concepts transfer to each tier
- Include SDK/ROS 2 compatibility notes
- Provide durability and support considerations

### 4. Write with Clarity and Honesty

**Do:**
- Use specific part numbers and models
- Provide realistic cost estimates
- Explain technical trade-offs
- Offer multiple tiers (minimum, recommended, ideal)
- Include "why this matters" for each requirement
- Link requirements to specific course modules

**Don't:**
- Use vague terms like "powerful PC" or "modern GPU"
- Recommend outdated hardware without caveat
- Hide costs or complexity
- Assume reader's hardware knowledge
- Copy marketing materials verbatim

### 5. Reference Templates and Examples

For detailed examples and templates, see:
- `references/workstation_specs.md` - Simulation workstation configurations
- `references/edge_kit_specs.md` - Jetson and sensor configurations
- `references/cost_tables.md` - Budget tier templates
- `references/cloud_alternatives.md` - AWS/Azure configurations

Load these references as needed when writing specific sections.

## Special Considerations

### Isaac Sim Requirements
Isaac Sim is the primary bottleneck. Always emphasize:
- RTX GPU requirement (ray tracing cores needed)
- VRAM minimum: 12GB, recommended: 24GB
- Ubuntu 22.04 LTS for native ROS 2 support
- Cannot run on non-RTX GPUs or standard laptops

### ROS 2 Compatibility
- Native to Linux (Ubuntu 22.04 LTS recommended)
- Windows support exists but creates friction
- macOS support very limited
- Recommend dual-boot or dedicated Linux machines

### Cloud vs On-Premise Decision Framework
Help readers decide by providing:

**On-Premise Lab (High CapEx):**
- Pros: No recurring costs, full control, no latency
- Cons: High upfront investment, maintenance burden
- Best for: Established institutions, long-term programs

**Cloud-Native (High OpEx):**
- Pros: Rapid deployment, scalable, lower entry barrier
- Cons: Recurring costs, latency for physical control
- Best for: Proof-of-concept, weak local hardware, temporary programs

### Progressive Disclosure Pattern
Start with essentials, link to detailed specifications:

```markdown
## Quick Start Hardware Guide
[Minimal working configuration]

## Detailed Specifications
For complete technical details, see:
- [Workstation Specifications](./workstation-specs)
- [Edge Kit Setup](./edge-kit-setup)
- [Cloud Configuration](./cloud-setup)
```

## Quality Checklist

Before finalizing documentation, verify:

- [ ] All GPU models include VRAM specifications
- [ ] All costs include approximate prices and currency
- [ ] OS requirements explicitly stated
- [ ] Compatibility limitations clearly noted
- [ ] At least two tiers provided (minimum and recommended)
- [ ] "Why this matters" explanations for key components
- [ ] Links to course modules that need each component
- [ ] Procurement checklist with specific product names/models
- [ ] Cloud alternative documented (if applicable)
- [ ] Beginner-friendly language without jargon overload

## Common Pitfalls to Avoid

1. **The "Latest and Greatest" Trap**: Don't recommend cutting-edge hardware unless truly necessary. Balance performance with availability and cost.

2. **The Vague Spec**: Avoid "powerful CPU" → Use "Intel i7-13700K or AMD Ryzen 9 7900X"

3. **Hidden Requirements**: Don't forget peripherals (monitors, keyboards, cables, power supplies)

4. **Ignoring Thermal/Power**: For edge devices, mention power consumption and cooling needs

5. **No Fallback Options**: Always provide alternatives for out-of-stock or budget-constrained scenarios

6. **Forgetting Software Compatibility**: Hardware specs mean nothing if OS/drivers aren't compatible

## Example Snippets

### Good Workstation Spec:
```markdown
**GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher
- *Why*: Isaac Sim requires RTX cores for ray tracing. 12GB VRAM minimum for loading USD robot assets + VLA models simultaneously. RTX 3090/4090 (24GB) recommended for smoother sim-to-real training.
- *Cost*: ~$800-1200 (RTX 4070 Ti), ~$1500-2000 (RTX 4090)
```

### Good Edge Kit Spec:
```markdown
**Brain**: NVIDIA Jetson Orin Nano Super Dev Kit (8GB) - $249
**Eyes**: Intel RealSense D435i - $349 (includes IMU for SLAM)
**Ears**: ReSpeaker USB Mic Array v2.0 - $69 (far-field voice commands)
**Total**: ~$700 per kit
```

## Integration with Course Material

When writing requirements for a full course:

1. Create a **main requirements page** with overview and tier system
2. Link to **module-specific requirements** (e.g., "Module 3: NVIDIA Isaac requires...")
3. Provide a **setup verification checklist** (how to test if hardware works)
4. Include **troubleshooting section** for common hardware issues

## File Organization Recommendation

For a Docusaurus textbook, structure like this:

```
docs/
├── hardware-requirements/
│   ├── index.md (overview + quick tier system)
│   ├── workstation.md (detailed simulation PC specs)
│   ├── edge-kit.md (Jetson + sensors)
│   ├── robot-hardware.md (physical robots, tiered)
│   ├── cloud-setup.md (AWS/Azure alternatives)
│   └── procurement-checklist.md (shopping list format)
```

## Acceptance Checklist

- [ ] All hardware recommendations cite specific model numbers
- [ ] Minimum vs recommended specs are clearly distinguished
- [ ] Cost estimates or price ranges are included
- [ ] No obsolete or discontinued hardware listed
- [ ] Component compatibility is verified (e.g., GPU fits in recommended chassis)
- [ ] No real procurement credentials in output