# Onboarding Questions Design

This document provides the complete question flow for user onboarding to collect profile information for personalization.

## Question Flow

### Question 1: Experience Level

**Question Text:**
> How would you describe your experience with robotics and AI?

**Options:**
- 🌱 **Beginner**: I'm new to robotics and AI
- 🌿 **Intermediate**: I have some programming experience
- 🌳 **Advanced**: I have experience with ROS, robotics, or machine learning

**Profile Key:** `experience_level`

**Values:** `beginner`, `intermediate`, `advanced`

---

### Question 2: Hardware Availability

**Question Text:**
> What hardware do you have access to for this course?

**Options:**
- 💻 **Standard Laptop/Desktop** (No dedicated GPU)
- 🎮 **Mid-range GPU** (GTX 1660 or similar, 6-8GB VRAM)
- 🚀 **High-end GPU** (RTX 3080+ or similar, 12GB+ VRAM)
- ☁️ **Cloud Access Only** (I'll use AWS/GCP/Azure)

**Profile Keys:** `hardware_tier`, `gpu_available`

**Mapping:**
- Standard Laptop → `hardware_tier: "low"`, `gpu_available: false`
- Mid-range GPU → `hardware_tier: "mid"`, `gpu_available: true`
- High-end GPU → `hardware_tier: "high"`, `gpu_available: true`
- Cloud Only → `hardware_tier: "mid"`, `gpu_available: true`

---

### Question 3: Learning Focus (Optional)

**Question Text:**
> Which area are you most interested in learning about?

**Options:**
- 🎮 **Robot Simulation** (Gazebo, Unity, Isaac Sim)
- 🤖 **ROS 2 Development** (Nodes, topics, services)
- 👁️ **AI & Perception** (Computer vision, SLAM, Isaac platform)
- 🚶 **Humanoid Robotics** (Bipedal locomotion, manipulation)
- 🧠 **Vision-Language-Action** (LLMs for robotics)
- 📚 **Everything** (I want a balanced overview)

**Profile Key:** `focus`

**Values:** `simulation`, `ros2`, `isaac`, `humanoid`, `vla`, `balanced`

---

### Question 4: Time Commitment (Optional)

**Question Text:**
> How much time can you dedicate to this course per week?

**Options:**
- ⏰ **Part-time** (5-10 hours/week)
- 📅 **Regular** (10-20 hours/week)
- 🚀 **Intensive** (20+ hours/week, full-time study)

**Profile Key:** `time_commitment`

**Values:** `part_time`, `regular`, `intensive`

---

### Question 5: Learning Style (Optional)

**Question Text:**
> How do you prefer to learn?

**Options:**
- 🛠️ **Hands-on**: Jump into projects and learn by doing
- 📖 **Theory-first**: Understand concepts before practicing
- ⚖️ **Balanced**: Mix of theory and practice

**Profile Key:** `learning_style`

**Values:** `hands_on`, `theory_first`, `balanced`

---

### Question 6: Language Preference

**Question Text:**
> What's your preferred language for learning?

**Options:**
- 🇬🇧 **English**
- 🇵🇰 **اردو (Urdu)**

**Profile Key:** `language`

**Values:** `en`, `ur`

---

## Advanced Hardware Questions (Conditional)

If user selected "Mid-range GPU" or "High-end GPU", ask these follow-up questions:

### Advanced Q1: GPU Model

**Question Text:**
> Which GPU do you have? (Optional - helps us optimize labs)

**Input Type:** Text field with autocomplete

**Common Options:**
- NVIDIA RTX 4090
- NVIDIA RTX 4080
- NVIDIA RTX 4070 Ti
- NVIDIA RTX 3090
- NVIDIA RTX 3080
- NVIDIA RTX 3070
- NVIDIA GTX 1660
- Other (specify)

**Profile Key:** `gpu_model`

---

### Advanced Q2: RAM

**Question Text:**
> How much RAM does your system have?

**Options:**
- 16 GB
- 32 GB
- 64 GB
- 128 GB or more

**Profile Key:** `ram_gb`

**Values:** `16`, `32`, `64`, `128`

---

### Advanced Q3: Jetson Availability

**Question Text:**
> Do you have access to NVIDIA Jetson hardware?

**Options:**
- ✅ Yes (Jetson Orin, Nano, etc.)
- ❌ No
- 🤔 Planning to get one

**Profile Key:** `has_jetson`

**Values:** `true`, `false`, `planned`

---

## Profile Summary

After all questions, show a summary:

```
📋 Your Learning Profile

Experience: Beginner
Hardware: Mid-range GPU
Focus: ROS 2 Development
Language: English

You'll receive:
✓ Detailed explanations for complex concepts
✓ Labs optimized for mid-range hardware
✓ Extra focus on ROS 2 topics
✓ Content in English

[Start Learning] [Change Preferences]
```

## Implementation Notes

### Storage

Store profile in user database:

```typescript
interface UserProfile {
  user_id: string;
  experience_level: "beginner" | "intermediate" | "advanced";
  hardware_tier: "low" | "mid" | "high";
  language: "en" | "ur";
  
  // Optional fields
  focus?: "simulation" | "ros2" | "isaac" | "humanoid" | "vla" | "balanced";
  time_commitment?: "part_time" | "regular" | "intensive";
  learning_style?: "hands_on" | "theory_first" | "balanced";
  
  // Advanced hardware
  gpu_available?: boolean;
  gpu_model?: string;
  ram_gb?: number;
  has_jetson?: boolean;
  
  // Metadata
  created_at: Date;
  updated_at: Date;
}
```

### Validation

- Ensure required fields are present
- Validate enum values
- Store timestamp for profile updates

### Progressive Disclosure

Don't overwhelm users:
1. Start with essential questions (1, 2, 6)
2. Show optional questions as expandable sections
3. Allow "Skip" option for optional questions
4. Show progress indicator (Question 1 of 3)

### Privacy

- Explain why you're collecting this data
- Allow users to skip optional questions
- Provide option to update/delete profile later
- Don't collect personally identifiable information

## Testing Scenarios

Test these user personas:

1. **Complete Beginner**
   - experience_level: beginner
   - hardware_tier: low
   - language: en
   - Expected: Maximum guidance, Gazebo-only labs

2. **Experienced Researcher**
   - experience_level: advanced
   - hardware_tier: high
   - language: en
   - Expected: Concise explanations, full Isaac Sim labs

3. **Urdu Speaker**
   - experience_level: intermediate
   - hardware_tier: mid
   - language: ur
   - Expected: Translated content with English code

4. **Cloud User**
   - experience_level: intermediate
   - hardware_tier: mid
   - language: en
   - Expected: Cloud alternative suggestions