#!/usr/bin/env python3
"""
Core rule application engine for personalization.
Applies transformation primitives based on user profile and rule configuration.
"""

import json
from typing import Dict, List, Any, Callable
from pathlib import Path
import re


class PersonalizationEngine:
    """Main engine for applying personalization rules."""
    
    def __init__(self, rules_config_path: str):
        """Initialize with rules configuration file."""
        with open(rules_config_path, 'r') as f:
            self.config = json.load(f)
        
        # Map transformation names to functions
        self.transformations = {
            'expand_explanations': self._expand_explanations,
            'add_glossary_hints': self._add_glossary_hints,
            'hide_advanced_blocks': self._hide_advanced_blocks,
            'simplify_language': self._simplify_language,
            'swap_lab_variant': self._swap_lab_variant,
            'add_cloud_alternative': self._add_cloud_alternative,
            'insert_prereq_links': self._insert_prereq_links,
            'add_recommended_next': self._add_recommended_next,
            'reorder_optional_reading': self._reorder_optional_reading,
            'translate_content': self._translate_content,
            'preserve_code_blocks': self._preserve_code_blocks,
            'add_bilingual_glossary': self._add_bilingual_glossary,
            'highlight_sections': self._highlight_sections,
        }
    
    def apply_rules(self, content: str, profile: Dict[str, Any]) -> str:
        """
        Apply personalization rules to content based on user profile.
        
        Args:
            content: Original chapter content (Markdown)
            profile: User profile dict with keys like experience_level, hardware_tier, etc.
        
        Returns:
            Transformed content
        """
        # Get matching rules sorted by priority (highest first)
        matching_rules = self._get_matching_rules(profile)
        matching_rules.sort(key=lambda r: r.get('priority', 0), reverse=True)
        
        # Apply each transformation
        result = content
        applied_transformations = []
        
        for rule in matching_rules:
            for transform in rule['transformations']:
                if isinstance(transform, str):
                    # Simple transformation name
                    transform_name = transform
                    transform_args = {}
                elif isinstance(transform, dict):
                    # Transformation with arguments
                    transform_name = transform['type']
                    transform_args = {k: v for k, v in transform.items() if k != 'type'}
                else:
                    continue
                
                if transform_name in self.transformations:
                    result = self.transformations[transform_name](result, profile, **transform_args)
                    applied_transformations.append(transform_name)
        
        # Add metadata about applied transformations
        metadata = self._create_metadata(profile, applied_transformations)
        result = metadata + "\n\n" + result
        
        return result
    
    def _get_matching_rules(self, profile: Dict[str, Any]) -> List[Dict]:
        """Get rules that match the user profile."""
        matching = []
        
        for rule in self.config.get('rules', []):
            condition = rule.get('condition', {})
            
            # Check if all conditions match
            if all(profile.get(key) == value for key, value in condition.items()):
                matching.append(rule)
        
        return matching
    
    def _create_metadata(self, profile: Dict[str, Any], transformations: List[str]) -> str:
        """Create metadata banner for personalized content."""
        level = profile.get('experience_level', 'default')
        hardware = profile.get('hardware_tier', 'standard')
        lang = profile.get('language', 'en')
        
        metadata = f""":::info Personalized Content
This chapter has been customized for:
- **Experience Level**: {level.title()}
- **Hardware**: {hardware.upper()} tier
- **Language**: {lang.upper()}

[View Original Content](#) | [Update Preferences](#)
:::
"""
        return metadata
    
    # ===== Transformation Primitives =====
    
    def _expand_explanations(self, content: str, profile: Dict, **kwargs) -> str:
        """Add detailed explanations for complex concepts."""
        # Find technical terms and add expandable details
        # This is a simplified example - real implementation would use a glossary
        
        expansions = {
            'ROS 2': '<details><summary>ROS 2</summary>Robot Operating System 2 is a flexible framework for writing robot software, providing tools and libraries for building robot applications.</details>',
            'URDF': '<details><summary>URDF</summary>Unified Robot Description Format - an XML format for representing a robot model, including its physical structure, sensors, and actuators.</details>',
            'Isaac Sim': '<details><summary>Isaac Sim</summary>NVIDIA Isaac Sim is a robotics simulation platform built on Omniverse, providing photorealistic, physically accurate environments for robot development.</details>',
        }
        
        result = content
        for term, expansion in expansions.items():
            # Only expand first occurrence to avoid clutter
            result = result.replace(term, expansion, 1)
        
        return result
    
    def _add_glossary_hints(self, content: str, profile: Dict, **kwargs) -> str:
        """Insert inline glossary tooltips."""
        # Add hover tooltips for technical terms
        glossary = {
            'VSLAM': 'Visual Simultaneous Localization and Mapping',
            'IMU': 'Inertial Measurement Unit',
            'LiDAR': 'Light Detection and Ranging',
            'Nav2': 'Navigation 2 - ROS 2 navigation stack',
        }
        
        result = content
        for term, definition in glossary.items():
            # Wrap term with tooltip syntax (Docusaurus format)
            tooltip = f'<abbr title="{definition}">{term}</abbr>'
            result = re.sub(rf'\b{term}\b', tooltip, result, count=1)
        
        return result
    
    def _hide_advanced_blocks(self, content: str, profile: Dict, **kwargs) -> str:
        """Collapse advanced/optional sections for beginners."""
        # Wrap advanced sections in collapsible details
        # Look for sections marked with "Advanced:" or similar
        
        pattern = r'(### Advanced:.*?(?=###|$))'
        
        def wrap_advanced(match):
            section = match.group(1)
            return f'<details>\n<summary>Advanced Topic (Optional)</summary>\n\n{section}\n</details>'
        
        result = re.sub(pattern, wrap_advanced, content, flags=re.DOTALL)
        return result
    
    def _simplify_language(self, content: str, profile: Dict, **kwargs) -> str:
        """Use simpler terminology for beginners."""
        # Replace complex terms with simpler equivalents
        simplifications = {
            'kinematic chain': 'connected robot joints',
            'degrees of freedom': 'ways the robot can move',
            'end effector': 'robot hand or tool',
            'trajectory planning': 'path planning',
        }
        
        result = content
        for complex_term, simple_term in simplifications.items():
            result = re.sub(rf'\b{complex_term}\b', simple_term, result, flags=re.IGNORECASE)
        
        return result
    
    def _swap_lab_variant(self, content: str, profile: Dict, variant: str = None, **kwargs) -> str:
        """Replace lab instructions with variant-specific version."""
        # This would load alternative lab content from assets/lab_variants/
        # For now, just add a note about the variant
        
        if variant:
            note = f"\n\n:::tip Hardware-Optimized Lab\nThis lab has been optimized for **{variant}** hardware.\n:::\n\n"
            # Insert after first heading
            result = re.sub(r'(^#.*?\n)', r'\1' + note, content, count=1, flags=re.MULTILINE)
            return result
        
        return content
    
    def _add_cloud_alternative(self, content: str, profile: Dict, **kwargs) -> str:
        """Show AWS/cloud alternatives for resource-intensive labs."""
        cloud_note = """
:::note Cloud Alternative
Don't have high-end hardware? You can run this lab on:
- **AWS RoboMaker**: Free tier available
- **Google Cloud Platform**: Compute Engine with GPU
- **NVIDIA Omniverse Cloud**: Isaac Sim streaming

[Learn more about cloud options →](#cloud-setup)
:::
"""
        # Add after first paragraph
        paragraphs = content.split('\n\n')
        if len(paragraphs) > 1:
            paragraphs.insert(1, cloud_note)
            return '\n\n'.join(paragraphs)
        
        return content
    
    def _insert_prereq_links(self, content: str, profile: Dict, **kwargs) -> str:
        """Add 'Review this first' links for beginners."""
        prereq = """
:::caution Prerequisites
Before starting this chapter, make sure you've completed:
- [Introduction to Physical AI](../intro/physical-ai)
- [ROS 2 Basics](../ros2/basics)
:::
"""
        # Insert at beginning after title
        result = re.sub(r'(^#.*?\n\n)', r'\1' + prereq, content, count=1, flags=re.MULTILINE)
        return result
    
    def _add_recommended_next(self, content: str, profile: Dict, **kwargs) -> str:
        """Suggest next lesson based on focus area."""
        focus = profile.get('focus', 'simulation')
        
        recommendations = {
            'simulation': 'Gazebo Advanced Features',
            'ros2': 'ROS 2 Actions and Services',
            'isaac': 'Isaac Sim Perception Pipeline',
            'humanoid': 'Bipedal Locomotion Control',
            'vla': 'Vision-Language-Action Models',
        }
        
        next_topic = recommendations.get(focus, 'Next Chapter')
        
        footer = f"""
---

### What's Next?

Based on your focus area (**{focus}**), we recommend:
👉 [{next_topic}](../next-chapter)
"""
        return content + footer
    
    def _reorder_optional_reading(self, content: str, profile: Dict, prioritize: str = None, **kwargs) -> str:
        """Prioritize readings by focus area."""
        # This would reorder "Further Reading" sections
        # Simplified implementation: just add a note
        if prioritize:
            note = f"\n\n📚 *Recommended for {prioritize} learners*\n\n"
            result = re.sub(r'(## Further Reading)', r'\1' + note, content)
            return result
        return content
    
    def _translate_content(self, content: str, profile: Dict, **kwargs) -> str:
        """Apply Urdu translation."""
        # This would use a translation service or pre-translated content
        # For now, just add a note
        if profile.get('language') == 'ur':
            note = "<!-- Content translated to Urdu -->\n"
            return note + content
        return content
    
    def _preserve_code_blocks(self, content: str, profile: Dict, **kwargs) -> str:
        """Keep code/commands in English even when translating."""
        # Mark code blocks to not be translated
        # This is handled by the translation service configuration
        return content
    
    def _add_bilingual_glossary(self, content: str, profile: Dict, **kwargs) -> str:
        """Include English terms in parentheses for Urdu content."""
        if profile.get('language') == 'ur':
            # Add glossary section
            glossary = """
## اصطلاحات (Glossary)

- **ROS 2** (Robot Operating System 2)
- **Gazebo** (Robot Simulator)
- **Isaac Sim** (NVIDIA Simulation Platform)
"""
            return content + "\n\n" + glossary
        return content
    
    def _highlight_sections(self, content: str, profile: Dict, sections: List[str] = None, **kwargs) -> str:
        """Highlight specific sections based on focus."""
        if not sections:
            return content
        
        # Add visual highlighting to matching sections
        for section in sections:
            pattern = rf'(## .*{section}.*)'
            replacement = r'## 🎯 \1'
            content = re.sub(pattern, replacement, content, flags=re.IGNORECASE)
        
        return content


def apply_personalization_rules(
    chapter_content: str,
    user_profile: Dict[str, Any],
    rules_config: str = "config/personalization_rules.json"
) -> str:
    """
    Main entry point for applying personalization rules.
    
    Args:
        chapter_content: Original Markdown content
        user_profile: User profile dictionary
        rules_config: Path to rules configuration JSON
    
    Returns:
        Personalized content
    """
    engine = PersonalizationEngine(rules_config)
    return engine.apply_rules(chapter_content, user_profile)


if __name__ == "__main__":
    # Example usage
    sample_content = """
# Introduction to ROS 2

ROS 2 is the next generation of the Robot Operating System. It uses VSLAM for navigation
and works with various sensors including LiDAR and IMU.

### Advanced: Performance Optimization

For production robots, consider using Nav2 with hardware acceleration.
"""
    
    sample_profile = {
        "experience_level": "beginner",
        "hardware_tier": "low",
        "language": "en"
    }
    
    # Create a minimal config for testing
    import tempfile
    import os
    
    config = {
        "rules": [
            {
                "id": "beginner_expand",
                "condition": {"experience_level": "beginner"},
                "transformations": ["expand_explanations", "add_glossary_hints", "hide_advanced_blocks"],
                "priority": 10
            }
        ]
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        config_path = f.name
    
    try:
        result = apply_personalization_rules(sample_content, sample_profile, config_path)
        print("=== Personalized Content ===")
        print(result)
    finally:
        os.unlink(config_path)