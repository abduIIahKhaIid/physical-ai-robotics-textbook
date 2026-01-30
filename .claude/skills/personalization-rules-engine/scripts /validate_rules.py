#!/usr/bin/env python3
"""
Validate personalization rules configuration.
Checks for common issues like broken links, conflicting rules, and invalid transformations.
"""

import json
import sys
from pathlib import Path
from typing import Dict, List, Set, Tuple
import re


class RulesValidator:
    """Validator for personalization rules."""
    
    VALID_TRANSFORMATIONS = {
        'expand_explanations',
        'add_glossary_hints',
        'hide_advanced_blocks',
        'simplify_language',
        'swap_lab_variant',
        'add_cloud_alternative',
        'insert_prereq_links',
        'add_recommended_next',
        'reorder_optional_reading',
        'translate_content',
        'preserve_code_blocks',
        'add_bilingual_glossary',
        'highlight_sections',
    }
    
    VALID_PROFILE_KEYS = {
        'experience_level',
        'hardware_tier',
        'focus',
        'language',
        'gpu_available',
        'gpu_model',
        'ram_gb',
        'has_jetson',
        'learning_style',
        'time_commitment',
    }
    
    def __init__(self, config_path: str):
        """Initialize validator with config file."""
        self.config_path = config_path
        self.errors: List[str] = []
        self.warnings: List[str] = []
        
        try:
            with open(config_path, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            self.errors.append(f"Config file not found: {config_path}")
            self.config = {}
        except json.JSONDecodeError as e:
            self.errors.append(f"Invalid JSON: {e}")
            self.config = {}
    
    def validate(self) -> bool:
        """Run all validation checks. Returns True if valid."""
        if self.errors:
            return False
        
        self._validate_structure()
        self._validate_rules()
        self._check_conflicts()
        self._check_priorities()
        
        return len(self.errors) == 0
    
    def _validate_structure(self):
        """Validate top-level config structure."""
        if not isinstance(self.config, dict):
            self.errors.append("Config must be a JSON object")
            return
        
        if 'rules' not in self.config:
            self.errors.append("Config must have 'rules' array")
            return
        
        if not isinstance(self.config['rules'], list):
            self.errors.append("'rules' must be an array")
    
    def _validate_rules(self):
        """Validate individual rules."""
        rules = self.config.get('rules', [])
        
        for idx, rule in enumerate(rules):
            self._validate_rule(rule, idx)
    
    def _validate_rule(self, rule: Dict, idx: int):
        """Validate a single rule."""
        rule_id = rule.get('id', f'rule_{idx}')
        
        # Check required fields
        if 'id' not in rule:
            self.warnings.append(f"Rule {idx} missing 'id' field")
        
        if 'condition' not in rule:
            self.errors.append(f"Rule '{rule_id}' missing 'condition' field")
            return
        
        if 'transformations' not in rule:
            self.errors.append(f"Rule '{rule_id}' missing 'transformations' field")
            return
        
        # Validate condition
        condition = rule['condition']
        if not isinstance(condition, dict):
            self.errors.append(f"Rule '{rule_id}' condition must be an object")
        else:
            for key in condition.keys():
                if key not in self.VALID_PROFILE_KEYS:
                    self.warnings.append(f"Rule '{rule_id}' uses unknown profile key: {key}")
        
        # Validate transformations
        transformations = rule['transformations']
        if not isinstance(transformations, list):
            self.errors.append(f"Rule '{rule_id}' transformations must be an array")
        else:
            for transform in transformations:
                self._validate_transformation(transform, rule_id)
        
        # Validate priority
        if 'priority' in rule:
            if not isinstance(rule['priority'], (int, float)):
                self.errors.append(f"Rule '{rule_id}' priority must be a number")
    
    def _validate_transformation(self, transform, rule_id: str):
        """Validate a transformation specification."""
        if isinstance(transform, str):
            # Simple transformation name
            if transform not in self.VALID_TRANSFORMATIONS:
                self.errors.append(f"Rule '{rule_id}' uses unknown transformation: {transform}")
        
        elif isinstance(transform, dict):
            # Transformation with arguments
            if 'type' not in transform:
                self.errors.append(f"Rule '{rule_id}' transformation object missing 'type' field")
                return
            
            transform_type = transform['type']
            if transform_type not in self.VALID_TRANSFORMATIONS:
                self.errors.append(f"Rule '{rule_id}' uses unknown transformation type: {transform_type}")
        
        else:
            self.errors.append(f"Rule '{rule_id}' transformation must be string or object")
    
    def _check_conflicts(self):
        """Check for conflicting rules."""
        rules = self.config.get('rules', [])
        
        # Group rules by condition
        condition_groups: Dict[str, List[str]] = {}
        
        for rule in rules:
            condition = rule.get('condition', {})
            condition_key = json.dumps(condition, sort_keys=True)
            
            rule_id = rule.get('id', 'unknown')
            if condition_key in condition_groups:
                condition_groups[condition_key].append(rule_id)
            else:
                condition_groups[condition_key] = [rule_id]
        
        # Warn about duplicate conditions
        for condition_key, rule_ids in condition_groups.items():
            if len(rule_ids) > 1:
                self.warnings.append(f"Multiple rules with same condition: {', '.join(rule_ids)}")
    
    def _check_priorities(self):
        """Check priority assignments."""
        rules = self.config.get('rules', [])
        
        priorities = [rule.get('priority', 0) for rule in rules]
        
        # Warn if all priorities are the same
        if len(set(priorities)) == 1 and len(rules) > 1:
            self.warnings.append("All rules have same priority - execution order may be unpredictable")
        
        # Warn if priorities are missing
        missing_priority = [rule.get('id', 'unknown') for rule in rules if 'priority' not in rule]
        if missing_priority:
            self.warnings.append(f"Rules missing priority: {', '.join(missing_priority)}")
    
    def print_results(self):
        """Print validation results."""
        if not self.errors and not self.warnings:
            print("✅ All checks passed!")
            return
        
        if self.errors:
            print(f"\n❌ {len(self.errors)} Error(s) Found:")
            for error in self.errors:
                print(f"  - {error}")
        
        if self.warnings:
            print(f"\n⚠️  {len(self.warnings)} Warning(s):")
            for warning in self.warnings:
                print(f"  - {warning}")


def validate_links_in_content(content: str) -> Tuple[List[str], List[str]]:
    """
    Check for broken links in content.
    
    Returns:
        Tuple of (broken_links, warnings)
    """
    broken = []
    warnings = []
    
    # Find all markdown links
    link_pattern = r'\[([^\]]+)\]\(([^\)]+)\)'
    links = re.findall(link_pattern, content)
    
    for text, url in links:
        # Check for common issues
        if url.startswith('#') or url.startswith('../'):
            # Relative link - can't validate without full context
            warnings.append(f"Relative link (verify manually): {url}")
        elif url.startswith('http'):
            # External link - would need network check
            pass
        elif not url:
            broken.append(f"Empty link for text: {text}")
    
    return broken, warnings


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Validate personalization rules")
    parser.add_argument('config', help='Path to rules config JSON file')
    parser.add_argument('--check-links', action='store_true', help='Check for broken links in content')
    parser.add_argument('--strict', action='store_true', help='Treat warnings as errors')
    
    args = parser.parse_args()
    
    # Validate config
    validator = RulesValidator(args.config)
    is_valid = validator.validate()
    validator.print_results()
    
    # Check links if requested
    if args.check_links:
        print("\n📝 Checking links...")
        # This would check actual content files
        print("   (Link checking requires content files)")
    
    # Exit with appropriate code
    if not is_valid:
        sys.exit(1)
    
    if args.strict and validator.warnings:
        print("\n❌ Warnings treated as errors (--strict mode)")
        sys.exit(1)
    
    sys.exit(0)


if __name__ == "__main__":
    main()