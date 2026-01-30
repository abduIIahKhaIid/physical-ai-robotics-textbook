#!/usr/bin/env python3
"""
Test that personalization is deterministic.
Same profile + same content should always produce same output.
"""

import json
import hashlib
from typing import Dict, Any, List
from pathlib import Path
import sys


def test_determinism(
    content: str,
    profile: Dict[str, Any],
    rules_config: str,
    iterations: int = 10
) -> bool:
    """
    Test that applying rules produces consistent results.
    
    Args:
        content: Sample content to personalize
        profile: User profile
        rules_config: Path to rules config
        iterations: Number of times to apply rules
    
    Returns:
        True if all iterations produce identical output
    """
    # Import the apply_rules function
    sys.path.insert(0, str(Path(__file__).parent))
    from apply_rules import apply_personalization_rules
    
    results = []
    hashes = []
    
    print(f"Running determinism test ({iterations} iterations)...")
    
    for i in range(iterations):
        result = apply_personalization_rules(content, profile, rules_config)
        results.append(result)
        
        # Hash the result for comparison
        result_hash = hashlib.sha256(result.encode()).hexdigest()
        hashes.append(result_hash)
        
        print(f"  Iteration {i+1}: {result_hash[:16]}...")
    
    # Check if all hashes are identical
    unique_hashes = set(hashes)
    
    if len(unique_hashes) == 1:
        print("\n✅ PASS: All iterations produced identical output")
        return True
    else:
        print(f"\n❌ FAIL: Found {len(unique_hashes)} different outputs")
        print("\nDifferences:")
        
        for idx, (hash1, hash2) in enumerate(zip(hashes[:-1], hashes[1:])):
            if hash1 != hash2:
                print(f"  Iteration {idx+1} != Iteration {idx+2}")
                print(f"    Hash 1: {hash1[:32]}...")
                print(f"    Hash 2: {hash2[:32]}...")
        
        return False


def test_idempotency(
    content: str,
    profile: Dict[str, Any],
    rules_config: str
) -> bool:
    """
    Test that applying rules twice produces the same result as applying once.
    
    Returns:
        True if the transformation is idempotent
    """
    sys.path.insert(0, str(Path(__file__).parent))
    from apply_rules import apply_personalization_rules
    
    print("\nRunning idempotency test...")
    
    # Apply once
    result1 = apply_personalization_rules(content, profile, rules_config)
    hash1 = hashlib.sha256(result1.encode()).hexdigest()
    
    # Apply to the result again
    result2 = apply_personalization_rules(result1, profile, rules_config)
    hash2 = hashlib.sha256(result2.encode()).hexdigest()
    
    print(f"  First application:  {hash1[:16]}...")
    print(f"  Second application: {hash2[:16]}...")
    
    if hash1 == hash2:
        print("\n✅ PASS: Transformation is idempotent")
        return True
    else:
        print("\n⚠️  WARNING: Transformation is NOT idempotent")
        print("  (Applying rules twice produces different output)")
        return False


def test_profile_isolation(
    content: str,
    profiles: List[Dict[str, Any]],
    rules_config: str
) -> bool:
    """
    Test that different profiles produce different outputs.
    
    Returns:
        True if profiles are properly isolated
    """
    sys.path.insert(0, str(Path(__file__).parent))
    from apply_rules import apply_personalization_rules
    
    print("\nRunning profile isolation test...")
    
    results = {}
    
    for profile in profiles:
        profile_key = json.dumps(profile, sort_keys=True)
        result = apply_personalization_rules(content, profile, rules_config)
        result_hash = hashlib.sha256(result.encode()).hexdigest()
        results[profile_key] = result_hash
        
        level = profile.get('experience_level', 'unknown')
        hw = profile.get('hardware_tier', 'unknown')
        print(f"  Profile {level}/{hw}: {result_hash[:16]}...")
    
    # Check if we got different outputs for different profiles
    unique_outputs = set(results.values())
    
    if len(unique_outputs) == len(profiles):
        print(f"\n✅ PASS: All {len(profiles)} profiles produced unique outputs")
        return True
    elif len(unique_outputs) == 1:
        print(f"\n⚠️  WARNING: All profiles produced identical output")
        print("  (Personalization may not be working)")
        return False
    else:
        print(f"\n⚠️  WARNING: Expected {len(profiles)} unique outputs, got {len(unique_outputs)}")
        return False


def run_test_suite():
    """Run complete test suite."""
    # Sample content
    sample_content = """
# Introduction to ROS 2

ROS 2 is the Robot Operating System. It uses VSLAM for navigation.

### Advanced: Performance

For production, use Nav2 with hardware acceleration.
"""
    
    # Create test config
    import tempfile
    import os
    
    config = {
        "rules": [
            {
                "id": "beginner_expand",
                "condition": {"experience_level": "beginner"},
                "transformations": ["expand_explanations", "add_glossary_hints"],
                "priority": 10
            },
            {
                "id": "advanced_concise",
                "condition": {"experience_level": "advanced"},
                "transformations": ["hide_advanced_blocks"],
                "priority": 10
            },
            {
                "id": "low_hardware",
                "condition": {"hardware_tier": "low"},
                "transformations": [{"type": "swap_lab_variant", "variant": "low_gpu"}],
                "priority": 20
            }
        ]
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        json.dump(config, f)
        config_path = f.name
    
    try:
        # Test profiles
        beginner_profile = {
            "experience_level": "beginner",
            "hardware_tier": "low",
            "language": "en"
        }
        
        advanced_profile = {
            "experience_level": "advanced",
            "hardware_tier": "high",
            "language": "en"
        }
        
        # Run tests
        print("=" * 60)
        print("PERSONALIZATION DETERMINISM TEST SUITE")
        print("=" * 60)
        
        test1 = test_determinism(sample_content, beginner_profile, config_path, iterations=5)
        test2 = test_idempotency(sample_content, beginner_profile, config_path)
        test3 = test_profile_isolation(sample_content, [beginner_profile, advanced_profile], config_path)
        
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        print(f"  Determinism:       {'✅ PASS' if test1 else '❌ FAIL'}")
        print(f"  Idempotency:       {'✅ PASS' if test2 else '⚠️  WARNING'}")
        print(f"  Profile Isolation: {'✅ PASS' if test3 else '⚠️  WARNING'}")
        print("=" * 60)
        
        # Overall result
        if test1 and test2 and test3:
            print("\n🎉 All tests passed!")
            return 0
        elif test1:
            print("\n✅ Core determinism test passed (warnings acceptable)")
            return 0
        else:
            print("\n❌ Critical test failed")
            return 1
        
    finally:
        os.unlink(config_path)


if __name__ == "__main__":
    exit_code = run_test_suite()
    sys.exit(exit_code)
    