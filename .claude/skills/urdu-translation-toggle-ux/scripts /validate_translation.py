#!/usr/bin/env python3
"""
Validate Urdu Translation Implementation

This script checks that the translation feature is correctly implemented:
- Translation button exists on chapter pages
- RTL support is configured
- Caching mechanism is in place
- Translation service endpoint exists
"""

import os
import json
import re
from pathlib import Path


def validate_translation_button(docs_path):
    """Check if translation button is present in chapter templates"""
    issues = []
    
    # Check common template locations
    template_paths = [
        "src/theme/DocItem/Layout/index.tsx",
        "src/theme/DocItem/index.tsx",
        "src/components/TranslationToggle.tsx"
    ]
    
    found_button = False
    for template in template_paths:
        full_path = os.path.join(docs_path, template)
        if os.path.exists(full_path):
            with open(full_path, 'r') as f:
                content = f.read()
                if 'translate' in content.lower() and ('button' in content.lower() or 'onclick' in content.lower()):
                    found_button = True
                    print(f"✅ Translation button found in {template}")
                    break
    
    if not found_button:
        issues.append("❌ Translation button not found in chapter templates")
    
    return issues


def validate_rtl_support(docs_path):
    """Check if RTL styling is configured"""
    issues = []
    
    # Check for RTL CSS
    css_files = list(Path(docs_path).rglob("*.css"))
    rtl_found = False
    
    for css_file in css_files:
        with open(css_file, 'r') as f:
            content = f.read()
            if 'dir="rtl"' in content or '[dir="rtl"]' in content or 'direction: rtl' in content:
                rtl_found = True
                print(f"✅ RTL support found in {css_file.name}")
                break
    
    if not rtl_found:
        issues.append("❌ No RTL CSS styling found")
    
    return issues


def validate_api_endpoint(project_path):
    """Check if translation API endpoint exists"""
    issues = []
    
    # Check for API routes
    api_paths = [
        "api/translate.py",
        "api/translate/route.ts",
        "pages/api/translate.ts",
        "backend/translate.py"
    ]
    
    found_api = False
    for api_path in api_paths:
        full_path = os.path.join(project_path, api_path)
        if os.path.exists(full_path):
            found_api = True
            print(f"✅ Translation API found at {api_path}")
            break
    
    if not found_api:
        issues.append("❌ Translation API endpoint not found")
    
    return issues


def validate_caching(project_path):
    """Check if caching mechanism exists"""
    issues = []
    
    # Check for cache implementation
    cache_indicators = [
        "translation_cache",
        "translationCache",
        "cache_translation",
        "IndexedDB",
        "localStorage"
    ]
    
    found_cache = False
    for root, dirs, files in os.walk(project_path):
        for file in files:
            if file.endswith(('.ts', '.tsx', '.js', '.jsx', '.py')):
                file_path = os.path.join(root, file)
                try:
                    with open(file_path, 'r') as f:
                        content = f.read()
                        if any(indicator in content for indicator in cache_indicators):
                            found_cache = True
                            print(f"✅ Caching mechanism found in {file}")
                            break
                except:
                    pass
        if found_cache:
            break
    
    if not found_cache:
        issues.append("⚠️  No caching mechanism detected (optional but recommended)")
    
    return issues


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python validate_translation.py <project_path>")
        sys.exit(1)
    
    project_path = sys.argv[1]
    
    if not os.path.exists(project_path):
        print(f"❌ Project path does not exist: {project_path}")
        sys.exit(1)
    
    print("🔍 Validating Urdu Translation Implementation...\n")
    
    all_issues = []
    
    # Run validations
    all_issues.extend(validate_translation_button(project_path))
    all_issues.extend(validate_rtl_support(project_path))
    all_issues.extend(validate_api_endpoint(project_path))
    all_issues.extend(validate_caching(project_path))
    
    print("\n" + "="*50)
    
    if all_issues:
        print("\n⚠️  Issues found:\n")
        for issue in all_issues:
            print(issue)
        print("\n❌ Validation failed")
        sys.exit(1)
    else:
        print("\n✅ All validations passed!")
        sys.exit(0)


if __name__ == "__main__":
    main()