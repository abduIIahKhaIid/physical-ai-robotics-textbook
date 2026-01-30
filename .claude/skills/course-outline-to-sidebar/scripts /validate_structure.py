#!/usr/bin/env python3
"""
Structure Validator

Validates Docusaurus docs structure and sidebar configuration.

Usage:
    validate_structure.py <docs_dir> [sidebars.ts]

Example:
    validate_structure.py docs/ sidebars.ts
"""

import re
import sys
import json
from pathlib import Path
from typing import List, Set, Dict, Any


class ValidationError:
    """Represents a validation error."""
    def __init__(self, level: str, message: str, location: str = None):
        self.level = level  # 'ERROR' or 'WARNING'
        self.message = message
        self.location = location
    
    def __str__(self):
        loc = f" [{self.location}]" if self.location else ""
        return f"{self.level}{loc}: {self.message}"


def extract_sidebar_refs(sidebars_ts: Path) -> Set[str]:
    """Extract all document references from sidebars.ts."""
    if not sidebars_ts or not sidebars_ts.exists():
        return set()
    
    content = sidebars_ts.read_text()
    refs = set()
    
    # Find all string literals that look like doc paths
    # Matches: 'module-1-name/week-1/1-chapter' or "module-1-name/week-1/1-chapter"
    pattern = r"['\"]([a-z0-9-]+(?:/[a-z0-9-]+)*)['\"]"
    for match in re.finditer(pattern, content):
        ref = match.group(1)
        # Filter out likely non-doc strings
        if '/' in ref or ref in ['index', 'introduction']:
            refs.add(ref)
    
    return refs


def find_mdx_files(docs_dir: Path) -> Dict[str, Path]:
    """Find all MDX files and their relative paths."""
    docs_dir = Path(docs_dir)
    mdx_files = {}
    
    for mdx_path in docs_dir.rglob('*.mdx'):
        # Get path relative to docs_dir
        rel_path = mdx_path.relative_to(docs_dir)
        # Remove .mdx extension for doc ID
        doc_id = str(rel_path.with_suffix(''))
        mdx_files[doc_id] = mdx_path
    
    return mdx_files


def check_frontmatter(mdx_path: Path) -> List[ValidationError]:
    """Validate MDX frontmatter."""
    errors = []
    content = mdx_path.read_text()
    
    # Check for frontmatter
    if not content.startswith('---\n'):
        errors.append(ValidationError(
            'ERROR',
            'Missing frontmatter',
            str(mdx_path)
        ))
        return errors
    
    # Extract frontmatter
    try:
        end = content.index('---\n', 4)
        frontmatter = content[4:end]
    except ValueError:
        errors.append(ValidationError(
            'ERROR',
            'Malformed frontmatter (missing closing ---)',
            str(mdx_path)
        ))
        return errors
    
    # Check required fields
    if 'title:' not in frontmatter:
        errors.append(ValidationError(
            'ERROR',
            'Missing required field: title',
            str(mdx_path)
        ))
    
    if 'sidebar_position:' not in frontmatter:
        errors.append(ValidationError(
            'WARNING',
            'Missing sidebar_position (recommended)',
            str(mdx_path)
        ))
    
    return errors


def check_category_files(docs_dir: Path) -> List[ValidationError]:
    """Validate _category_.json files."""
    errors = []
    docs_dir = Path(docs_dir)
    
    # Find all directories that should have categories
    for dir_path in docs_dir.rglob('*'):
        if not dir_path.is_dir():
            continue
        if dir_path.name.startswith('.') or dir_path.name == 'node_modules':
            continue
        
        # Skip root docs directory
        if dir_path == docs_dir:
            continue
        
        # Check for _category_.json
        category_file = dir_path / '_category_.json'
        if not category_file.exists():
            errors.append(ValidationError(
                'WARNING',
                f'Missing _category_.json',
                str(dir_path)
            ))
            continue
        
        # Validate category file
        try:
            category_data = json.loads(category_file.read_text())
            if 'label' not in category_data:
                errors.append(ValidationError(
                    'ERROR',
                    'Missing required field: label',
                    str(category_file)
                ))
        except json.JSONDecodeError as e:
            errors.append(ValidationError(
                'ERROR',
                f'Invalid JSON: {e}',
                str(category_file)
            ))
    
    return errors


def validate_structure(docs_dir: Path, sidebars_ts: Path = None) -> List[ValidationError]:
    """Run all validation checks."""
    errors = []
    
    # Find all MDX files
    mdx_files = find_mdx_files(docs_dir)
    print(f"📄 Found {len(mdx_files)} MDX files")
    
    # Extract sidebar references
    sidebar_refs = extract_sidebar_refs(sidebars_ts) if sidebars_ts else set()
    if sidebar_refs:
        print(f"🔗 Found {len(sidebar_refs)} sidebar references")
    
    # Check 1: All sidebar refs point to existing files
    if sidebar_refs:
        for ref in sidebar_refs:
            if ref not in mdx_files:
                errors.append(ValidationError(
                    'ERROR',
                    f'Sidebar references non-existent file: {ref}.mdx',
                    'sidebars.ts'
                ))
    
    # Check 2: Find orphaned files (not in sidebar)
    if sidebar_refs:
        orphaned = set(mdx_files.keys()) - sidebar_refs
        for orphan in orphaned:
            if orphan not in ['index', 'introduction']:  # Skip special files
                errors.append(ValidationError(
                    'WARNING',
                    f'File not referenced in sidebar: {orphan}.mdx',
                    str(mdx_files[orphan])
                ))
    
    # Check 3: Validate frontmatter
    for doc_id, mdx_path in mdx_files.items():
        errors.extend(check_frontmatter(mdx_path))
    
    # Check 4: Validate category files
    errors.extend(check_category_files(docs_dir))
    
    return errors


def main():
    if len(sys.argv) < 2:
        print("Usage: validate_structure.py <docs_dir> [sidebars.ts]")
        sys.exit(1)
    
    docs_dir = Path(sys.argv[1])
    sidebars_ts = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    
    if not docs_dir.exists():
        print(f"❌ Error: docs_dir does not exist: {docs_dir}")
        sys.exit(1)
    
    print(f"🔍 Validating structure in: {docs_dir}")
    if sidebars_ts:
        print(f"🔍 Checking sidebar: {sidebars_ts}")
    print()
    
    # Run validation
    errors = validate_structure(docs_dir, sidebars_ts)
    
    # Report results
    error_count = sum(1 for e in errors if e.level == 'ERROR')
    warning_count = sum(1 for e in errors if e.level == 'WARNING')
    
    if errors:
        print("\n📋 Validation Results:\n")
        for error in errors:
            print(f"  {error}")
        
        print(f"\n{'❌' if error_count > 0 else '⚠️'} Found {error_count} errors and {warning_count} warnings")
        
        if error_count > 0:
            sys.exit(1)
    else:
        print("\n✅ No validation errors found!")
    
    sys.exit(0)


if __name__ == '__main__':
    main()