#!/usr/bin/env python3
"""
Directory Structure Generator

Creates Docusaurus docs directory structure from course outline JSON.

Usage:
    generate_structure.py <outline.json> <docs_dir>

Example:
    generate_structure.py outline.json docs/
"""

import json
import sys
from pathlib import Path
from typing import Dict, Any


def create_category_json(label: str, position: int, description: str = None) -> str:
    """Generate _category_.json content."""
    category = {
        "label": label,
        "position": position
    }
    
    if description:
        category["link"] = {
            "type": "generated-index",
            "description": description
        }
    
    return json.dumps(category, indent=2)


def generate_structure(outline: Dict[str, Any], docs_dir: Path):
    """
    Generate complete directory structure with _category_.json files.
    
    Structure:
    docs/
    ├── module-1-slug/
    │   ├── _category_.json
    │   ├── week-1-2/
    │   │   └── _category_.json
    │   └── week-3-5/
    │       └── _category_.json
    └── module-2-slug/
        └── ...
    """
    docs_dir = Path(docs_dir)
    docs_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"🚀 Generating structure in: {docs_dir}")
    
    # Create root index if it doesn't exist
    index_path = docs_dir / 'index.mdx'
    if not index_path.exists():
        index_content = f"""---
title: {outline.get('title', 'Course Overview')}
sidebar_position: 1
---

# {outline.get('title', 'Course Overview')}

Welcome to the course!

## Course Structure

This course is organized into modules and weeks to guide your learning journey.
"""
        index_path.write_text(index_content)
        print(f"✅ Created: index.mdx")
    
    # Process each module
    for module in outline.get('modules', []):
        module_slug = f"module-{module['number']}-{module['slug']}"
        module_dir = docs_dir / module_slug
        module_dir.mkdir(parents=True, exist_ok=True)
        
        # Create module _category_.json
        category_path = module_dir / '_category_.json'
        category_content = create_category_json(
            label=f"Module {module['number']}: {module['title']}",
            position=module['number'],
            description=f"Learn about {module['title'].lower()}."
        )
        category_path.write_text(category_content)
        print(f"✅ Created: {module_slug}/_category_.json")
        
        # Process each week in the module
        for week_idx, week in enumerate(module.get('weeks', []), 1):
            week_dir = module_dir / week['slug']
            week_dir.mkdir(parents=True, exist_ok=True)
            
            # Create week _category_.json
            week_category_path = week_dir / '_category_.json'
            week_label = f"Weeks {week['start']}-{week['end']}: {week['topic']}" if week['start'] != week['end'] else f"Week {week['start']}: {week['topic']}"
            week_category_content = create_category_json(
                label=week_label,
                position=week_idx,
                description=week['topic']
            )
            week_category_path.write_text(week_category_content)
            print(f"✅ Created: {module_slug}/{week['slug']}/_category_.json")
    
    print(f"\n✅ Structure generation complete!")


def main():
    if len(sys.argv) != 3:
        print("Usage: generate_structure.py <outline.json> <docs_dir>")
        sys.exit(1)
    
    outline_file = Path(sys.argv[1])
    docs_dir = Path(sys.argv[2])
    
    try:
        # Load outline
        outline = json.loads(outline_file.read_text())
        
        # Generate structure
        generate_structure(outline, docs_dir)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()