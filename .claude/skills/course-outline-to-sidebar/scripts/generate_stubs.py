#!/usr/bin/env python3
"""
MDX Stub File Generator

Creates placeholder MDX files with proper frontmatter from course outline.

Usage:
    generate_stubs.py <outline.json> <docs_dir>

Example:
    generate_stubs.py outline.json docs/
"""

import json
import sys
from pathlib import Path
from typing import Dict, Any


def generate_mdx_stub(title: str, position: int, additional_frontmatter: Dict[str, Any] = None) -> str:
    """
    Generate MDX stub content with frontmatter.
    
    Args:
        title: Chapter title
        position: Sidebar position number
        additional_frontmatter: Extra frontmatter fields
    
    Returns:
        Complete MDX file content
    """
    frontmatter = {
        'title': title,
        'sidebar_position': position
    }
    
    if additional_frontmatter:
        frontmatter.update(additional_frontmatter)
    
    # Build frontmatter string
    fm_lines = ['---']
    for key, value in frontmatter.items():
        if isinstance(value, str):
            fm_lines.append(f"{key}: {value}")
        elif isinstance(value, list):
            fm_lines.append(f"{key}:")
            for item in value:
                fm_lines.append(f"  - {item}")
        else:
            fm_lines.append(f"{key}: {value}")
    fm_lines.append('---')
    
    mdx_content = '\n'.join(fm_lines) + f"""

# {title}

[Content to be written]

## Learning Objectives

After completing this chapter, you will be able to:

- Objective 1
- Objective 2
- Objective 3

## Overview

[Brief introduction to the topic]

## Key Concepts

### Concept 1

[Explanation]

### Concept 2

[Explanation]

## Hands-on Activities

### Activity 1

[Step-by-step instructions]

## Summary

[Recap of key points]

## Additional Resources

- [Resource 1](#)
- [Resource 2](#)

## Next Steps

Continue to the next chapter to learn about...
"""
    
    return mdx_content


def generate_stubs(outline: Dict[str, Any], docs_dir: Path):
    """
    Generate all MDX stub files based on outline.
    
    Creates files only if they don't already exist to avoid overwriting.
    """
    docs_dir = Path(docs_dir)
    
    print(f"🚀 Generating MDX stubs in: {docs_dir}")
    
    created_count = 0
    skipped_count = 0
    
    # Process each module
    for module in outline.get('modules', []):
        module_slug = f"module-{module['number']}-{module['slug']}"
        module_dir = docs_dir / module_slug
        
        # Process each week
        for week in module.get('weeks', []):
            week_slug = week['slug']
            week_dir = module_dir / week_slug
            week_dir.mkdir(parents=True, exist_ok=True)
            
            # Process each chapter
            for idx, chapter in enumerate(week.get('chapters', []), 1):
                chapter_filename = f"{idx}-{chapter['slug']}.mdx"
                chapter_path = week_dir / chapter_filename
                
                if chapter_path.exists():
                    print(f"⏭️  Skipped (exists): {module_slug}/{week_slug}/{chapter_filename}")
                    skipped_count += 1
                    continue
                
                # Generate stub content
                stub_content = generate_mdx_stub(
                    title=chapter['title'],
                    position=idx
                )
                
                # Write file
                chapter_path.write_text(stub_content)
                print(f"✅ Created: {module_slug}/{week_slug}/{chapter_filename}")
                created_count += 1
    
    print(f"\n✅ Stub generation complete!")
    print(f"   Created: {created_count} files")
    print(f"   Skipped: {skipped_count} files (already exist)")


def main():
    if len(sys.argv) != 3:
        print("Usage: generate_stubs.py <outline.json> <docs_dir>")
        sys.exit(1)
    
    outline_file = Path(sys.argv[1])
    docs_dir = Path(sys.argv[2])
    
    try:
        # Load outline
        outline = json.loads(outline_file.read_text())
        
        # Generate stubs
        generate_stubs(outline, docs_dir)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()