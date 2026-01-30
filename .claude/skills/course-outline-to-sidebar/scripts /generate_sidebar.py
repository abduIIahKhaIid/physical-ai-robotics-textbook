#!/usr/bin/env python3
"""
Sidebar Configuration Generator

Generates Docusaurus sidebars.ts from course outline JSON.

Usage:
    generate_sidebar.py <outline.json> [--output <sidebars.ts>]

Example:
    generate_sidebar.py outline.json > sidebars.ts
    generate_sidebar.py outline.json --output sidebars.ts
"""

import json
import sys
from pathlib import Path
from typing import Dict, Any, List


def generate_sidebar_items(outline: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Generate sidebar items from course outline.
    
    Returns list of sidebar items with proper nesting.
    """
    items = ['index']  # Start with index page
    
    # Add introduction if present
    if 'introduction' in outline:
        items.append('introduction')
    
    # Process each module
    for module in outline.get('modules', []):
        module_slug = f"module-{module['number']}-{module['slug']}"
        
        # Create week items for this module
        week_items = []
        for week in module.get('weeks', []):
            week_slug = week['slug']
            week_path = f"{module_slug}/{week_slug}"
            
            # Create chapter items for this week
            chapter_items = []
            for idx, chapter in enumerate(week.get('chapters', []), 1):
                chapter_path = f"{week_path}/{idx}-{chapter['slug']}"
                chapter_items.append(chapter_path)
            
            # Create week category
            week_start = week['start']
            week_end = week['end']
            week_label = f"Weeks {week_start}-{week_end}: {week['topic']}" if week_start != week_end else f"Week {week_start}: {week['topic']}"
            
            week_category = {
                'type': 'category',
                'label': week_label,
                'items': chapter_items
            }
            week_items.append(week_category)
        
        # Create module category
        module_item = {
            'type': 'category',
            'label': f"Module {module['number']}: {module['title']}",
            'link': {
                'type': 'generated-index',
                'description': f"Learn about {module['title'].lower()}."
            },
            'items': week_items
        }
        items.append(module_item)
    
    return items


def format_sidebar_ts(items: List[Any]) -> str:
    """
    Format sidebar items as TypeScript configuration.
    
    Converts Python data structures to valid TypeScript syntax.
    """
    def format_value(value, indent=0):
        """Recursively format values to TypeScript."""
        ind = '  ' * indent
        
        if isinstance(value, str):
            return f"'{value}'"
        elif isinstance(value, bool):
            return 'true' if value else 'false'
        elif isinstance(value, (int, float)):
            return str(value)
        elif isinstance(value, list):
            if not value:
                return '[]'
            items_str = ',\n'.join(f"{ind}  {format_value(item, indent + 1)}" for item in value)
            return f"[\n{items_str},\n{ind}]"
        elif isinstance(value, dict):
            if not value:
                return '{}'
            items_str = ',\n'.join(
                f"{ind}  {key}: {format_value(val, indent + 1)}"
                for key, val in value.items()
            )
            return f"{{\n{items_str},\n{ind}}}"
        return str(value)
    
    # Build the TypeScript file
    ts_content = """import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation
 
 The sidebars can be generated from the filesystem, or explicitly defined here.
 
 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Main tutorial sidebar
  tutorialSidebar: """
    
    ts_content += format_value(items, 2)
    ts_content += """,
};

export default sidebars;
"""
    
    return ts_content


def main():
    if len(sys.argv) < 2:
        print("Usage: generate_sidebar.py <outline.json> [--output <sidebars.ts>]")
        sys.exit(1)
    
    outline_file = Path(sys.argv[1])
    
    # Determine output
    if '--output' in sys.argv:
        output_idx = sys.argv.index('--output')
        output_file = Path(sys.argv[output_idx + 1])
        output_to_file = True
    else:
        output_file = None
        output_to_file = False
    
    try:
        # Load outline
        outline = json.loads(outline_file.read_text())
        
        # Generate sidebar items
        items = generate_sidebar_items(outline)
        
        # Format as TypeScript
        ts_content = format_sidebar_ts(items)
        
        # Output
        if output_to_file:
            output_file.write_text(ts_content)
            print(f"✅ Generated sidebar: {output_file}", file=sys.stderr)
        else:
            print(ts_content)
        
    except Exception as e:
        print(f"❌ Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()