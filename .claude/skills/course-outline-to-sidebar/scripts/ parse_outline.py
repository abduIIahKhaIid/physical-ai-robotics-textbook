#!/usr/bin/env python3
"""
Course Outline Parser

Extracts hierarchical course structure from text and outputs JSON.

Usage:
    parse_outline.py <input_file> [--output <output_file>]

Example:
    parse_outline.py course_description.txt --output outline.json
"""

import re
import json
import sys
from pathlib import Path
from typing import List, Dict, Any


def slugify(text: str) -> str:
    """Convert text to URL-friendly slug."""
    # Remove content in parentheses
    text = re.sub(r'\([^)]*\)', '', text)
    # Convert to lowercase
    text = text.lower()
    # Replace special characters and spaces with hyphens
    text = re.sub(r'[^a-z0-9]+', '-', text)
    # Remove leading/trailing hyphens
    text = text.strip('-')
    # Collapse multiple consecutive hyphens
    text = re.sub(r'-+', '-', text)
    return text


def parse_week_range(week_text: str) -> tuple:
    """Parse week range from text like 'Weeks 1-2' or 'Week 3-5'."""
    match = re.search(r'Week[s]?\s+(\d+)(?:-(\d+))?', week_text, re.IGNORECASE)
    if match:
        start = int(match.group(1))
        end = int(match.group(2)) if match.group(2) else start
        return (start, end)
    return (None, None)


def parse_course_outline(text: str) -> Dict[str, Any]:
    """
    Parse course outline text into structured JSON.
    
    Expected format:
    - Module lines start with "Module" or "●"
    - Week lines start with "Week" or "○"  
    - Chapter/topic lines are indented or bulleted
    """
    lines = text.split('\n')
    
    course = {
        'title': '',
        'modules': []
    }
    
    current_module = None
    current_week = None
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
            
        # Extract course title from first significant line or header
        if not course['title'] and ('Physical AI' in line or 'Robotics' in line):
            course['title'] = line.strip('#').strip()
            continue
        
        # Module detection
        module_match = re.match(r'[●•]\s*Module\s+(\d+):\s*(.+)', line, re.IGNORECASE)
        if not module_match:
            module_match = re.match(r'Module\s+(\d+):\s*(.+)', line, re.IGNORECASE)
        
        if module_match:
            module_num = int(module_match.group(1))
            module_title = module_match.group(2).strip()
            current_module = {
                'number': module_num,
                'title': module_title,
                'slug': slugify(module_title),
                'weeks': []
            }
            course['modules'].append(current_module)
            current_week = None
            continue
        
        # Week detection
        week_match = re.match(r'[○◦]\s*(.+)', line)
        if week_match:
            week_text = week_match.group(1).strip()
        elif re.match(r'Week[s]?\s+\d+', line, re.IGNORECASE):
            week_text = line
        else:
            week_text = None
            
        if week_text and current_module:
            start, end = parse_week_range(week_text)
            if start:
                # Extract topic from week text
                topic_match = re.search(r':\s*(.+)', week_text)
                topic = topic_match.group(1).strip() if topic_match else week_text
                
                current_week = {
                    'start': start,
                    'end': end,
                    'topic': topic,
                    'slug': f'week-{start}' if start == end else f'week-{start}-{end}',
                    'chapters': []
                }
                current_module['weeks'].append(current_week)
            continue
        
        # Chapter/lesson detection (bullet points or indented)
        if current_week and (line.startswith('-') or line.startswith('•') or line.startswith('○')):
            chapter_text = re.sub(r'^[-•○]\s*', '', line).strip()
            if chapter_text:
                current_week['chapters'].append({
                    'title': chapter_text,
                    'slug': slugify(chapter_text)
                })
    
    return course


def main():
    if len(sys.argv) < 2:
        print("Usage: parse_outline.py <input_file> [--output <output_file>]")
        sys.exit(1)
    
    input_file = Path(sys.argv[1])
    
    # Determine output file
    if '--output' in sys.argv:
        output_idx = sys.argv.index('--output')
        output_file = Path(sys.argv[output_idx + 1])
    else:
        output_file = input_file.with_suffix('.json')
    
    # Read and parse
    try:
        text = input_file.read_text()
        outline = parse_course_outline(text)
        
        # Write JSON
        output_file.write_text(json.dumps(outline, indent=2))
        print(f"✅ Parsed outline saved to: {output_file}")
        print(f"   Found {len(outline['modules'])} modules")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()