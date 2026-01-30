#!/usr/bin/env python3
"""
MDX Parser for Docusaurus Files

Parses MDX files, extracting:
- YAML frontmatter
- Import statements
- JSX components
- Prose content
- Code blocks

Usage:
    python mdx_parser.py <mdx_file_path>
"""

import re
import yaml
from typing import Dict, List, Tuple
from dataclasses import dataclass
from pathlib import Path


@dataclass
class MDXContent:
    """Structured representation of MDX file."""
    frontmatter: dict
    imports: List[str]
    components: List[Dict]
    prose_sections: List[str]
    code_blocks: List[Dict]
    metadata: dict


class MDXParser:
    """Parser for Docusaurus MDX files."""
    
    def __init__(self):
        self.frontmatter_pattern = r'^---\n(.*?)\n---'
        self.import_pattern = r'^import\s+.*?;?\s*$'
        self.jsx_pattern = r'<([A-Z][A-Za-z0-9]*)[^>]*>(.*?)</\1>|<([A-Z][A-Za-z0-9]*)[^>]*\/>'
        self.code_block_pattern = r'```(\w+)?\n(.*?)\n```'
    
    def parse(self, mdx_path: str) -> MDXContent:
        """
        Parse MDX file and extract structured content.
        """
        with open(mdx_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract frontmatter
        frontmatter = self._extract_frontmatter(content)
        content_without_frontmatter = self._remove_frontmatter(content)
        
        # Extract imports
        imports = self._extract_imports(content_without_frontmatter)
        content_without_imports = self._remove_imports(content_without_frontmatter)
        
        # Extract code blocks (before JSX to avoid conflicts)
        code_blocks = self._extract_code_blocks(content_without_imports)
        content_without_code = self._remove_code_blocks(content_without_imports)
        
        # Extract JSX components
        components = self._extract_jsx_components(content_without_code)
        
        # Extract prose (remaining text after removing JSX)
        prose = self._extract_prose(content_without_code)
        
        # Build metadata
        metadata = self._build_metadata(frontmatter, imports, components, code_blocks)
        
        return MDXContent(
            frontmatter=frontmatter,
            imports=imports,
            components=components,
            prose_sections=prose,
            code_blocks=code_blocks,
            metadata=metadata
        )
    
    def _extract_frontmatter(self, content: str) -> dict:
        """Extract YAML frontmatter."""
        match = re.search(self.frontmatter_pattern, content, re.DOTALL)
        if match:
            yaml_content = match.group(1)
            try:
                return yaml.safe_load(yaml_content) or {}
            except yaml.YAMLError:
                return {}
        return {}
    
    def _remove_frontmatter(self, content: str) -> str:
        """Remove frontmatter from content."""
        return re.sub(self.frontmatter_pattern, '', content, flags=re.DOTALL).lstrip()
    
    def _extract_imports(self, content: str) -> List[str]:
        """Extract import statements."""
        imports = []
        lines = content.split('\n')
        for line in lines:
            if re.match(self.import_pattern, line, re.MULTILINE):
                imports.append(line.strip())
        return imports
    
    def _remove_imports(self, content: str) -> str:
        """Remove import statements from content."""
        lines = content.split('\n')
        filtered_lines = [
            line for line in lines 
            if not re.match(self.import_pattern, line, re.MULTILINE)
        ]
        return '\n'.join(filtered_lines)
    
    def _extract_code_blocks(self, content: str) -> List[Dict]:
        """Extract code blocks with language and content."""
        code_blocks = []
        for match in re.finditer(self.code_block_pattern, content, re.DOTALL):
            language = match.group(1) or 'text'
            code_content = match.group(2)
            code_blocks.append({
                'language': language,
                'content': code_content,
                'start_pos': match.start(),
                'end_pos': match.end()
            })
        return code_blocks
    
    def _remove_code_blocks(self, content: str) -> str:
        """Replace code blocks with placeholders."""
        return re.sub(self.code_block_pattern, '[CODE_BLOCK]', content, flags=re.DOTALL)
    
    def _extract_jsx_components(self, content: str) -> List[Dict]:
        """Extract JSX component instances."""
        components = []
        for match in re.finditer(self.jsx_pattern, content, re.DOTALL):
            # Self-closing or paired tags
            component_name = match.group(1) or match.group(3)
            component_content = match.group(2) if match.group(2) else ''
            
            # Extract props from the tag
            full_tag = match.group(0)
            props = self._extract_props(full_tag)
            
            components.append({
                'name': component_name,
                'props': props,
                'content': component_content.strip(),
                'start_pos': match.start(),
                'end_pos': match.end()
            })
        return components
    
    def _extract_props(self, tag: str) -> dict:
        """Extract props from JSX tag."""
        props = {}
        # Simple prop extraction (key="value" or key={value})
        prop_pattern = r'(\w+)=(?:"([^"]*)"|{([^}]*)})'
        for match in re.finditer(prop_pattern, tag):
            key = match.group(1)
            value = match.group(2) or match.group(3)
            props[key] = value
        return props
    
    def _extract_prose(self, content: str) -> List[str]:
        """Extract prose sections (text between JSX components)."""
        # Remove JSX components
        prose_content = re.sub(self.jsx_pattern, '', content, flags=re.DOTALL)
        # Remove placeholder code blocks
        prose_content = prose_content.replace('[CODE_BLOCK]', '')
        
        # Split by double newlines (paragraphs)
        sections = [s.strip() for s in prose_content.split('\n\n') if s.strip()]
        return sections
    
    def _build_metadata(self, frontmatter: dict, imports: List[str], 
                       components: List[Dict], code_blocks: List[Dict]) -> dict:
        """Build metadata from parsed content."""
        # Extract programming languages
        languages = list(set(
            block['language'] for block in code_blocks 
            if block['language'] != 'text'
        ))
        
        # Component types used
        component_types = list(set(comp['name'] for comp in components))
        
        # Build structured metadata
        metadata = {
            'title': frontmatter.get('title', ''),
            'sidebar_label': frontmatter.get('sidebar_label', ''),
            'sidebar_position': frontmatter.get('sidebar_position', 0),
            'has_code': len(code_blocks) > 0,
            'programming_languages': languages,
            'has_jsx_components': len(components) > 0,
            'component_types': component_types,
            'import_count': len(imports),
            'code_block_count': len(code_blocks),
            'component_count': len(components)
        }
        
        # Add custom frontmatter fields
        for key, value in frontmatter.items():
            if key not in metadata:
                metadata[key] = value
        
        return metadata


def parse_mdx_file(mdx_path: str) -> MDXContent:
    """
    Convenience function to parse an MDX file.
    
    Args:
        mdx_path: Path to MDX file
    
    Returns:
        MDXContent object with parsed content
    """
    parser = MDXParser()
    return parser.parse(mdx_path)


def main():
    """CLI entry point."""
    import sys
    import json
    
    if len(sys.argv) < 2:
        print("Usage: python mdx_parser.py <mdx_file_path>")
        sys.exit(1)
    
    mdx_path = sys.argv[1]
    
    if not Path(mdx_path).exists():
        print(f"Error: File not found: {mdx_path}")
        sys.exit(1)
    
    # Parse file
    result = parse_mdx_file(mdx_path)
    
    # Print results
    print("=== Frontmatter ===")
    print(json.dumps(result.frontmatter, indent=2))
    
    print("\n=== Imports ===")
    for imp in result.imports:
        print(imp)
    
    print("\n=== Components ===")
    for comp in result.components:
        print(f"{comp['name']}: {comp['props']}")
    
    print("\n=== Code Blocks ===")
    for block in result.code_blocks:
        print(f"Language: {block['language']}, Lines: {len(block['content'].split(chr(10)))}")
    
    print("\n=== Prose Sections ===")
    print(f"Total sections: {len(result.prose_sections)}")
    
    print("\n=== Metadata ===")
    print(json.dumps(result.metadata, indent=2))


if __name__ == '__main__':
    main()