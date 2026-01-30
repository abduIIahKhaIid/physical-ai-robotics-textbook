# Chunking Algorithms Reference

## Overview

This document provides detailed implementations of chunking algorithms for Docusaurus textbook content.

## Semantic Text Splitter

```python
import re
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class TextChunk:
    content: str
    start_pos: int
    end_pos: int
    metadata: dict

class SemanticTextSplitter:
    """
    Splits text at semantic boundaries while respecting size constraints.
    """
    
    def __init__(self, target_size: int = 600, overlap: int = 100, 
                 min_size: int = 200, max_size: int = 1200):
        self.target_size = target_size
        self.overlap = overlap
        self.min_size = min_size
        self.max_size = max_size
    
    def split(self, text: str, metadata: dict = None) -> List[TextChunk]:
        """
        Main splitting function.
        """
        # Parse document structure
        sections = self._parse_structure(text)
        
        # Chunk each section
        chunks = []
        for section in sections:
            section_chunks = self._chunk_section(section)
            chunks.extend(section_chunks)
        
        # Add overlap
        chunks = self._add_overlap(chunks)
        
        return chunks
    
    def _parse_structure(self, text: str) -> List[dict]:
        """
        Parse markdown structure into hierarchical sections.
        """
        # Split by headings
        heading_pattern = r'^(#{1,6})\s+(.+)$'
        lines = text.split('\n')
        
        sections = []
        current_section = {
            'level': 0,
            'title': 'Root',
            'content': [],
            'start_line': 0
        }
        
        for i, line in enumerate(lines):
            match = re.match(heading_pattern, line, re.MULTILINE)
            if match:
                # Save previous section
                if current_section['content']:
                    current_section['content'] = '\n'.join(current_section['content'])
                    current_section['end_line'] = i - 1
                    sections.append(current_section)
                
                # Start new section
                level = len(match.group(1))
                title = match.group(2)
                current_section = {
                    'level': level,
                    'title': title,
                    'content': [],
                    'start_line': i
                }
            else:
                current_section['content'].append(line)
        
        # Add final section
        if current_section['content']:
            current_section['content'] = '\n'.join(current_section['content'])
            current_section['end_line'] = len(lines) - 1
            sections.append(current_section)
        
        return sections
    
    def _chunk_section(self, section: dict) -> List[TextChunk]:
        """
        Chunk a single section.
        """
        content = section['content']
        tokens = self._estimate_tokens(content)
        
        # If section fits in one chunk, return it
        if tokens <= self.target_size:
            return [TextChunk(
                content=content,
                start_pos=section['start_line'],
                end_pos=section['end_line'],
                metadata={
                    'heading': section['title'],
                    'heading_level': section['level']
                }
            )]
        
        # Split by paragraphs
        paragraphs = self._split_paragraphs(content)
        chunks = []
        current_chunk = []
        current_size = 0
        
        for para in paragraphs:
            para_size = self._estimate_tokens(para)
            
            # If single paragraph exceeds max, split it forcefully
            if para_size > self.max_size:
                if current_chunk:
                    chunks.append(self._create_chunk(current_chunk, section))
                    current_chunk = []
                    current_size = 0
                
                # Force split the large paragraph
                para_chunks = self._force_split(para)
                chunks.extend([self._create_chunk([p], section) for p in para_chunks])
                continue
            
            # If adding paragraph exceeds target, save current chunk
            if current_size + para_size > self.target_size and current_chunk:
                chunks.append(self._create_chunk(current_chunk, section))
                current_chunk = []
                current_size = 0
            
            current_chunk.append(para)
            current_size += para_size
        
        # Add remaining content
        if current_chunk:
            chunks.append(self._create_chunk(current_chunk, section))
        
        return chunks
    
    def _split_paragraphs(self, text: str) -> List[str]:
        """
        Split text into paragraphs, preserving code blocks and special elements.
        """
        # Detect code blocks
        code_block_pattern = r'```[\s\S]*?```'
        code_blocks = list(re.finditer(code_block_pattern, text))
        
        # Split into paragraphs, avoiding code blocks
        paragraphs = []
        last_pos = 0
        
        for code_block in code_blocks:
            # Add paragraphs before code block
            before_text = text[last_pos:code_block.start()]
            paras = [p.strip() for p in before_text.split('\n\n') if p.strip()]
            paragraphs.extend(paras)
            
            # Add code block as single paragraph
            paragraphs.append(code_block.group(0))
            last_pos = code_block.end()
        
        # Add remaining paragraphs
        remaining_text = text[last_pos:]
        paras = [p.strip() for p in remaining_text.split('\n\n') if p.strip()]
        paragraphs.extend(paras)
        
        return paragraphs
    
    def _force_split(self, text: str, boundary: str = '. ') -> List[str]:
        """
        Force split text that exceeds max_size.
        Tries to split at sentence boundaries.
        """
        sentences = text.split(boundary)
        chunks = []
        current = []
        current_size = 0
        
        for sentence in sentences:
            sentence_size = self._estimate_tokens(sentence)
            
            if current_size + sentence_size > self.max_size and current:
                chunks.append(boundary.join(current))
                current = []
                current_size = 0
            
            current.append(sentence)
            current_size += sentence_size
        
        if current:
            chunks.append(boundary.join(current))
        
        return chunks
    
    def _estimate_tokens(self, text: str) -> int:
        """
        Rough token estimation: 1 token ≈ 4 characters.
        """
        return len(text) // 4
    
    def _create_chunk(self, paragraphs: List[str], section: dict) -> TextChunk:
        """
        Create TextChunk from paragraphs.
        """
        content = '\n\n'.join(paragraphs)
        return TextChunk(
            content=content,
            start_pos=0,  # Would need actual line tracking
            end_pos=0,
            metadata={
                'heading': section['title'],
                'heading_level': section['level']
            }
        )
    
    def _add_overlap(self, chunks: List[TextChunk]) -> List[TextChunk]:
        """
        Add overlap between consecutive chunks.
        """
        if len(chunks) <= 1:
            return chunks
        
        overlapped_chunks = [chunks[0]]
        
        for i in range(1, len(chunks)):
            prev_chunk = chunks[i - 1]
            curr_chunk = chunks[i]
            
            # Extract last N tokens from previous chunk
            prev_words = prev_chunk.content.split()
            overlap_words = prev_words[-self.overlap:] if len(prev_words) > self.overlap else prev_words
            overlap_text = ' '.join(overlap_words)
            
            # Prepend to current chunk
            new_content = overlap_text + '\n\n' + curr_chunk.content
            overlapped_chunk = TextChunk(
                content=new_content,
                start_pos=curr_chunk.start_pos,
                end_pos=curr_chunk.end_pos,
                metadata=curr_chunk.metadata
            )
            overlapped_chunks.append(overlapped_chunk)
        
        return overlapped_chunks
```

## Code-Aware Chunking

```python
import ast
from typing import List

class CodeBlockChunker:
    """
    Specialized chunker for code blocks.
    Preserves syntactic boundaries.
    """
    
    def chunk_python(self, code: str, max_size: int = 1200) -> List[str]:
        """
        Chunk Python code at function/class boundaries.
        """
        try:
            tree = ast.parse(code)
        except SyntaxError:
            # If unparseable, treat as plain text
            return [code]
        
        chunks = []
        imports = []
        current_chunk = []
        current_size = 0
        
        for node in ast.walk(tree):
            # Collect imports
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                imports.append(ast.unparse(node))
            
            # Split at function/class definitions
            elif isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                node_code = ast.unparse(node)
                node_size = len(node_code) // 4
                
                # If node exceeds max_size, keep as single chunk
                if node_size > max_size:
                    if current_chunk:
                        chunks.append(self._build_chunk(imports, current_chunk))
                        current_chunk = []
                        current_size = 0
                    chunks.append(self._build_chunk(imports, [node_code]))
                    continue
                
                # If adding node exceeds max_size, save current chunk
                if current_size + node_size > max_size and current_chunk:
                    chunks.append(self._build_chunk(imports, current_chunk))
                    current_chunk = []
                    current_size = 0
                
                current_chunk.append(node_code)
                current_size += node_size
        
        if current_chunk:
            chunks.append(self._build_chunk(imports, current_chunk))
        
        return chunks if chunks else [code]
    
    def _build_chunk(self, imports: List[str], code_sections: List[str]) -> str:
        """
        Build code chunk with imports.
        """
        import_block = '\n'.join(imports)
        code_block = '\n\n'.join(code_sections)
        return f"{import_block}\n\n{code_block}" if imports else code_block
```

## Table Chunking

```python
class TableChunker:
    """
    Specialized chunker for markdown tables.
    """
    
    def chunk_table(self, table: str, max_rows: int = 20) -> List[str]:
        """
        Split large tables by rows while preserving headers.
        """
        lines = table.strip().split('\n')
        
        # Extract header (first 2 lines in markdown table)
        if len(lines) < 2:
            return [table]
        
        header = lines[:2]
        rows = lines[2:]
        
        if len(rows) <= max_rows:
            return [table]
        
        # Split into chunks
        chunks = []
        for i in range(0, len(rows), max_rows):
            chunk_rows = rows[i:i + max_rows]
            chunk_table = '\n'.join(header + chunk_rows)
            chunks.append(chunk_table)
        
        return chunks
```

## Usage Example

```python
# Initialize splitter
splitter = SemanticTextSplitter(
    target_size=600,
    overlap=100,
    min_size=200,
    max_size=1200
)

# Read document
with open('docs/module-1/ros2-intro.md', 'r') as f:
    content = f.read()

# Split into chunks
chunks = splitter.split(content)

# Enrich with metadata
for i, chunk in enumerate(chunks):
    chunk.metadata.update({
        'chunk_index': i,
        'document_id': 'module-1/ros2-intro',
        'course_name': 'Physical AI & Humanoid Robotics'
    })

# Generate embeddings and store
for chunk in chunks:
    embedding = generate_embedding(chunk.content)
    qdrant_client.upsert(
        collection_name='textbook_chunks',
        points=[{
            'id': generate_uuid(),
            'vector': embedding,
            'payload': {
                'content': chunk.content,
                **chunk.metadata
            }
        }]
    )
```