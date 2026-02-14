"""Parse YAML frontmatter and content from Docusaurus markdown files."""

import re

import frontmatter

from rag.models import Document


def _title_from_h1(content: str) -> str:
    """Extract title from first H1 heading in content."""
    match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
    return match.group(1).strip() if match else ""


def _title_from_filename(path: str) -> str:
    """Derive a title from filename by replacing hyphens with spaces."""
    from pathlib import Path

    stem = Path(path).stem
    return stem.replace("-", " ").replace("_", " ").title()


def parse_frontmatter(doc: Document) -> Document:
    """Read file, parse YAML frontmatter, and populate Document fields.

    Falls back gracefully when frontmatter is missing.

    Args:
        doc: Document with absolute_path set.

    Returns:
        Document with title, description, tags, content, etc. populated.
    """
    with open(doc.absolute_path, "r", encoding="utf-8") as f:
        post = frontmatter.load(f)

    content = post.content
    metadata = post.metadata

    title = metadata.get("title", "")
    if not title:
        title = _title_from_h1(content) or _title_from_filename(doc.absolute_path)

    return doc.model_copy(
        update={
            "title": title,
            "description": metadata.get("description", ""),
            "tags": metadata.get("tags", []),
            "learning_objectives": metadata.get("learning-objectives", []),
            "sidebar_label": metadata.get("sidebar_label", ""),
            "sidebar_position": metadata.get("sidebar_position", 0),
            "content": content,
        }
    )
