"""Discover Docusaurus markdown documents in a directory tree."""

from pathlib import Path

from rag.models import ContentType, Document


_CONTENT_TYPE_KEYWORDS = {
    ContentType.LAB: ("lab", "exercise"),
    ContentType.QUIZ: ("quiz",),
    ContentType.ASSESSMENT: ("capstone", "assessment", "evaluation"),
}


def _detect_content_type(filename: str) -> ContentType:
    """Infer content type from filename keywords."""
    lower = filename.lower()
    for ct, keywords in _CONTENT_TYPE_KEYWORDS.items():
        if any(kw in lower for kw in keywords):
            return ct
    return ContentType.PROSE


def _extract_module(doc_path: str) -> str:
    """Extract module slug from doc_path (e.g. 'module-1')."""
    parts = Path(doc_path).parts
    for part in parts:
        if part.startswith("module-"):
            return part
    return ""


def _extract_chapter(doc_path: str) -> str:
    """Extract chapter slug from doc_path (second-level directory)."""
    parts = Path(doc_path).parts
    for i, part in enumerate(parts):
        if part.startswith("module-") and i + 1 < len(parts):
            candidate = parts[i + 1]
            if not candidate.endswith(".md"):
                return candidate
    return ""


def discover_documents(docs_dir: str) -> list[Document]:
    """Recursively discover .md files and return Document objects.

    Args:
        docs_dir: Root directory containing Docusaurus docs (website/docs/).

    Returns:
        List of Document objects with paths and content_type set.

    Raises:
        FileNotFoundError: If docs_dir does not exist.
    """
    root = Path(docs_dir)
    if not root.is_dir():
        raise FileNotFoundError(f"Docs directory not found: {docs_dir}")

    documents: list[Document] = []
    for md_path in sorted(root.rglob("*.md")):
        if not md_path.is_file():
            continue
        doc_path = str(md_path.relative_to(root))
        documents.append(
            Document(
                doc_path=doc_path,
                absolute_path=str(md_path),
                module=_extract_module(doc_path),
                chapter=_extract_chapter(doc_path),
                content_type=_detect_content_type(md_path.name),
            )
        )
    return documents
