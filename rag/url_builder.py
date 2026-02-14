"""Generate GitHub Pages URLs compatible with Docusaurus baseUrl."""

import re

BASE_URL = "/physical-ai-robotics-textbook/docs/"


def slugify(text: str) -> str:
    """Slugify a heading for use as a URL anchor.

    Matches Docusaurus auto-generated anchor behavior:
    lowercase, replace spaces/specials with hyphens, collapse multiples.
    """
    slug = text.lower()
    slug = re.sub(r"[^\w\s-]", "", slug)
    slug = re.sub(r"[\s_]+", "-", slug)
    slug = re.sub(r"-+", "-", slug)
    slug = slug.strip("-")
    return slug


def build_url(doc_path: str, section_heading: str | None = None) -> str:
    """Generate a GitHub Pages URL from doc_path and optional section heading.

    Args:
        doc_path: Path relative to website/docs/ (e.g. "module-1/.../file.md")
        section_heading: Optional heading text for anchor generation
    """
    path = doc_path
    if path.startswith("website/docs/"):
        path = path[len("website/docs/"):]

    if path.endswith(".md"):
        path = path[:-3]

    if path.endswith("/index"):
        path = path[:-5]
    elif path == "index":
        path = ""

    if path and not path.endswith("/"):
        url = BASE_URL + path
    elif path:
        url = BASE_URL + path
    else:
        url = BASE_URL

    if section_heading:
        url += "#" + slugify(section_heading)

    return url
