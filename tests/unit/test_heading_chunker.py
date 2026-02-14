"""Tests for rag.chunker.heading_chunker."""

import tiktoken

from rag.chunker.heading_chunker import chunk_blocks
from rag.models import ContentType, Document
from rag.parser.markdown_splitter import split_markdown


_encoder = tiktoken.get_encoding("cl100k_base")


def _count_tokens(text: str) -> int:
    return len(_encoder.encode(text))


def _make_doc(**kwargs):
    defaults = dict(
        doc_path="module-1/1.1-intro/file.md",
        module="module-1",
        chapter="1.1-intro",
        title="Test Doc",
        tags=["test"],
        content_type=ContentType.PROSE,
    )
    defaults.update(kwargs)
    return Document(**defaults)


class TestChunkBlocks:
    def test_basic_chunking(self):
        content = "# Title\n\n" + ("word " * 150) + "\n\n## Section 2\n\n" + ("another " * 150)
        blocks = split_markdown(content)
        doc = _make_doc()
        chunks = chunk_blocks(blocks, doc, min_tokens=50, max_tokens=400, target_tokens=100)
        assert len(chunks) >= 2

    def test_respects_heading_boundaries(self):
        content = "# H1\n\nShort text.\n\n## H2\n\n" + ("word " * 300)
        blocks = split_markdown(content)
        doc = _make_doc()
        chunks = chunk_blocks(blocks, doc, min_tokens=10, max_tokens=400, target_tokens=100)
        # First chunk should contain "Short text" and not bleed into H2 content
        if len(chunks) > 1:
            assert "Short text" in chunks[0].text

    def test_token_range(self, sample_chapter_path):
        import frontmatter as fm
        with open(sample_chapter_path) as f:
            post = fm.load(f)
        blocks = split_markdown(post.content)
        doc = _make_doc()
        chunks = chunk_blocks(blocks, doc, min_tokens=50, max_tokens=800, target_tokens=200)
        for chunk in chunks:
            tokens = _count_tokens(chunk.text)
            # Allow small chunks at boundaries
            assert tokens <= 1200, f"Chunk too large: {tokens} tokens"

    def test_oversized_code_becomes_solo(self):
        big_code = "```python\n" + ("x = 1\n" * 500) + "```"
        content = "# Title\n\nSome text.\n\n" + big_code + "\n\nMore text."
        blocks = split_markdown(content)
        doc = _make_doc()
        chunks = chunk_blocks(blocks, doc, min_tokens=50, max_tokens=200, target_tokens=100)
        code_chunks = [c for c in chunks if "```python" in c.text]
        assert len(code_chunks) == 1

    def test_breadcrumb_tracking(self):
        content = "# H1\n\n## H2a\n\nText a.\n\n## H2b\n\nText b."
        blocks = split_markdown(content)
        doc = _make_doc()
        chunks = chunk_blocks(blocks, doc, min_tokens=1, max_tokens=800, target_tokens=10)
        # Should have at least one chunk with breadcrumb info
        all_breadcrumbs = [c.metadata.heading_breadcrumb for c in chunks]
        assert any(len(b) > 0 for b in all_breadcrumbs)

    def test_metadata_populated(self):
        content = "# Title\n\nSome text content here."
        blocks = split_markdown(content)
        doc = _make_doc()
        chunks = chunk_blocks(blocks, doc, min_tokens=1, max_tokens=800, target_tokens=10)
        assert len(chunks) > 0
        c = chunks[0]
        assert c.doc_path == "module-1/1.1-intro/file.md"
        assert c.metadata.module == "module-1"
        assert c.metadata.title == "Test Doc"
        assert c.metadata.url != ""
