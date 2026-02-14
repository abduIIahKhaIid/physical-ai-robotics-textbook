"""Tests for rag.parser.markdown_splitter structural block parsing."""

from rag.parser.markdown_splitter import BlockType, split_markdown


class TestSplitMarkdown:
    def test_heading_detection(self):
        content = "# Title\n\nSome text\n\n## Subtitle"
        blocks = split_markdown(content)
        headings = [b for b in blocks if b.block_type == BlockType.HEADING]
        assert len(headings) == 2
        assert headings[0].heading_level == 1
        assert headings[0].heading_text == "Title"
        assert headings[1].heading_level == 2

    def test_fenced_code_block_atomic(self):
        content = "```python\ndef foo():\n    pass\n```"
        blocks = split_markdown(content)
        code_blocks = [b for b in blocks if b.block_type == BlockType.CODE]
        assert len(code_blocks) == 1
        assert "def foo():" in code_blocks[0].text
        assert "```" in code_blocks[0].text

    def test_tilde_code_block(self):
        content = "~~~\ncode here\n~~~"
        blocks = split_markdown(content)
        code_blocks = [b for b in blocks if b.block_type == BlockType.CODE]
        assert len(code_blocks) == 1

    def test_pipe_table_atomic(self):
        content = "| A | B |\n|---|---|\n| 1 | 2 |"
        blocks = split_markdown(content)
        table_blocks = [b for b in blocks if b.block_type == BlockType.TABLE]
        assert len(table_blocks) == 1
        assert "| A | B |" in table_blocks[0].text

    def test_admonition_detected(self):
        content = ":::note\nThis is a note.\n:::"
        blocks = split_markdown(content)
        admon_blocks = [b for b in blocks if b.block_type == BlockType.ADMONITION]
        assert len(admon_blocks) == 1
        assert "note" in admon_blocks[0].text

    def test_consecutive_prose_merged(self):
        content = "Line one\nLine two\nLine three"
        blocks = split_markdown(content)
        prose_blocks = [b for b in blocks if b.block_type == BlockType.PROSE]
        assert len(prose_blocks) == 1
        assert "Line one" in prose_blocks[0].text
        assert "Line three" in prose_blocks[0].text

    def test_latex_preserved(self):
        content = "The equation $E = mc^2$ is important.\n\n$$\nF = ma\n$$"
        blocks = split_markdown(content)
        all_text = " ".join(b.text for b in blocks)
        assert "$E = mc^2$" in all_text
        assert "$$" in all_text

    def test_sample_chapter(self, sample_chapter_path):
        with open(sample_chapter_path) as f:
            import frontmatter
            post = frontmatter.load(f)
        blocks = split_markdown(post.content)
        types = {b.block_type for b in blocks}
        assert BlockType.HEADING in types
        assert BlockType.PROSE in types
        assert BlockType.CODE in types
        assert BlockType.TABLE in types
