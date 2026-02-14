"""Tests for rag.parser.frontmatter YAML parsing."""

from rag.models import Document
from rag.parser.frontmatter import parse_frontmatter


class TestParseFrontmatter:
    def test_extracts_title(self, sample_chapter_path):
        doc = Document(doc_path="test.md", absolute_path=sample_chapter_path)
        result = parse_frontmatter(doc)
        assert result.title == "1.1 Foundations of Physical AI"

    def test_extracts_description(self, sample_chapter_path):
        doc = Document(doc_path="test.md", absolute_path=sample_chapter_path)
        result = parse_frontmatter(doc)
        assert "core principles" in result.description.lower()

    def test_extracts_tags(self, sample_chapter_path):
        doc = Document(doc_path="test.md", absolute_path=sample_chapter_path)
        result = parse_frontmatter(doc)
        assert "physical-ai" in result.tags
        assert "foundations" in result.tags

    def test_extracts_learning_objectives(self, sample_chapter_path):
        doc = Document(doc_path="test.md", absolute_path=sample_chapter_path)
        result = parse_frontmatter(doc)
        assert len(result.learning_objectives) > 0

    def test_extracts_content(self, sample_chapter_path):
        doc = Document(doc_path="test.md", absolute_path=sample_chapter_path)
        result = parse_frontmatter(doc)
        assert "Physical AI" in result.content

    def test_no_frontmatter_fallback(self, no_frontmatter_path):
        doc = Document(doc_path="no_frontmatter.md", absolute_path=no_frontmatter_path)
        result = parse_frontmatter(doc)
        assert result.title == "Document Without Frontmatter"
        assert result.tags == []

    def test_missing_fields_default(self, sample_quiz_path):
        doc = Document(doc_path="quiz.md", absolute_path=sample_quiz_path)
        result = parse_frontmatter(doc)
        assert result.sidebar_label == ""
        assert result.sidebar_position == 0
