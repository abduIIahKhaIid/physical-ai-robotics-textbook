"""Tests for rag.url_builder URL generation."""

from rag.url_builder import BASE_URL, build_url, slugify


class TestSlugify:
    def test_basic(self):
        assert slugify("Core Principles") == "core-principles"

    def test_special_characters(self):
        assert slugify("What's New (2024)?") == "whats-new-2024"

    def test_multiple_spaces(self):
        assert slugify("hello   world") == "hello-world"

    def test_leading_trailing_hyphens(self):
        assert slugify("--hello--") == "hello"

    def test_empty(self):
        assert slugify("") == ""

    def test_underscores(self):
        assert slugify("foo_bar") == "foo-bar"


class TestBuildUrl:
    def test_regular_doc_path(self):
        url = build_url("module-1/1.1-intro/file.md")
        assert url == BASE_URL + "module-1/1.1-intro/file"

    def test_strips_website_docs_prefix(self):
        url = build_url("website/docs/module-1/file.md")
        assert url == BASE_URL + "module-1/file"

    def test_index_md_handling(self):
        url = build_url("module-1/index.md")
        assert url == BASE_URL + "module-1/"

    def test_root_index(self):
        url = build_url("index.md")
        assert url == BASE_URL

    def test_with_section_heading(self):
        url = build_url("module-1/file.md", "Core Principles")
        assert url == BASE_URL + "module-1/file#core-principles"

    def test_empty_heading_no_anchor(self):
        url = build_url("module-1/file.md", None)
        assert "#" not in url

    def test_base_url_prefix(self):
        url = build_url("module-1/file.md")
        assert url.startswith("/physical-ai-robotics-textbook/docs/")
