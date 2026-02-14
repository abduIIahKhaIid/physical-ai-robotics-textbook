"""Tests for rag.parser.discovery document discovery."""

import pytest

from rag.models import ContentType
from rag.parser.discovery import discover_documents


class TestDiscoverDocuments:
    def test_finds_md_files(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        assert len(docs) >= 2
        paths = [d.doc_path for d in docs]
        assert any("physical-ai-foundations.md" in p for p in paths)
        assert any("physical-ai-lab-exercises.md" in p for p in paths)

    def test_doc_path_is_relative(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        for doc in docs:
            assert not doc.doc_path.startswith("/")

    def test_extracts_module(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        for doc in docs:
            assert doc.module == "module-1"

    def test_extracts_chapter(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        for doc in docs:
            assert doc.chapter == "1.1-introduction-to-physical-ai"

    def test_detects_lab_content_type(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        lab_docs = [d for d in docs if "lab" in d.doc_path.lower()]
        assert len(lab_docs) > 0
        assert lab_docs[0].content_type == ContentType.LAB

    def test_detects_prose_content_type(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        prose_docs = [d for d in docs if "foundations" in d.doc_path.lower()]
        assert len(prose_docs) > 0
        assert prose_docs[0].content_type == ContentType.PROSE

    def test_missing_dir_raises(self):
        with pytest.raises(FileNotFoundError):
            discover_documents("/nonexistent/path/that/does/not/exist")

    def test_absolute_path_set(self, temp_docs_dir):
        docs = discover_documents(temp_docs_dir)
        for doc in docs:
            assert doc.absolute_path.startswith("/")
