"""Tests for rag.response.grounding policy enforcement."""

from rag.models import (
    ChunkMetadata,
    Citation,
    Query,
    RetrievalResult,
    ScoredChunk,
)
from rag.response.grounding import apply_grounding_policy


def _make_result(mode: str, chunks: list[ScoredChunk] | None = None) -> RetrievalResult:
    q = Query(
        question="Test question",
        mode=mode,
        selected_text="selected" if mode == "selected_text_only" else None,
    )
    return RetrievalResult(
        chunks=chunks or [],
        query=q,
        mode=mode,
    )


def _make_chunk(text: str = "Some answer text.") -> ScoredChunk:
    return ScoredChunk(
        chunk_id="test",
        text=text,
        score=0.9,
        metadata=ChunkMetadata(doc_path="test.md", title="Title", section_heading="Section"),
        citation=Citation(title="Title", section="Section", url="/test"),
    )


class TestApplyGroundingPolicy:
    def test_empty_chunks_refusal(self):
        result = _make_result("normal", [])
        grounded = apply_grounding_policy(result, "What is AI?")
        assert grounded.sufficient_context is False
        assert "cannot find" in grounded.system_instruction.lower() or "do not" in grounded.system_instruction.lower()

    def test_normal_mode_citation_instruction(self):
        result = _make_result("normal", [_make_chunk()])
        grounded = apply_grounding_policy(result, "What is AI?")
        assert grounded.sufficient_context is True
        assert "cite" in grounded.system_instruction.lower()
        assert grounded.mode == "normal"

    def test_selected_text_strict_instruction(self):
        result = _make_result("selected_text_only", [_make_chunk()])
        grounded = apply_grounding_policy(result, "Explain this")
        assert grounded.sufficient_context is True
        assert "ONLY" in grounded.system_instruction
        assert "other knowledge" in grounded.system_instruction.lower()
        assert grounded.mode == "selected_text_only"

    def test_context_contains_text(self):
        result = _make_result("normal", [_make_chunk("Answer about Physical AI.")])
        grounded = apply_grounding_policy(result, "Question")
        assert "Answer about Physical AI." in grounded.context

    def test_citations_populated(self):
        result = _make_result("normal", [_make_chunk()])
        grounded = apply_grounding_policy(result, "Question")
        assert len(grounded.citations) == 1
        assert grounded.citations[0].title == "Title"
