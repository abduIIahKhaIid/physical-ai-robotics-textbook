"""Tests for rag.retriever.dedup near-duplicate removal."""

from rag.models import ChunkMetadata, Citation, ScoredChunk
from rag.retriever.dedup import deduplicate


def _make_scored(text: str, score: float, chunk_id: str = "id") -> ScoredChunk:
    return ScoredChunk(
        chunk_id=chunk_id,
        text=text,
        score=score,
        metadata=ChunkMetadata(doc_path="test.md"),
        citation=Citation(title="T", section="S", url="/u"),
    )


class TestDeduplicate:
    def test_removes_near_duplicates(self):
        chunks = [
            _make_scored("The quick brown fox jumps over the lazy dog", 0.95, "a"),
            _make_scored("The quick brown fox jumps over the lazy dog", 0.90, "b"),
        ]
        result = deduplicate(chunks, threshold=0.95)
        assert len(result) == 1
        assert result[0].chunk_id == "a"  # higher score kept

    def test_preserves_unique(self):
        chunks = [
            _make_scored("Physical AI operates in real time", 0.9, "a"),
            _make_scored("ROS2 nodes communicate via topics", 0.85, "b"),
        ]
        result = deduplicate(chunks, threshold=0.95)
        assert len(result) == 2

    def test_maintains_score_order(self):
        chunks = [
            _make_scored("text one unique", 0.7, "a"),
            _make_scored("text two unique", 0.9, "b"),
            _make_scored("text three unique", 0.8, "c"),
        ]
        result = deduplicate(chunks, threshold=0.95)
        assert result[0].score >= result[1].score

    def test_empty_input(self):
        assert deduplicate([], threshold=0.95) == []

    def test_single_chunk(self):
        chunks = [_make_scored("only one", 0.5, "a")]
        result = deduplicate(chunks, threshold=0.95)
        assert len(result) == 1
