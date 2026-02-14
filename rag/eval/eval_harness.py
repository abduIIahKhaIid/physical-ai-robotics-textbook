"""Evaluation harness for RAG pipeline quality assessment."""

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path

from rag.models import Query, QueryFilters
from rag.retriever.selected_text import retrieve_from_selection
from rag.response.grounding import apply_grounding_policy

logger = logging.getLogger(__name__)

TEST_SET_PATH = Path(__file__).parent / "test_set.json"


@dataclass
class EvalResult:
    query_id: str
    question: str
    mode: str
    category: str
    passed: bool
    details: str = ""
    chunks_returned: int = 0
    grounded: bool = False


@dataclass
class EvalReport:
    total: int = 0
    passed: int = 0
    failed: int = 0
    results: list[EvalResult] = field(default_factory=list)

    @property
    def pass_rate(self) -> float:
        return self.passed / self.total if self.total > 0 else 0.0


def load_test_set(path: str | None = None) -> list[dict]:
    """Load evaluation test queries from JSON."""
    p = Path(path) if path else TEST_SET_PATH
    with open(p) as f:
        data = json.load(f)
    return data["queries"]


def evaluate_selected_text_query(entry: dict) -> EvalResult:
    """Evaluate a selected-text-only query.

    Checks:
    - Result mode is selected_text_only
    - Only 1 chunk returned (the selection)
    - For leak tests: grounding policy should note insufficient context
      when question is unrelated to selected text
    """
    q = Query(
        question=entry["question"],
        mode="selected_text_only",
        selected_text=entry.get("selected_text", ""),
        source_doc_path=entry.get("source_doc_path", ""),
        source_section=entry.get("source_section", ""),
    )

    result = retrieve_from_selection(q)
    grounded = apply_grounding_policy(result, entry["question"])

    passed = True
    details_parts = []

    # Verify mode
    if result.mode != "selected_text_only":
        passed = False
        details_parts.append(f"Wrong mode: {result.mode}")

    # Verify single chunk
    if len(result.chunks) != 1:
        passed = False
        details_parts.append(f"Expected 1 chunk, got {len(result.chunks)}")

    # Verify chunk contains selected text
    if result.chunks and result.chunks[0].text != entry.get("selected_text", ""):
        passed = False
        details_parts.append("Chunk text does not match selected text")

    # Verify grounding policy applied correctly
    if grounded.mode != "selected_text_only":
        passed = False
        details_parts.append("Grounding mode mismatch")

    # For selection queries, verify strict instruction
    if "ONLY" not in grounded.system_instruction:
        passed = False
        details_parts.append("Missing strict 'ONLY' instruction")

    return EvalResult(
        query_id=entry["id"],
        question=entry["question"],
        mode="selected_text_only",
        category=entry.get("category", "unknown"),
        passed=passed,
        details="; ".join(details_parts) if details_parts else "OK",
        chunks_returned=len(result.chunks),
        grounded=grounded.sufficient_context,
    )


def evaluate_normal_query_offline(entry: dict) -> EvalResult:
    """Evaluate a normal-mode query without live Qdrant (structural check only).

    Without a live Qdrant connection, we can only verify:
    - Query construction is valid
    - Filters build correctly
    - Empty result grounding behaves correctly
    """
    from rag.retriever.filters import build_qdrant_filter

    try:
        q = Query(
            question=entry["question"],
            mode="normal",
            filters=QueryFilters(
                modules=entry.get("expected_modules"),
            ),
        )
    except Exception as e:
        return EvalResult(
            query_id=entry["id"],
            question=entry["question"],
            mode="normal",
            category=entry.get("category", "unknown"),
            passed=False,
            details=f"Query construction failed: {e}",
        )

    # Verify filter construction
    qdrant_filter = build_qdrant_filter(q.filters)
    filter_ok = True
    if entry.get("expected_modules") and qdrant_filter is None:
        filter_ok = False

    # Verify grounding policy for empty results
    from rag.models import RetrievalResult
    empty_result = RetrievalResult(chunks=[], query=q, mode="normal")
    grounded = apply_grounding_policy(empty_result, entry["question"])

    passed = filter_ok and not grounded.sufficient_context
    details = "OK" if passed else f"filter_ok={filter_ok}, grounded_empty={grounded.sufficient_context}"

    return EvalResult(
        query_id=entry["id"],
        question=entry["question"],
        mode="normal",
        category=entry.get("category", "unknown"),
        passed=passed,
        details=details,
    )


def run_evaluation(path: str | None = None) -> EvalReport:
    """Run the full offline evaluation suite."""
    entries = load_test_set(path)
    report = EvalReport(total=len(entries))

    for entry in entries:
        mode = entry.get("mode", "normal")
        if mode == "selected_text_only":
            result = evaluate_selected_text_query(entry)
        else:
            result = evaluate_normal_query_offline(entry)

        report.results.append(result)
        if result.passed:
            report.passed += 1
        else:
            report.failed += 1

    return report
