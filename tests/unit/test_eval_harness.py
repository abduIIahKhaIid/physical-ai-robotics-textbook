"""Tests for the evaluation harness."""

from rag.eval.eval_harness import (
    evaluate_normal_query_offline,
    evaluate_selected_text_query,
    load_test_set,
    run_evaluation,
)


class TestLoadTestSet:
    def test_loads_default(self):
        queries = load_test_set()
        assert len(queries) == 20
        assert all("id" in q for q in queries)
        assert all("question" in q for q in queries)

    def test_has_both_modes(self):
        queries = load_test_set()
        modes = {q["mode"] for q in queries}
        assert "normal" in modes
        assert "selected_text_only" in modes

    def test_has_leak_tests(self):
        queries = load_test_set()
        leak_tests = [q for q in queries if q.get("category") == "selection_leak_test"]
        assert len(leak_tests) >= 4


class TestEvaluateSelectedText:
    def test_normal_selection_passes(self):
        entry = {
            "id": "test-sel-1",
            "question": "Explain this",
            "mode": "selected_text_only",
            "selected_text": "Physical AI operates in real-time.",
            "category": "selection_normal",
        }
        result = evaluate_selected_text_query(entry)
        assert result.passed
        assert result.chunks_returned == 1

    def test_leak_test_still_passes_structurally(self):
        entry = {
            "id": "test-leak-1",
            "question": "What is quantum computing?",
            "mode": "selected_text_only",
            "selected_text": "Physical AI operates in real-time.",
            "expected_refuse": True,
            "category": "selection_leak_test",
        }
        result = evaluate_selected_text_query(entry)
        # Structural check: selected text was returned, strict instruction applied
        assert result.passed
        assert result.grounded  # has context (the selection)


class TestEvaluateNormalQuery:
    def test_basic_normal_query(self):
        entry = {
            "id": "test-norm-1",
            "question": "What are the core principles?",
            "mode": "normal",
            "expected_modules": ["module-1"],
            "category": "factual",
        }
        result = evaluate_normal_query_offline(entry)
        assert result.passed

    def test_no_module_filter(self):
        entry = {
            "id": "test-norm-2",
            "question": "How do ROS2 nodes work?",
            "mode": "normal",
            "category": "technical",
        }
        result = evaluate_normal_query_offline(entry)
        assert result.passed


class TestRunEvaluation:
    def test_full_run(self):
        report = run_evaluation()
        assert report.total == 20
        assert report.passed > 0
        assert report.pass_rate > 0
