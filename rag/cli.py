"""CLI entry point for RAG ingestion and retrieval."""

import asyncio
import json
import logging
import time

import click

from rag.config import load_settings

logger = logging.getLogger(__name__)


@click.group()
@click.option("--verbose", "-v", is_flag=True, help="Enable verbose logging")
def main(verbose: bool):
    """RAG pipeline CLI for Physical AI textbook."""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level, format="%(levelname)s: %(message)s")


@main.command()
@click.argument("docs_dir")
def ingest(docs_dir: str):
    """Ingest all markdown documents from DOCS_DIR into Qdrant."""
    from rag.parser.discovery import discover_documents
    from rag.parser.frontmatter import parse_frontmatter
    from rag.parser.markdown_splitter import split_markdown
    from rag.chunker.heading_chunker import chunk_blocks
    from rag.chunker.overlap import add_overlaps
    from rag.chunker.id_generator import generate_ids
    from rag.embedder.openai_embedder import batch_embed
    from rag.store.qdrant_store import ensure_collection, upsert_chunks, delete_stale_chunks

    start = time.time()
    settings = load_settings()

    click.echo(f"Discovering documents in {docs_dir}...")
    docs = discover_documents(docs_dir)
    click.echo(f"Found {len(docs)} documents")

    ensure_collection()

    total_chunks = 0
    total_deleted = 0
    errors: list[str] = []

    for doc in docs:
        try:
            doc = parse_frontmatter(doc)
            blocks = split_markdown(doc.content)
            chunks = chunk_blocks(
                blocks,
                doc,
                min_tokens=settings.chunk_min_tokens,
                max_tokens=settings.chunk_max_tokens,
                target_tokens=settings.chunk_target_tokens,
            )
            chunks = add_overlaps(chunks, overlap_tokens=settings.chunk_overlap_tokens)
            chunks = generate_ids(chunks)
            chunks = asyncio.run(batch_embed(chunks))

            upsert_chunks(chunks)
            current_ids = {c.id for c in chunks}
            deleted = delete_stale_chunks(doc.doc_path, current_ids)

            total_chunks += len(chunks)
            total_deleted += deleted
            click.echo(f"  {doc.doc_path}: {len(chunks)} chunks")
        except Exception as e:
            errors.append(f"{doc.doc_path}: {e}")
            click.echo(f"  ERROR: {doc.doc_path}: {e}", err=True)

    duration = time.time() - start
    click.echo(f"\nIngestion complete:")
    click.echo(f"  Documents: {len(docs)}")
    click.echo(f"  Chunks created: {total_chunks}")
    click.echo(f"  Stale chunks deleted: {total_deleted}")
    click.echo(f"  Errors: {len(errors)}")
    click.echo(f"  Duration: {duration:.1f}s")


@main.command("ingest-doc")
@click.argument("doc_path")
@click.argument("docs_dir")
def ingest_doc(doc_path: str, docs_dir: str):
    """Ingest a single document by relative path within DOCS_DIR."""
    from pathlib import Path
    from rag.models import Document, ContentType
    from rag.parser.frontmatter import parse_frontmatter
    from rag.parser.markdown_splitter import split_markdown
    from rag.chunker.heading_chunker import chunk_blocks
    from rag.chunker.overlap import add_overlaps
    from rag.chunker.id_generator import generate_ids
    from rag.embedder.openai_embedder import batch_embed
    from rag.store.qdrant_store import ensure_collection, upsert_chunks, delete_stale_chunks

    abs_path = Path(docs_dir) / doc_path
    if not abs_path.exists():
        click.echo(f"File not found: {abs_path}", err=True)
        raise SystemExit(1)

    settings = load_settings()
    doc = Document(doc_path=doc_path, absolute_path=str(abs_path))
    doc = parse_frontmatter(doc)
    blocks = split_markdown(doc.content)
    chunks = chunk_blocks(blocks, doc)
    chunks = add_overlaps(chunks)
    chunks = generate_ids(chunks)
    chunks = asyncio.run(batch_embed(chunks))

    ensure_collection()
    upsert_chunks(chunks)
    current_ids = {c.id for c in chunks}
    delete_stale_chunks(doc.doc_path, current_ids)

    click.echo(f"Ingested {len(chunks)} chunks from {doc_path}")


@main.command()
@click.argument("question")
@click.option("--mode", default="normal", help="Query mode: normal or selected_text_only")
@click.option("--selected-text", default=None, help="Selected text for selected_text_only mode")
@click.option("--top-k", default=5, help="Number of results")
@click.option("--module", default=None, help="Filter by module")
def query(question: str, mode: str, selected_text: str, top_k: int, module: str):
    """Query the textbook knowledge base."""
    from rag.models import Query, QueryFilters
    from rag.retriever.query_engine import retrieve
    from rag.response.grounding import apply_grounding_policy

    filters = QueryFilters(top_k=top_k)
    if module:
        filters.modules = [module]

    q = Query(
        question=question,
        mode=mode,
        selected_text=selected_text,
        filters=filters,
    )

    result = retrieve(q)
    grounded = apply_grounding_policy(result, question)

    click.echo(f"\nMode: {grounded.mode}")
    click.echo(f"Sufficient context: {grounded.sufficient_context}")
    click.echo(f"\nSystem instruction:\n{grounded.system_instruction}")
    click.echo(f"\nContext:\n{grounded.context}")
    click.echo(f"\nCitations:")
    for c in grounded.citations:
        click.echo(f"  - {c.title} ({c.section}): {c.url}")


@main.command()
@click.option("--test-set", default=None, help="Path to test set JSON")
def evaluate(test_set: str):
    """Run the evaluation harness against the test set."""
    from rag.eval.eval_harness import run_evaluation

    click.echo("Running evaluation harness...")
    report = run_evaluation(test_set)

    click.echo(f"\nResults: {report.passed}/{report.total} passed ({report.pass_rate:.0%})")
    click.echo(f"Failed: {report.failed}")
    click.echo("")

    for r in report.results:
        status = "PASS" if r.passed else "FAIL"
        click.echo(f"  [{status}] {r.query_id} ({r.category}): {r.details}")
