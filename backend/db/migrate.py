"""Simple SQL migration runner for the chat backend."""

import asyncio
import logging
import os
from pathlib import Path

import asyncpg

logger = logging.getLogger(__name__)

MIGRATIONS_DIR = Path(__file__).parent.parent / "migrations"


async def run_migrations(database_url: str) -> list[str]:
    """Apply pending SQL migrations.

    Returns list of applied migration versions.
    """
    conn = await asyncpg.connect(dsn=database_url)
    applied = []
    try:
        # Create schema_migrations table if not exists
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS schema_migrations (
                version VARCHAR(20) PRIMARY KEY,
                applied_at TIMESTAMPTZ NOT NULL DEFAULT now()
            )
        """)

        # Get already-applied versions
        rows = await conn.fetch("SELECT version FROM schema_migrations")
        existing = {row["version"] for row in rows}

        # Read and sort migration files
        migration_files = sorted(MIGRATIONS_DIR.glob("*.sql"))

        for migration_file in migration_files:
            version = migration_file.stem.split("_")[0]  # e.g., "001"
            if version in existing:
                logger.info(f"Skipping already-applied migration: {version}")
                continue

            sql = migration_file.read_text()
            logger.info(f"Applying migration {version}: {migration_file.name}")

            async with conn.transaction():
                await conn.execute(sql)
                await conn.execute(
                    "INSERT INTO schema_migrations (version) VALUES ($1)",
                    version,
                )
            applied.append(version)
            logger.info(f"Migration {version} applied successfully")

    finally:
        await conn.close()

    return applied


def main() -> None:
    """CLI entry point for running migrations."""
    logging.basicConfig(level=logging.INFO)
    database_url = os.environ.get("DATABASE_URL", "")
    if not database_url:
        logger.error("DATABASE_URL environment variable is required")
        raise SystemExit(1)

    applied = asyncio.run(run_migrations(database_url))
    if applied:
        logger.info(f"Applied {len(applied)} migration(s): {applied}")
    else:
        logger.info("No pending migrations")


if __name__ == "__main__":
    main()
