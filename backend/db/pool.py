"""Asyncpg connection pool lifecycle management."""

import asyncpg


async def create_pool(database_url: str) -> asyncpg.Pool:
    """Create an asyncpg connection pool."""
    return await asyncpg.create_pool(
        dsn=database_url,
        min_size=2,
        max_size=20,
        command_timeout=30,
    )


async def close_pool(pool: asyncpg.Pool) -> None:
    """Close the connection pool gracefully."""
    await pool.close()
