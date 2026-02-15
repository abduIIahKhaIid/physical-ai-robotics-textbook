"""In-memory sliding window rate limiter."""

from __future__ import annotations

import asyncio
import time


class SlidingWindowRateLimiter:
    """Sliding window rate limiter using in-memory timestamp tracking.

    Thread-safe via asyncio.Lock. Suitable for single-instance deployment.
    """

    def __init__(self, window_seconds: int = 60):
        self._window_seconds = window_seconds
        self._windows: dict[str, list[float]] = {}
        self._lock = asyncio.Lock()

    async def check(self, identifier: str, limit: int) -> tuple[bool, int]:
        """Check if a request is allowed under the rate limit.

        Args:
            identifier: User ID or IP address.
            limit: Maximum requests per window.

        Returns:
            Tuple of (allowed, retry_after_seconds).
            If allowed is True, retry_after is 0.
        """
        now = time.monotonic()
        window_start = now - self._window_seconds

        async with self._lock:
            # Get or create window
            timestamps = self._windows.get(identifier, [])

            # Prune expired entries
            timestamps = [t for t in timestamps if t > window_start]

            if len(timestamps) >= limit:
                # Rate limited — calculate retry_after
                oldest = timestamps[0]
                retry_after = int(oldest - window_start) + 1
                self._windows[identifier] = timestamps
                return False, max(retry_after, 1)

            # Allowed — record this request
            timestamps.append(now)
            self._windows[identifier] = timestamps
            return True, 0
