"""Unit tests for the sliding window rate limiter."""

from __future__ import annotations

import time
from unittest.mock import patch

import pytest

from backend.services.rate_limiter import SlidingWindowRateLimiter


class TestSlidingWindowRateLimiter:
    """Test rate limiter logic."""

    @pytest.mark.asyncio
    async def test_allows_requests_under_limit(self):
        limiter = SlidingWindowRateLimiter(window_seconds=60)
        for i in range(5):
            allowed, retry_after = await limiter.check("user1", 10)
            assert allowed is True
            assert retry_after == 0

    @pytest.mark.asyncio
    async def test_blocks_at_limit_plus_one(self):
        limiter = SlidingWindowRateLimiter(window_seconds=60)
        for i in range(10):
            allowed, _ = await limiter.check("user1", 10)
            assert allowed is True

        # 11th request should be blocked
        allowed, retry_after = await limiter.check("user1", 10)
        assert allowed is False
        assert retry_after >= 1

    @pytest.mark.asyncio
    async def test_returns_positive_retry_after(self):
        limiter = SlidingWindowRateLimiter(window_seconds=60)
        for i in range(10):
            await limiter.check("user1", 10)

        allowed, retry_after = await limiter.check("user1", 10)
        assert allowed is False
        assert retry_after > 0

    @pytest.mark.asyncio
    async def test_resets_after_window_expires(self):
        limiter = SlidingWindowRateLimiter(window_seconds=1)

        # Fill up the limit
        for i in range(5):
            await limiter.check("user1", 5)

        # Should be blocked
        allowed, _ = await limiter.check("user1", 5)
        assert allowed is False

        # Fast-forward time past window
        with patch("backend.services.rate_limiter.time.monotonic", return_value=time.monotonic() + 2):
            allowed, retry_after = await limiter.check("user1", 5)
            assert allowed is True
            assert retry_after == 0

    @pytest.mark.asyncio
    async def test_tracks_different_identifiers_independently(self):
        limiter = SlidingWindowRateLimiter(window_seconds=60)

        # Fill user1's limit
        for i in range(10):
            await limiter.check("user1", 10)

        # user1 blocked
        allowed, _ = await limiter.check("user1", 10)
        assert allowed is False

        # user2 still allowed
        allowed, _ = await limiter.check("user2", 10)
        assert allowed is True

    @pytest.mark.asyncio
    async def test_anonymous_vs_identified_limits(self):
        """Anonymous (10/min) and identified (20/min) limits differ."""
        limiter = SlidingWindowRateLimiter(window_seconds=60)

        # Anonymous user: 10 requests allowed
        for i in range(10):
            allowed, _ = await limiter.check("anon-ip", 10)
            assert allowed is True

        allowed, _ = await limiter.check("anon-ip", 10)
        assert allowed is False

        # Identified user: 20 requests allowed
        for i in range(20):
            allowed, _ = await limiter.check("identified-user", 20)
            assert allowed is True

        allowed, _ = await limiter.check("identified-user", 20)
        assert allowed is False
