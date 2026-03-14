# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Rate limiting middleware for production deployment.

Uses a sliding window counter per IP address. Only active when
rate_limit_enabled=True in config. Exempt paths include WebSocket
and health check endpoints.
"""

import time
from collections import defaultdict
from typing import Optional

from fastapi import Request, Response
from loguru import logger
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint

from app.config import settings


# Exempt paths — never rate limited
EXEMPT_PATHS = {
    "/ws/live",
    "/api/auth/status",
    "/health",
    "/",
}

# Exempt prefixes
EXEMPT_PREFIXES = (
    "/static/",
    "/frontend/",
    "/ws/",
)


class RateLimitEntry:
    """Sliding window rate limit tracker for a single IP."""

    __slots__ = ("requests", "window_start")

    def __init__(self) -> None:
        self.requests: int = 0
        self.window_start: float = time.monotonic()

    def check(self, max_requests: int, window_seconds: int) -> tuple[bool, int]:
        """Check if request is allowed. Returns (allowed, remaining)."""
        now = time.monotonic()
        elapsed = now - self.window_start

        if elapsed > window_seconds:
            # Window expired — reset
            self.requests = 1
            self.window_start = now
            return True, max_requests - 1

        self.requests += 1
        remaining = max(0, max_requests - self.requests)
        return self.requests <= max_requests, remaining


class RateLimitMiddleware(BaseHTTPMiddleware):
    """Rate limiting middleware using per-IP sliding window counters."""

    def __init__(self, app, max_requests: int = 100, window_seconds: int = 60):
        super().__init__(app)
        self.max_requests = max_requests
        self.window_seconds = window_seconds
        self._entries: dict[str, RateLimitEntry] = defaultdict(RateLimitEntry)
        self._cleanup_counter = 0

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        # Skip if rate limiting disabled
        if not settings.rate_limit_enabled:
            return await call_next(request)

        # Check exempt paths
        path = request.url.path
        if path in EXEMPT_PATHS or path.startswith(EXEMPT_PREFIXES):
            return await call_next(request)

        # Get client IP
        client_ip = self._get_client_ip(request)
        entry = self._entries[client_ip]

        allowed, remaining = entry.check(self.max_requests, self.window_seconds)

        if not allowed:
            logger.warning(f"Rate limit exceeded for {client_ip} on {path}")
            return Response(
                content='{"detail": "Rate limit exceeded. Try again later."}',
                status_code=429,
                media_type="application/json",
                headers={
                    "X-RateLimit-Limit": str(self.max_requests),
                    "X-RateLimit-Remaining": "0",
                    "X-RateLimit-Reset": str(int(entry.window_start + self.window_seconds)),
                    "Retry-After": str(self.window_seconds),
                },
            )

        response = await call_next(request)

        # Add rate limit headers
        response.headers["X-RateLimit-Limit"] = str(self.max_requests)
        response.headers["X-RateLimit-Remaining"] = str(remaining)

        # Periodic cleanup of stale entries
        self._cleanup_counter += 1
        if self._cleanup_counter >= 1000:
            self._cleanup()
            self._cleanup_counter = 0

        return response

    def _get_client_ip(self, request: Request) -> str:
        """Get client IP from request, respecting X-Forwarded-For."""
        forwarded = request.headers.get("X-Forwarded-For")
        if forwarded:
            return forwarded.split(",")[0].strip()
        return request.client.host if request.client else "unknown"

    def _cleanup(self) -> None:
        """Remove stale rate limit entries."""
        now = time.monotonic()
        stale = [
            ip for ip, entry in self._entries.items()
            if (now - entry.window_start) > self.window_seconds * 2
        ]
        for ip in stale:
            del self._entries[ip]
