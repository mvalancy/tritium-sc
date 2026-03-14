# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for authentication module."""

import os
import pytest


# Ensure auth is disabled by default for tests
os.environ.setdefault("AUTH_ENABLED", "false")
os.environ.setdefault("AUTH_SECRET_KEY", "test-secret-key-32-chars-long-ok")


class TestAuthModule:
    """Test auth module imports and basic functions."""

    def test_import(self):
        from app.auth import (
            authenticate_user,
            create_access_token,
            create_refresh_token,
            decode_token,
            init_default_admin,
            optional_auth,
            require_auth,
        )
        assert callable(create_access_token)
        assert callable(decode_token)
        assert callable(init_default_admin)

    def test_password_hashing(self):
        from app.auth import _hash_password, _verify_password
        hashed = _hash_password("test123")
        assert ":" in hashed
        assert _verify_password("test123", hashed)
        assert not _verify_password("wrong", hashed)

    def test_create_access_token(self):
        from app.auth import create_access_token, decode_token
        token = create_access_token("testuser", "admin")
        assert isinstance(token, str)
        assert len(token) > 20

        payload = decode_token(token)
        assert payload["sub"] == "testuser"
        assert payload["role"] == "admin"
        assert "jti" in payload
        assert "exp" in payload

    def test_create_refresh_token(self):
        from app.auth import create_refresh_token, decode_token
        token = create_refresh_token("testuser")
        payload = decode_token(token)
        assert payload["sub"] == "testuser"
        assert payload["type"] == "refresh"

    def test_authenticate_user_no_users(self):
        from app.auth import authenticate_user
        result = authenticate_user("nobody", "nopass")
        assert result is None

    def test_init_default_admin(self):
        from app.auth import _users, authenticate_user, init_default_admin
        from app.config import settings

        # Save original values
        orig_enabled = settings.auth_enabled
        orig_password = settings.auth_admin_password

        try:
            settings.auth_enabled = True
            settings.auth_admin_password = "testpass"
            init_default_admin()

            result = authenticate_user("admin", "testpass")
            assert result is not None
            assert result["sub"] == "admin"
            assert result["role"] == "admin"

            # Wrong password should fail
            assert authenticate_user("admin", "wrong") is None
        finally:
            settings.auth_enabled = orig_enabled
            settings.auth_admin_password = orig_password
            _users.clear()


class TestAuthRouter:
    """Test auth API router."""

    def test_import(self):
        from app.routers.auth import router
        assert router is not None

    def test_auth_status_endpoint(self):
        """Auth status endpoint should be available."""
        from app.routers.auth import auth_status
        import asyncio
        result = asyncio.get_event_loop().run_until_complete(auth_status())
        assert "auth_enabled" in result
