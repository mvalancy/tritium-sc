# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the /api/version endpoint."""

import pytest
from unittest.mock import MagicMock


@pytest.mark.unit
class TestVersionRouter:
    def test_version_info_structure(self):
        from app.routers.version import API_VERSION_INFO
        assert API_VERSION_INFO["api_version"] == "v1"
        assert API_VERSION_INFO["app"] == "TRITIUM-SC"
        assert "v1" in API_VERSION_INFO["supported_versions"]

    @pytest.mark.asyncio
    async def test_api_version_endpoint(self):
        from app.routers.version import api_version
        result = await api_version()
        assert result["api_version"] == "v1"
        assert result["app_version"] == "0.1.0"
        assert "server_boot" in result

    @pytest.mark.asyncio
    async def test_api_v1_version_endpoint(self):
        from app.routers.version import api_v1_version
        result = await api_v1_version()
        assert result["namespace"] == "/api/v1"
        assert result["api_version"] == "v1"
