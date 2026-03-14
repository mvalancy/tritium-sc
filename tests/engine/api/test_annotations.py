# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for map annotations API."""

import pytest
from unittest.mock import patch


@pytest.fixture
def _clear_store():
    """Reset annotation store before each test."""
    from app.routers.annotations import _annotations
    _annotations.clear()
    yield
    _annotations.clear()


class TestAnnotationsAPI:
    """Test annotation CRUD operations."""

    @pytest.mark.unit
    def test_create_text_annotation(self, _clear_store):
        from app.routers.annotations import _annotations
        from app.routers.annotations import AnnotationCreate

        body = AnnotationCreate(
            type="text",
            lat=33.12,
            lng=-97.45,
            text="Rally point Alpha",
            color="#ff2a6d",
        )
        # Verify model creation
        assert body.type == "text"
        assert body.lat == 33.12
        assert body.text == "Rally point Alpha"

    @pytest.mark.unit
    def test_create_circle_annotation(self, _clear_store):
        from app.routers.annotations import AnnotationCreate

        body = AnnotationCreate(
            type="circle",
            lat=33.12,
            lng=-97.45,
            radius_m=100.0,
            fill=True,
        )
        assert body.type == "circle"
        assert body.radius_m == 100.0
        assert body.fill is True

    @pytest.mark.unit
    def test_create_arrow_annotation(self, _clear_store):
        from app.routers.annotations import AnnotationCreate

        body = AnnotationCreate(
            type="arrow",
            lat=33.12,
            lng=-97.45,
            end_lat=33.13,
            end_lng=-97.44,
            color="#05ffa1",
        )
        assert body.type == "arrow"
        assert body.end_lat == 33.13

    @pytest.mark.unit
    def test_create_freehand_annotation(self, _clear_store):
        from app.routers.annotations import AnnotationCreate

        body = AnnotationCreate(
            type="freehand",
            lat=33.12,
            lng=-97.45,
            points=[[33.12, -97.45], [33.13, -97.44], [33.14, -97.43]],
        )
        assert body.type == "freehand"
        assert len(body.points) == 3

    @pytest.mark.unit
    def test_annotation_defaults(self, _clear_store):
        from app.routers.annotations import AnnotationCreate

        body = AnnotationCreate(type="text", lat=0, lng=0)
        assert body.color == "#00f0ff"
        assert body.stroke_width == 2.0
        assert body.opacity == 0.8
        assert body.layer == "default"
        assert body.locked is False


class TestAnnotationUpdate:
    """Test annotation update model."""

    @pytest.mark.unit
    def test_partial_update(self):
        from app.routers.annotations import AnnotationUpdate

        body = AnnotationUpdate(text="Updated text", color="#ff00ff")
        updates = body.model_dump(exclude_none=True)
        assert updates == {"text": "Updated text", "color": "#ff00ff"}

    @pytest.mark.unit
    def test_empty_update(self):
        from app.routers.annotations import AnnotationUpdate

        body = AnnotationUpdate()
        updates = body.model_dump(exclude_none=True)
        assert updates == {}
