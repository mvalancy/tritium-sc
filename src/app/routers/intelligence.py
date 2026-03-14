# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Intelligence model management API.

Endpoints for retraining the correlation model and querying model status.
Also includes Ollama-powered anomaly description for RF events.

Endpoints:
    POST /api/intelligence/retrain      — Trigger model retraining
    GET  /api/intelligence/model/status  — Model accuracy, training count, timestamps
    POST /api/intelligence/anomaly/describe — Ollama-powered anomaly description
"""
from __future__ import annotations

import time
from typing import Any, Optional

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field
from loguru import logger

router = APIRouter(prefix="/api/intelligence", tags=["intelligence"])


class RetrainResponse(BaseModel):
    """Response from model retrain endpoint."""
    success: bool
    accuracy: float = 0.0
    training_count: int = 0
    error: Optional[str] = None
    feature_names: list[str] = Field(default_factory=list)


class ModelStatusResponse(BaseModel):
    """Current model status."""
    trained: bool = False
    accuracy: float = 0.0
    training_count: int = 0
    last_trained: Optional[float] = None
    last_trained_iso: Optional[str] = None
    sklearn_available: bool = False
    model_path: str = ""
    feature_names: list[str] = Field(default_factory=list)
    training_data_stats: dict[str, Any] = Field(default_factory=dict)


class AnomalyDescribeRequest(BaseModel):
    """Request to describe an anomaly using LLM."""
    anomaly_type: str = Field(..., description="Type of anomaly (rf_drop, rf_spike, device_loss)")
    context: dict[str, Any] = Field(
        default_factory=dict,
        description="Anomaly context: device counts, RSSI values, timestamps, etc.",
    )
    ollama_model: str = Field("qwen2.5:7b", description="Ollama model to use")


class AnomalyDescribeResponse(BaseModel):
    """LLM-generated anomaly description."""
    description: str
    severity: str = "unknown"
    suggested_action: str = ""
    model_used: str = ""
    generated: bool = True


@router.post("/retrain", response_model=RetrainResponse)
async def retrain_model(request: Request) -> RetrainResponse:
    """Trigger retraining of the correlation model from accumulated training data.

    Loads all confirmed correlation decisions from the TrainingStore,
    trains a logistic regression model, and saves it to disk. Returns
    the model accuracy and training data count.
    """
    try:
        from engine.intelligence.correlation_learner import get_correlation_learner
        learner = get_correlation_learner()
        result = learner.train()

        return RetrainResponse(
            success=result.get("success", False),
            accuracy=result.get("accuracy", 0.0),
            training_count=result.get("training_count", 0),
            error=result.get("error"),
            feature_names=result.get("feature_names", []),
        )
    except Exception as exc:
        logger.error("Retrain endpoint failed: %s", exc)
        return RetrainResponse(success=False, error=str(exc))


@router.get("/model/status", response_model=ModelStatusResponse)
async def model_status(request: Request) -> ModelStatusResponse:
    """Get current model accuracy, training data count, and last trained timestamp."""
    try:
        from engine.intelligence.correlation_learner import get_correlation_learner
        learner = get_correlation_learner()
        status = learner.get_status()

        # Also get training data stats
        training_stats: dict[str, Any] = {}
        try:
            from engine.intelligence.training_store import get_training_store
            store = get_training_store()
            training_stats = store.get_stats()
        except Exception:
            pass

        return ModelStatusResponse(
            trained=status.get("trained", False),
            accuracy=status.get("accuracy", 0.0),
            training_count=status.get("training_count", 0),
            last_trained=status.get("last_trained"),
            last_trained_iso=status.get("last_trained_iso"),
            sklearn_available=status.get("sklearn_available", False),
            model_path=status.get("model_path", ""),
            feature_names=status.get("feature_names", []),
            training_data_stats=training_stats,
        )
    except Exception as exc:
        logger.error("Model status endpoint failed: %s", exc)
        return ModelStatusResponse()


@router.post("/anomaly/describe", response_model=AnomalyDescribeResponse)
async def describe_anomaly(req: AnomalyDescribeRequest) -> AnomalyDescribeResponse:
    """Use a local Ollama LLM to describe an anomaly in natural language.

    When the anomaly detector flags unusual RF activity, this endpoint
    generates a human-readable description of what is unusual and why
    it matters. Requires a running Ollama instance.

    Example output: "BLE device count dropped from 12 to 3 in the last
    5 minutes -- possible jamming or evacuation."
    """
    prompt = _build_anomaly_prompt(req.anomaly_type, req.context)

    try:
        description = await _query_ollama(prompt, req.ollama_model)

        # Extract severity from description
        severity = _classify_severity(req.anomaly_type, req.context)
        suggested_action = _suggest_action(req.anomaly_type, req.context)

        return AnomalyDescribeResponse(
            description=description,
            severity=severity,
            suggested_action=suggested_action,
            model_used=req.ollama_model,
            generated=True,
        )
    except Exception as exc:
        # Fallback to template-based description
        logger.warning("Ollama unavailable, using template: %s", exc)
        fallback = _template_description(req.anomaly_type, req.context)
        return AnomalyDescribeResponse(
            description=fallback,
            severity=_classify_severity(req.anomaly_type, req.context),
            suggested_action=_suggest_action(req.anomaly_type, req.context),
            model_used="template",
            generated=False,
        )


def _build_anomaly_prompt(anomaly_type: str, context: dict[str, Any]) -> str:
    """Build a prompt for the LLM to describe an anomaly."""
    ctx_lines = []
    for k, v in context.items():
        ctx_lines.append(f"  {k}: {v}")
    ctx_str = "\n".join(ctx_lines) if ctx_lines else "  (no additional context)"

    return f"""You are a cybersecurity analyst monitoring an RF sensor network.
An anomaly has been detected. Describe what is happening in 1-2 sentences,
using plain language that a security operator would understand. Include
possible explanations and severity.

Anomaly type: {anomaly_type}
Context:
{ctx_str}

Respond with ONLY the description, no headers or formatting."""


async def _query_ollama(prompt: str, model: str) -> str:
    """Query local Ollama instance for anomaly description."""
    import httpx

    async with httpx.AsyncClient(timeout=30.0) as client:
        response = await client.post(
            "http://localhost:11434/api/generate",
            json={
                "model": model,
                "prompt": prompt,
                "stream": False,
                "options": {
                    "temperature": 0.3,
                    "num_predict": 150,
                },
            },
        )
        response.raise_for_status()
        data = response.json()
        return data.get("response", "").strip()


def _classify_severity(anomaly_type: str, context: dict[str, Any]) -> str:
    """Classify anomaly severity based on type and context."""
    severity_map = {
        "rf_drop": "warning",
        "rf_spike": "warning",
        "device_loss": "critical",
        "jamming_suspected": "critical",
        "mass_departure": "warning",
        "new_device_flood": "warning",
        "rssi_anomaly": "info",
    }

    severity = severity_map.get(anomaly_type, "info")

    # Escalate based on magnitude
    magnitude = context.get("magnitude", 0)
    if isinstance(magnitude, (int, float)) and magnitude > 0.8:
        if severity == "info":
            severity = "warning"
        elif severity == "warning":
            severity = "critical"

    return severity


def _suggest_action(anomaly_type: str, context: dict[str, Any]) -> str:
    """Suggest a response action based on anomaly type."""
    actions = {
        "rf_drop": "Investigate area for RF interference or jamming. Check sensor health.",
        "rf_spike": "Monitor for unauthorized transmitter. Run spectrum analysis.",
        "device_loss": "Verify sensor connectivity. Check for power outage or physical damage.",
        "jamming_suspected": "Activate counter-jamming protocols. Alert security team. Switch to backup frequencies.",
        "mass_departure": "Review camera feeds for evacuation. Check for active threats.",
        "new_device_flood": "Run device fingerprinting. Check for spoofing attack.",
        "rssi_anomaly": "Log for pattern analysis. No immediate action required.",
    }
    return actions.get(anomaly_type, "Monitor and log. Investigate if pattern persists.")


def _template_description(anomaly_type: str, context: dict[str, Any]) -> str:
    """Generate a template-based description when Ollama is unavailable."""
    templates = {
        "rf_drop": (
            "RF activity dropped significantly. "
            "Device count went from {prev_count} to {current_count} "
            "in the last {window_minutes} minutes. "
            "Possible causes: jamming, power outage, or evacuation."
        ),
        "rf_spike": (
            "Unusual spike in RF activity detected. "
            "{current_count} devices now visible (was {prev_count}). "
            "Could indicate new device deployment or spoofing attack."
        ),
        "device_loss": (
            "Sensor node went offline. Device {device_id} has not "
            "reported in {minutes_silent} minutes. "
            "Check physical connectivity and power."
        ),
        "jamming_suspected": (
            "Possible RF jamming detected. Multiple sensors report "
            "degraded signal quality simultaneously. "
            "{affected_count} nodes affected."
        ),
        "mass_departure": (
            "Large number of tracked devices departed the area. "
            "{departed_count} devices lost in {window_minutes} minutes."
        ),
        "new_device_flood": (
            "Rapid appearance of {new_count} new devices. "
            "Unusual pattern may indicate spoofing or large group arrival."
        ),
    }

    template = templates.get(anomaly_type, "Anomaly detected: {anomaly_type}. Investigating.")

    # Fill template with context values, using defaults for missing keys
    defaults = {
        "prev_count": "?",
        "current_count": "?",
        "window_minutes": "5",
        "device_id": "unknown",
        "minutes_silent": "?",
        "affected_count": "?",
        "departed_count": "?",
        "new_count": "?",
        "anomaly_type": anomaly_type,
    }
    merged = {**defaults, **{k: str(v) for k, v in context.items()}}

    try:
        return template.format(**merged)
    except (KeyError, IndexError):
        return f"Anomaly detected: {anomaly_type}. Context: {context}"
