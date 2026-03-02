# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Structured vision model prompt templates and manager.

Provides VisionPromptTemplate (frozen dataclass) and VisionPromptManager
for building, sending, and parsing structured vision analysis requests
through Ollama's chat API.

6 built-in templates:
  - scene_description: General scene understanding
  - person_count: Count people and describe activities
  - threat_assessment: Evaluate hostile indicators
  - equipment_id: Identify vehicles, packages, tools
  - weather_conditions: Assess environmental conditions
  - change_detection: Compare frames and describe changes

Model-specific tuning is supported via model_overrides on each template.
Retry logic escalates from correction prompt to schema-only prompt before
giving up.
"""

from __future__ import annotations

import json
import logging
import re
from dataclasses import dataclass, field
from typing import Any

from engine.perception.vision import ollama_chat

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# VisionPromptTemplate
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class VisionPromptTemplate:
    """Frozen descriptor for a vision analysis prompt.

    Attributes:
        template_id: Unique identifier (e.g. "scene_description").
        system_prompt: System-level instructions for the vision model.
        user_prompt: User message template. May contain ``{placeholders}``
            that are filled by ``VisionPromptManager.build_messages()``.
        response_schema: JSON-schema-like dict describing expected response.
            Must include a ``"required"`` key listing mandatory field names.
        temperature: Sampling temperature (default 0.3).
        max_tokens: Maximum response tokens (default 256).
        model_overrides: Per-model tuning.  Keys are model names (e.g.
            ``"llava:7b"``).  Values are dicts that may contain:
            - ``"temperature"`` — override the default temperature.
            - ``"system_suffix"`` — text appended to the system prompt.
            - ``"max_tokens"`` — override the default max_tokens.
    """

    template_id: str
    system_prompt: str
    user_prompt: str
    response_schema: dict[str, Any]
    temperature: float = 0.3
    max_tokens: int = 256
    model_overrides: dict[str, dict[str, Any]] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Built-in template definitions
# ---------------------------------------------------------------------------

_SCENE_DESCRIPTION = VisionPromptTemplate(
    template_id="scene_description",
    system_prompt=(
        "You are a security camera analyst for a neighborhood watch system. "
        "Describe exactly what you see in the image. Focus on people, vehicles, "
        "and activities. Respond with valid JSON only, no markdown, no extra text."
    ),
    user_prompt=(
        "Describe the scene in this security camera frame. "
        "Include all visible people, vehicles, objects, and activities."
    ),
    response_schema={
        "required": ["description", "confidence"],
        "properties": {
            "description": {"type": "string"},
            "confidence": {"type": "number"},
            "objects": {"type": "array"},
            "activities": {"type": "array"},
        },
    },
    temperature=0.3,
    max_tokens=512,
    model_overrides={
        "llava:7b": {
            "temperature": 0.2,
            "system_suffix": " Be concise. JSON only.",
        },
        "minicpm-v:8b": {
            "temperature": 0.3,
        },
    },
)

_PERSON_COUNT = VisionPromptTemplate(
    template_id="person_count",
    system_prompt=(
        "You are a vision system that counts people in security camera frames. "
        "Count every visible person and briefly describe what each is doing. "
        "Respond with valid JSON only, no markdown, no extra text."
    ),
    user_prompt=(
        "Count the number of people visible in this image. "
        "For each person, describe their approximate location and activity."
    ),
    response_schema={
        "required": ["count", "people"],
        "properties": {
            "count": {"type": "integer"},
            "people": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "location": {"type": "string"},
                        "activity": {"type": "string"},
                    },
                },
            },
        },
    },
    temperature=0.3,
    max_tokens=512,
    model_overrides={
        "llava:7b": {
            "temperature": 0.2,
            "system_suffix": " Count carefully. JSON only.",
        },
        "minicpm-v:8b": {
            "temperature": 0.3,
        },
    },
)

_THREAT_ASSESSMENT = VisionPromptTemplate(
    template_id="threat_assessment",
    system_prompt=(
        "You are a security threat analysis system. Evaluate the image for "
        "potential hostile indicators: weapons, aggressive posture, trespassing, "
        "concealment, or suspicious behavior. Rate the threat level. "
        "Respond with valid JSON only, no markdown, no extra text."
    ),
    user_prompt=(
        "Assess the threat level in this security camera frame. "
        "Identify any hostile indicators, weapons, aggressive posture, "
        "or suspicious behavior."
    ),
    response_schema={
        "required": ["threat_level", "indicators"],
        "properties": {
            "threat_level": {
                "type": "string",
                "enum": ["none", "low", "medium", "high", "critical"],
            },
            "indicators": {"type": "array"},
            "description": {"type": "string"},
            "confidence": {"type": "number"},
        },
    },
    temperature=0.2,
    max_tokens=512,
    model_overrides={
        "llava:7b": {
            "temperature": 0.2,
            "system_suffix": " Be precise and cautious. JSON only.",
        },
        "minicpm-v:8b": {
            "temperature": 0.3,
        },
    },
)

_EQUIPMENT_ID = VisionPromptTemplate(
    template_id="equipment_id",
    system_prompt=(
        "You are an equipment identification system for a neighborhood security "
        "camera network. Identify all vehicles, packages, tools, and notable "
        "objects visible in the image. "
        "Respond with valid JSON only, no markdown, no extra text."
    ),
    user_prompt=(
        "Identify all equipment, vehicles, packages, and notable objects "
        "visible in this security camera frame."
    ),
    response_schema={
        "required": ["items"],
        "properties": {
            "items": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "type": {"type": "string"},
                        "description": {"type": "string"},
                        "location": {"type": "string"},
                    },
                },
            },
            "confidence": {"type": "number"},
        },
    },
    temperature=0.3,
    max_tokens=512,
    model_overrides={
        "llava:7b": {
            "temperature": 0.2,
            "system_suffix": " List items precisely. JSON only.",
        },
        "minicpm-v:8b": {
            "temperature": 0.4,
        },
    },
)

_WEATHER_CONDITIONS = VisionPromptTemplate(
    template_id="weather_conditions",
    system_prompt=(
        "You are a weather assessment system analyzing outdoor security camera "
        "images. Determine current weather conditions, lighting, visibility, "
        "and time of day. "
        "Respond with valid JSON only, no markdown, no extra text."
    ),
    user_prompt=(
        "Assess the current weather conditions, lighting, and visibility "
        "from this outdoor camera frame."
    ),
    response_schema={
        "required": ["conditions", "visibility"],
        "properties": {
            "conditions": {"type": "string"},
            "visibility": {
                "type": "string",
                "enum": ["clear", "reduced", "poor", "minimal"],
            },
            "lighting": {"type": "string"},
            "time_of_day": {"type": "string"},
            "precipitation": {"type": "string"},
        },
    },
    temperature=0.3,
    max_tokens=256,
    model_overrides={
        "llava:7b": {
            "temperature": 0.3,
            "system_suffix": " Focus on observable conditions. JSON only.",
        },
        "minicpm-v:8b": {
            "temperature": 0.3,
        },
    },
)

_CHANGE_DETECTION = VisionPromptTemplate(
    template_id="change_detection",
    system_prompt=(
        "You are a change detection system comparing security camera frames "
        "over time. Identify what has changed between the reference frame "
        "and the current frame. Focus on new objects, missing objects, "
        "moved objects, and activity changes. "
        "Respond with valid JSON only, no markdown, no extra text."
    ),
    user_prompt=(
        "Compare this frame to the reference. Describe all changes: "
        "new objects, missing objects, moved items, and activity differences. "
        "{context}"
    ),
    response_schema={
        "required": ["changes_detected", "changes"],
        "properties": {
            "changes_detected": {"type": "boolean"},
            "changes": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "type": {"type": "string"},
                        "description": {"type": "string"},
                    },
                },
            },
            "summary": {"type": "string"},
        },
    },
    temperature=0.3,
    max_tokens=512,
    model_overrides={
        "llava:7b": {
            "temperature": 0.2,
            "system_suffix": " Focus on concrete changes. JSON only.",
        },
        "minicpm-v:8b": {
            "temperature": 0.3,
        },
    },
)

_DEFAULT_TEMPLATES: list[VisionPromptTemplate] = [
    _SCENE_DESCRIPTION,
    _PERSON_COUNT,
    _THREAT_ASSESSMENT,
    _EQUIPMENT_ID,
    _WEATHER_CONDITIONS,
    _CHANGE_DETECTION,
]


# ---------------------------------------------------------------------------
# JSON parsing helpers
# ---------------------------------------------------------------------------

# Regex to strip markdown code fences: ```json ... ``` or ``` ... ```
_CODE_FENCE_RE = re.compile(
    r"```(?:json)?\s*\n?(.*?)\n?\s*```", re.DOTALL
)

# Regex to find the first JSON object in text
_JSON_OBJECT_RE = re.compile(r"\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}", re.DOTALL)


def _extract_json(text: str) -> dict[str, Any] | None:
    """Try to extract a JSON object from *text*.

    Strategy:
    1. Strip markdown code fences (```json ... ```)
    2. Try ``json.loads()`` on the whole string
    3. Regex for the first ``{...}`` block and try ``json.loads()`` on that
    4. Return ``None`` if nothing works
    """
    # Step 1: strip code fences
    fence_match = _CODE_FENCE_RE.search(text)
    if fence_match:
        text = fence_match.group(1).strip()

    # Step 2: try direct parse
    try:
        obj = json.loads(text)
        if isinstance(obj, dict):
            return obj
    except (json.JSONDecodeError, ValueError):
        pass

    # Step 3: regex for embedded JSON
    obj_match = _JSON_OBJECT_RE.search(text)
    if obj_match:
        try:
            obj = json.loads(obj_match.group(0))
            if isinstance(obj, dict):
                return obj
        except (json.JSONDecodeError, ValueError):
            pass

    return None


def _validate_schema(data: dict[str, Any], schema: dict[str, Any]) -> bool:
    """Check that *data* has all keys listed in ``schema["required"]``."""
    required = schema.get("required", [])
    return all(key in data for key in required)


# ---------------------------------------------------------------------------
# VisionPromptManager
# ---------------------------------------------------------------------------


class VisionPromptManager:
    """Registry and executor for structured vision model prompts.

    On construction the 6 built-in templates are loaded.  Additional
    templates can be registered at runtime via ``register_template()``.

    The ``analyze()`` method is the main entry point: it builds messages,
    calls Ollama via ``ollama_chat()``, parses the response, and retries
    on failure with escalating correction prompts.
    """

    def __init__(self) -> None:
        self._templates: dict[str, VisionPromptTemplate] = {}
        for tmpl in _DEFAULT_TEMPLATES:
            self._templates[tmpl.template_id] = tmpl

    # -- template management ------------------------------------------------

    def get_template(self, template_id: str) -> VisionPromptTemplate | None:
        """Return the template with *template_id*, or ``None``."""
        return self._templates.get(template_id)

    def register_template(self, template: VisionPromptTemplate) -> None:
        """Register (or overwrite) a template."""
        self._templates[template.template_id] = template

    # -- message building ---------------------------------------------------

    def build_messages(
        self,
        template_id: str,
        model: str,
        images: list[str],
        context: dict[str, str] | None = None,
    ) -> list[dict[str, Any]]:
        """Build an Ollama chat message list for *template_id*.

        Args:
            template_id: Which template to use.
            model: Ollama model name (e.g. ``"llava:7b"``).
            images: List of base64-encoded image strings.
            context: Optional dict of ``{placeholder: value}`` pairs to
                fill into the ``user_prompt`` template.

        Returns:
            A list of message dicts suitable for ``ollama_chat()``.

        Raises:
            ValueError: If *template_id* is not registered.
        """
        tmpl = self._templates.get(template_id)
        if tmpl is None:
            raise ValueError(f"Unknown template: {template_id}")

        # Build system prompt, applying model-specific suffix if present
        system_prompt = tmpl.system_prompt
        overrides = tmpl.model_overrides.get(model, {})
        suffix = overrides.get("system_suffix", "")
        if suffix:
            system_prompt = system_prompt + suffix

        # Build user prompt, filling placeholders from context
        user_prompt = tmpl.user_prompt
        if context:
            for key, value in context.items():
                user_prompt = user_prompt.replace(f"{{{key}}}", str(value))

        messages: list[dict[str, Any]] = [
            {"role": "system", "content": system_prompt},
        ]

        user_msg: dict[str, Any] = {
            "role": "user",
            "content": user_prompt,
        }
        if images:
            user_msg["images"] = list(images)

        messages.append(user_msg)
        return messages

    # -- response parsing ---------------------------------------------------

    def parse_response(
        self,
        template_id: str,
        raw_response: str,
    ) -> dict[str, Any] | None:
        """Parse raw LLM text into a validated dict.

        Steps:
        1. Strip markdown code blocks
        2. Try ``json.loads()`` directly
        3. Regex for ``{...}`` within the response
        4. Validate required keys from ``response_schema``
        5. Return ``None`` if anything fails

        Args:
            template_id: Template whose schema to validate against.
            raw_response: Raw text from the vision model.

        Returns:
            Parsed dict if valid, ``None`` otherwise.
        """
        tmpl = self._templates.get(template_id)
        if tmpl is None:
            return None

        data = _extract_json(raw_response)
        if data is None:
            return None

        if not _validate_schema(data, tmpl.response_schema):
            return None

        return data

    # -- full analysis pipeline ---------------------------------------------

    def analyze(
        self,
        template_id: str,
        model: str,
        images: list[str],
        context: dict[str, str] | None = None,
        fleet: Any | None = None,
        max_retries: int = 2,
    ) -> dict[str, Any] | None:
        """Run a full vision analysis pipeline.

        1. Build messages for the template
        2. Call Ollama via ``ollama_chat()``
        3. Parse and validate the response
        4. On failure, retry with escalating correction prompts

        Retry strategy (up to *max_retries*):
            - Retry 1: Append "Your previous response was not valid JSON.
              Respond ONLY with valid JSON." to messages.
            - Retry 2+: Replace user prompt with schema-only prompt that
              shows the expected JSON structure.

        Args:
            template_id: Which analysis template to use.
            model: Ollama model name.
            images: List of base64-encoded images.
            context: Optional placeholder context for the user prompt.
            fleet: Optional OllamaFleet for host discovery.  If ``None``
                the method returns ``None`` immediately.
            max_retries: Maximum number of retry attempts (default 2).

        Returns:
            Parsed response dict, or ``None`` if all attempts fail.
        """
        if fleet is None:
            logger.warning("analyze() called with no fleet — returning None")
            return None

        tmpl = self._templates.get(template_id)
        if tmpl is None:
            logger.error("Unknown template: %s", template_id)
            return None

        # Resolve host
        try:
            host = fleet.best_host(model)
            base_url = host.base_url if host else None
        except Exception:
            base_url = None

        if base_url is None:
            logger.warning("No host available for model %s", model)
            return None

        # Resolve temperature from model overrides
        overrides = tmpl.model_overrides.get(model, {})
        temperature = overrides.get("temperature", tmpl.temperature)

        # Build initial messages
        messages = self.build_messages(template_id, model, images, context)

        # Attempt loop: 1 initial + max_retries
        for attempt in range(1 + max_retries):
            try:
                response = ollama_chat(
                    model=model,
                    messages=messages,
                    base_url=base_url,
                )
            except Exception as exc:
                logger.error(
                    "ollama_chat failed (attempt %d): %s", attempt + 1, exc
                )
                continue

            raw_text = response.get("message", {}).get("content", "")
            result = self.parse_response(template_id, raw_text)
            if result is not None:
                return result

            # Escalate prompt for next attempt
            if attempt == 0:
                # Retry 1: correction prompt
                messages.append(
                    {
                        "role": "assistant",
                        "content": raw_text,
                    }
                )
                messages.append(
                    {
                        "role": "user",
                        "content": (
                            "Your previous response was not valid JSON. "
                            "Respond ONLY with valid JSON matching this schema: "
                            + json.dumps(tmpl.response_schema)
                        ),
                    }
                )
                if images:
                    messages[-1]["images"] = list(images)
            elif attempt < max_retries:
                # Retry 2+: schema-only prompt — replace all messages
                schema_prompt = (
                    "Respond with ONLY a valid JSON object. "
                    "Required keys: "
                    + json.dumps(tmpl.response_schema.get("required", []))
                    + ". Schema: "
                    + json.dumps(tmpl.response_schema)
                    + ". No markdown, no explanation, just the JSON object."
                )
                messages = [
                    {"role": "system", "content": tmpl.system_prompt},
                ]
                user_msg: dict[str, Any] = {
                    "role": "user",
                    "content": schema_prompt,
                }
                if images:
                    user_msg["images"] = list(images)
                messages.append(user_msg)

            logger.debug(
                "Vision analysis attempt %d/%d failed for %s",
                attempt + 1,
                1 + max_retries,
                template_id,
            )

        logger.warning(
            "All %d attempts exhausted for template %s",
            1 + max_retries,
            template_id,
        )
        return None
