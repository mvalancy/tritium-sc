# Engine: Perception

Vision analysis and fact extraction pipeline for camera feeds.

## Files

| File | Purpose |
|------|---------|
| `perception.py` | Layered perception: quality gate, complexity scoring, motion detection |
| `vision.py` | Ollama chat API wrapper for image analysis |
| `vision_prompts.py` | Prompt templates for different vision tasks |
| `extraction.py` | Regex-based fact extraction from conversation text |

## Pipeline

Frames flow through quality gate (blur/brightness check) -> motion
detection -> complexity scoring -> LLM vision analysis. Only frames
that pass all gates are sent to the LLM for description.
