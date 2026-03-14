# Engine: Inference

AI model routing and fleet management for LLM and vision tasks.

## Files

| File | Purpose |
|------|---------|
| `model_router.py` | Task-aware model selection with fallback chain |
| `fleet.py` | OllamaFleet — multi-host Ollama discovery (local, SSH, Tailscale) |
| `robot_thinker.py` | LLM-powered autonomous robot thinking and action generation |

## Architecture

The model router selects the best available model for a given task
(thinking, vision, extraction) across the fleet of Ollama instances.
Falls back through local -> remote -> stub chain.
