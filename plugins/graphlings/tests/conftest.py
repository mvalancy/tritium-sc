"""Conftest for graphlings plugin tests — add plugins/ to sys.path."""
import sys
from pathlib import Path

# Add plugins/ directory to path so `from graphlings.xxx import ...` works
_plugins_dir = str(Path(__file__).resolve().parent.parent.parent)
if _plugins_dir not in sys.path:
    sys.path.insert(0, _plugins_dir)
