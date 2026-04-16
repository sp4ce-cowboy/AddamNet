"""
config.py — Loads config.yaml and provides typed access.
"""

import os
import yaml
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_CFG = None


def _load():
    global _CFG
    cfg_path = _PROJECT_ROOT / "config.yaml"
    with open(cfg_path, "r") as f:
        _CFG = yaml.safe_load(f)
    return _CFG


def get():
    """Return the full config dict (loaded once)."""
    if _CFG is None:
        _load()
    return _CFG


def project_root() -> Path:
    return _PROJECT_ROOT


def resolve(rel_path: str) -> str:
    """Resolve a config-relative path to an absolute path."""
    p = Path(rel_path)
    if p.is_absolute():
        return str(p)
    return str(_PROJECT_ROOT / p)
