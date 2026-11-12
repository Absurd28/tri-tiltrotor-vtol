"""Parameter loader for the VTOL drone project.

Usage:
    from params import load_params, get_param
    p = load_params()          # returns full dict
    mass = get_param("mass.total")  # dot-path access
"""
from __future__ import annotations
import os
from pathlib import Path
from typing import Any

import yaml

_PARAMS_FILE = Path(__file__).parent / "params.yaml"
_cache: dict | None = None


def load_params(path: str | Path | None = None, force_reload: bool = False) -> dict:
    global _cache
    if _cache is not None and not force_reload and path is None:
        return _cache
    p = Path(path) if path else _PARAMS_FILE
    with open(p, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if path is None:
        _cache = data
    return data


def get_param(key: str, default: Any = None, params: dict | None = None) -> Any:
    """Retrieve a value by dot-separated key, e.g. 'mass.total'."""
    d = params if params is not None else load_params()
    for part in key.split("."):
        if not isinstance(d, dict) or part not in d:
            return default
        d = d[part]
    return d