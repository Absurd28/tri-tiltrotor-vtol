"""Logging helpers for the VTOL drone project."""
from __future__ import annotations
import logging
import sys
from pathlib import Path


def get_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter(
        "%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
        datefmt="%H:%M:%S",
    ))
    logger.addHandler(handler)
    logger.setLevel(level)
    return logger


def setup_file_logger(name: str, log_dir: str | Path, level: int = logging.DEBUG) -> logging.Logger:
    logger = get_logger(name, level)
    path = Path(log_dir) / f"{name}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    fh = logging.FileHandler(path, encoding="utf-8")
    fh.setFormatter(logging.Formatter(
        "%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
    ))
    logger.addHandler(fh)
    return logger