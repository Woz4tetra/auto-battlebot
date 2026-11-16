"""Logging configuration for the synthgen package."""

import logging

_ROOT_LOGGER_NAME = "synthgen"


class _TerseFormatter(logging.Formatter):
    """Prefix WARNING+ records with their level; keep INFO/DEBUG lines clean."""

    def format(self, record: logging.LogRecord) -> str:
        base = f"[{record.name}] {record.getMessage()}"
        if record.levelno >= logging.WARNING:
            return f"[{record.name}] {record.levelname}: {record.getMessage()}"
        return base


def configure(verbosity: int = 0) -> None:
    """Configure the synthgen root logger once (-1 quiet, 0 info, 1+ debug)."""
    root = logging.getLogger(_ROOT_LOGGER_NAME)
    if verbosity <= -1:
        level = logging.WARNING
    elif verbosity == 0:
        level = logging.INFO
    else:
        level = logging.DEBUG
    root.setLevel(level)
    if not root.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(_TerseFormatter())
        root.addHandler(handler)
    root.propagate = False


def get_logger(module_name: str) -> logging.Logger:
    """Return the logger for a synthgen module."""
    short = module_name.rsplit(".", 1)[-1]
    return logging.getLogger(f"{_ROOT_LOGGER_NAME}.{short}")


def fmt_ctx(scene_idx: int, frame_idx: int | None = None) -> str:
    """Format the standard scene/frame prefix used in per-frame log lines."""
    if frame_idx is None:
        return f"scene {scene_idx}"
    return f"scene {scene_idx} frame {frame_idx}"
