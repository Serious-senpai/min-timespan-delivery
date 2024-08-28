from __future__ import annotations

from typing_extensions import Any


__all__ = ("man_distance", "euc_distance", "csv_wrap")


def man_distance(dx: float, dy: float) -> float:
    return abs(dx) + abs(dy)


def euc_distance(dx: float, dy: float) -> float:
    return (dx ** 2 + dy ** 2) ** 0.5


def csv_wrap(value: Any) -> str:
    return f"\"{value}\""
