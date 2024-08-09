from __future__ import annotations


__all__ = ("man_distance", "euc_distance")


def man_distance(dx: float, dy: float) -> float:
    return abs(dx) + abs(dy)


def euc_distance(dx: float, dy: float) -> float:
    return (dx ** 2 + dy ** 2) ** 0.5
