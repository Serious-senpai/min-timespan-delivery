from __future__ import annotations

from typing import List, TypedDict


__all__ = ("SolutionJSON",)


class SolutionJSON(TypedDict):
    problem: str
    iterations: int
    tabu_size: int
    config: str
    speed_type: str
    range_type: str
    cost: float
    truck_paths: List[List[List[int]]]
    drone_paths: List[List[List[int]]]
    feasible: bool
    last_improved: int
    real: float
    user: float
    sys: float
