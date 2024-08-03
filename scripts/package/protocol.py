from __future__ import annotations

from typing import List, TypedDict


__all__ = ("SolutionJSON",)


class SolutionJSON(TypedDict):
    problem: str
    iterations: int
    tabu_size_coefficient: int
    config: str
    speed_type: str
    range_type: str
    cost: float
    capacity_violation: float
    drone_energy_violation: float
    waiting_time_violation: float
    fixed_time_violation: float
    fixed_distance_violation: float
    truck_paths: List[List[List[int]]]
    drone_paths: List[List[List[int]]]
    feasible: bool
    last_improved: int
    real: float
    user: float
    sys: float
