from __future__ import annotations

from typing import List, TypedDict


__all__ = ("SolutionJSON", "ResultJSON")


class SolutionJSON(TypedDict):
    cost: float
    capacity_violation: float
    drone_energy_violation: float
    waiting_time_violation: float
    fixed_time_violation: float
    fixed_distance_violation: float
    truck_paths: List[List[List[int]]]
    drone_paths: List[List[List[int]]]
    feasible: bool


class ResultJSON(TypedDict):
    problem: str
    trucks_count: int
    drones_count: int
    iterations: int
    tabu_size: int
    config: str
    speed_type: str
    range_type: str
    solution: SolutionJSON
    propagation: List[SolutionJSON]
    last_improved: int
    real: float
    user: float
    sys: float
