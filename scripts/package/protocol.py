from __future__ import annotations

from typing_extensions import Generic, List, Optional, Tuple, TypedDict, TypeVar, overload


__all__ = (
    "SolutionJSON",
    "PrettySolutionJSON",
    "PropagationJSON",
    "NeighborhoodJSON",
    "ResultJSON",
    "prettify"
)


class _BaseSolutionJSON(TypedDict):
    cost: float
    travel_cost: float
    drone_energy_violation: float
    capacity_violation: float
    working_time_violation: float
    fixed_time_violation: float
    feasible: bool


T = TypeVar("T", bound=_BaseSolutionJSON)


class SolutionJSON(_BaseSolutionJSON):
    truck_paths: List[List[List[int]]]
    drone_paths: List[List[List[int]]]


class PrettySolutionJSON(_BaseSolutionJSON):
    truck_paths: str
    drone_paths: str


class PropagationJSON(Generic[T], TypedDict):
    solution: Optional[T]
    label: str


class NeighborhoodJSON(TypedDict):
    label: str
    pair: Tuple[int, int]


class ResultJSON(Generic[T], TypedDict):
    problem: str
    trucks_count: int
    drones_count: int
    iterations: int
    tabu_size: int
    config: str
    speed_type: str
    range_type: str
    solution: T
    propagation: List[PropagationJSON[T]]
    history: List[Optional[T]]
    progress: List[Optional[T]]
    coefficients: List[List[float]]
    neighborhoods: List[NeighborhoodJSON]
    initialization_label: str
    last_improved: int
    elite_set: List[List[float]]
    real: float
    user: float
    sys: float


@overload
def prettify(solution: SolutionJSON) -> PrettySolutionJSON: ...
@overload
def prettify(solution: None) -> None: ...


def prettify(solution: Optional[SolutionJSON]) -> Optional[PrettySolutionJSON]:
    if solution is None:
        return None

    return {
        **solution,
        "truck_paths": str(solution["truck_paths"]),
        "drone_paths": str(solution["drone_paths"]),
    }
