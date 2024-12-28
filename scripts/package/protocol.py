from __future__ import annotations

from typing_extensions import Dict, Generic, List, Literal, Optional, Tuple, TypedDict, TypeVar, Union, overload


__all__ = (
    "SolutionJSON",
    "PrettySolutionJSON",
    "PropagationJSON",
    "NeighborhoodJSON",
    "ResultJSON",
    "MILPResultJSON",
    "prettify"
)


class _BaseSolutionJSON(TypedDict):
    cost: float
    working_time: float
    drone_energy_violation: float
    capacity_violation: float
    waiting_time_violation: float
    fixed_time_violation: float
    feasible: bool
    truck_working_time: List[float]
    drone_working_time: List[float]


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
    tabu_size_factor: int
    reset_after_factor: int
    tabu_size: int
    reset_after: int
    max_elite_size: int
    destroy_rate: int
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
    elapsed: float
    url: Optional[str]


class _FeasibleMILPResultJSON(TypedDict):
    x: Dict[str, float]
    y: Dict[str, float]
    L_w: float
    L_d: float
    staff_velocity: float
    drone_velocity: float
    num_staff: int
    num_drone: int
    use_ejection: bool
    use_inter: bool
    use_intra: bool
    Optimal: float
    Solve_Time: float
    status: str


class _InfeasibleMILPResultJSON(TypedDict):
    status: Literal["INFEASIBLE"]


MILPResultJSON = Union[_FeasibleMILPResultJSON, _InfeasibleMILPResultJSON]


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
