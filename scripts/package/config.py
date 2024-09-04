from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing_extensions import Any, Dict, Final, List, Literal, Tuple

from .utils import euc_distance


__all__ = ("ROOT", "TruckConfig", "DroneLinearConfig", "DroneNonlinearConfig", "DroneEnduranceConfig", "Problem")
ROOT: Final[Path] = Path(__file__).parent.parent.parent


@dataclass(frozen=True, kw_only=True, slots=True)
class TruckConfig:
    maximum_velocity: float
    capacity: float
    coefficients: Tuple[float, ...]

    @staticmethod
    def import_data() -> TruckConfig:
        with open(ROOT / "problems" / "config_parameter" / "truck_config.json", "r") as file:
            data = json.load(file)

        coefficients_d = data["T (hour)"]
        assert isinstance(coefficients_d, dict)

        return TruckConfig(
            maximum_velocity=data["V_max (m/s)"],
            capacity=data["M_t (kg)"],
            coefficients=tuple(coefficients_d.values()),
        )


@dataclass(frozen=True, kw_only=True, slots=True)
class _BaseDroneConfig:
    capacity: float
    speed_type: Literal["low", "high"]
    range_type: Literal["low", "high"]


@dataclass(frozen=True, kw_only=True, slots=True)
class _VariableDroneConfig(_BaseDroneConfig):
    takeoff_speed: float
    cruise_speed: float
    landing_speed: float
    altitude: float
    battery: float

    @staticmethod
    def from_data(data: Dict[str, Any]) -> _VariableDroneConfig:
        return _VariableDroneConfig(
            takeoff_speed=data["takeoffSpeed [m/s]"],
            cruise_speed=data["cruiseSpeed [m/s]"],
            landing_speed=data["landingSpeed [m/s]"],
            altitude=data["cruiseAlt [m]"],
            battery=data["batteryPower [Joule]"],
            capacity=data["capacity [kg]"],
            speed_type=data["speed_type"],
            range_type=data["range"],
        )


@dataclass(frozen=True, kw_only=True, slots=True)
class DroneLinearConfig(_VariableDroneConfig):
    beta: float
    gamma: float

    @staticmethod
    def import_data() -> Tuple[DroneLinearConfig, ...]:
        with open(ROOT / "problems" / "config_parameter" / "drone_linear_config.json", "r") as file:
            data = json.load(file)
            assert isinstance(data, dict)

        results: List[DroneLinearConfig] = []
        for d in data.values():
            base = _VariableDroneConfig.from_data(d)
            item = DroneLinearConfig(
                beta=d["beta(w/kg)"],
                gamma=d["gamma(w)"],
                **asdict(base),
            )

            results.append(item)

        return tuple(results)


@dataclass(frozen=True, kw_only=True, slots=True)
class DroneNonlinearConfig(_VariableDroneConfig):
    k1: float
    k2: float
    c1: float
    c2: float
    c4: float
    c5: float

    @staticmethod
    def import_data() -> Tuple[DroneNonlinearConfig, ...]:
        with open(ROOT / "problems" / "config_parameter" / "drone_nonlinear_config.json", "r") as file:
            data = json.load(file)
            assert isinstance(data, dict)

        results: List[DroneNonlinearConfig] = []
        for d in data.values():
            if isinstance(d, dict):
                base = _VariableDroneConfig.from_data(d)
                item = DroneNonlinearConfig(
                    k1=data["k1"],
                    k2=data["k2 (sqrt(kg/m)"],
                    c1=data["c1 (sqrt(m/kg)"],
                    c2=data["c2 (sqrt(m/kg)"],
                    c4=data["c4 (kg/m)"],
                    c5=data["c5 (Ns/m)"],
                    **asdict(base),
                )

                results.append(item)

        return tuple(results)


@dataclass(frozen=True, kw_only=True, slots=True)
class DroneEnduranceConfig(_BaseDroneConfig):
    fixed_time: float
    # fixed_distance: float
    drone_speed: float

    @staticmethod
    def import_data() -> Tuple[DroneEnduranceConfig, ...]:
        with open(ROOT / "problems" / "config_parameter" / "drone_endurance_config.json", "r") as file:
            data = json.load(file)
            assert isinstance(data, dict)

        results: List[DroneEnduranceConfig] = []
        for d in data.values():
            item = DroneEnduranceConfig(
                fixed_time=d["FixedTime (s)"],
                # fixed_distance=d["FixedDistance (m)"],
                drone_speed=d["Drone_speed (m/s)"],
                capacity=d["capacity [kg]"],
                speed_type=d["speed_type"],
                range_type=d["range"],
            )

            results.append(item)

        return tuple(results)


@dataclass(frozen=True, kw_only=True, slots=True)
class Problem:
    problem: str
    customers_count: int
    trucks_count: int
    drones_count: int

    x: Tuple[float, ...]
    y: Tuple[float, ...]
    demands: Tuple[float, ...]
    dronable: Tuple[bool, ...]

    truck_service_time: Tuple[float, ...]
    drone_service_time: Tuple[float, ...]

    truck_capacity: float
    drone_capacity: float

    truck_speed: float
    drone_speed: float

    drone_endurance: float

    truck_time_limit: float
    drone_time_limit: float

    truck_unit_cost: float
    drone_unit_cost: float

    @staticmethod
    def import_data(problem: str, /) -> Problem:
        problem = problem.removesuffix(".txt")
        with ROOT.joinpath("problems", "[11]", "instances", "min-cost", f"{problem}.txt").open("r") as file:
            data = file.read()

        customers_count = int(problem.split("-")[0])

        trucks_count = int(re.search(r"NUM TRUCKS,(\d+)", data).group(1))  # type: ignore
        drones_count = int(re.search(r"NUM DRONES,(\d+)", data).group(1))  # type: ignore

        truck_capacity = float(re.search(r"TRUCK CAP,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore
        drone_capacity = float(re.search(r"DRONE CAP,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore

        truck_speed = float(re.search(r"TRUCK SPEED,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore
        drone_speed = float(re.search(r"DRONE SPEED,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore

        drone_endurance = float(re.search(r"DRONE ENDURANCE,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore

        truck_time_limit = float(re.search(r"TRUCK TIME LIMIT,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore
        drone_time_limit = float(re.search(r"DRONE TIME LIMIT,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore

        truck_unit_cost = float(re.search(r"TRUCK UNIT COST,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore
        drone_unit_cost = float(re.search(r"DRONE UNIT COST,(\d+(?:\.\d+)?)", data).group(1))  # type: ignore

        x: List[float] = []
        y: List[float] = []
        demands: List[float] = []
        dronable: List[bool] = []
        truck_service_time = [0.0] * (customers_count + 1)
        drone_service_time = [0.0] * (customers_count + 1)
        for match in re.finditer(r"^\d+\s+(\d+(?:\.\d+)?)\s+(\d+(?:\.\d+)?)\s+(\d+(?:\.\d+)?)$", data, re.MULTILINE):
            _x, _y, demand = match.groups()
            x.append(float(_x))
            y.append(float(_y))
            demands.append(float(demand))
            dronable.append(demands[-1] <= drone_capacity and 2 * euc_distance(x[-1] - x[0], y[-1] - y[0]) <= drone_speed * drone_endurance)

        return Problem(
            problem=problem,
            customers_count=customers_count,
            trucks_count=trucks_count,
            drones_count=drones_count,
            x=tuple(x),
            y=tuple(y),
            demands=tuple(demands),
            dronable=tuple(dronable),
            truck_service_time=tuple(truck_service_time),
            drone_service_time=tuple(drone_service_time),
            truck_capacity=truck_capacity,
            drone_capacity=drone_capacity,
            truck_speed=truck_speed,
            drone_speed=drone_speed,
            drone_endurance=drone_endurance,
            truck_time_limit=truck_time_limit,
            drone_time_limit=drone_time_limit,
            truck_unit_cost=truck_unit_cost,
            drone_unit_cost=drone_unit_cost,
        )
