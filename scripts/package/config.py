from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Final, List, Literal, Tuple


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
    fixed_distance: float
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
                fixed_distance=d["FixedDistance (m)"],
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

    @staticmethod
    def import_data(problem: str, /) -> Problem:
        problem = problem.removesuffix(".txt")
        with open(ROOT / "problems" / "data" / f"{problem}.txt", "r") as file:
            data = file.read()

        customers_count = int(re.search(r"Customers (\d+)", data).group(1))  # type: ignore
        trucks_count = int(re.search(r"number_staff (\d+)", data).group(1))  # type: ignore
        drones_count = int(re.search(r"number_drone (\d+)", data).group(1))  # type: ignore

        x = [0.0]
        y = [0.0]
        demands = [0.0]
        dronable = [True]
        truck_service_time = [0.0]
        drone_service_time = [0.0]
        for match in re.finditer(r"([-\d\.]+)\s+([-\d\.]+)\s+([\d\.]+)\s+(0|1)\s+([\d\.]+)\s+([\d\.]+)", data):
            _x, _y, demand, truck_only, _truck_service_time, _drone_service_time = match.groups()
            x.append(float(_x))
            y.append(float(_y))
            demands.append(float(demand))
            dronable.append(truck_only == "0")
            truck_service_time.append(float(_truck_service_time))
            drone_service_time.append(float(_drone_service_time))

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
        )
