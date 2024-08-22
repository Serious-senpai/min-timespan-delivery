from __future__ import annotations

import argparse
import sys
from typing_extensions import Literal, Tuple, Union, TYPE_CHECKING

from package import DroneEnduranceConfig, DroneLinearConfig, DroneNonlinearConfig, Problem, TruckConfig


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        tabu_size: int
        config: Literal["linear", "non-linear", "endurance"]
        speed_type: Literal["low", "high"]
        range_type: Literal["low", "high"]
        verbose: bool
        max_elite_set_size: int
        reset_after: int
        hamming_distance_factor: float


parser = argparse.ArgumentParser(
    description="The min-timespan parallel technician-and-drone scheduling in door-to-door sampling service system.\nAlgorithm input transformer.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("problem", type=str, help="the problem name in the archive")
parser.add_argument("-t", "--tabu-size", default=10, type=int, help="the tabu size for each neighborhood")
parser.add_argument("--max-elite-set-size", default=10, type=int, help="the maximum size of the elite set")
parser.add_argument("--reset-after", default=250, type=int, help="the number of non-improved iterations before resetting the current solution")
parser.add_argument("--hamming-distance-factor", default=0.1, type=float, help="the factor F to remove an elite solution when hamming distance <= Fn")
parser.add_argument("-c", "--config", default="linear", choices=["linear", "non-linear", "endurance"], help="the energy consumption model to use")
parser.add_argument("--speed-type", default="low", choices=["low", "high"], help="speed type of drones")
parser.add_argument("--range-type", default="low", choices=["low", "high"], help="range type of drones")
parser.add_argument("-v", "--verbose", action="store_true", help="the verbose mode")


if __name__ == "__main__":
    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    print(namespace, file=sys.stderr)

    problem = Problem.import_data(namespace.problem)
    print(problem.customers_count, problem.trucks_count, problem.drones_count)

    print(*problem.x)
    print(*problem.y)
    print(*problem.demands)
    print(*map(int, problem.dronable))

    print(*problem.truck_service_time)
    print(*problem.drone_service_time)

    print(namespace.tabu_size)
    print(int(namespace.verbose))

    truck = TruckConfig.import_data()
    print(truck.maximum_velocity, truck.capacity)
    print(len(truck.coefficients), *truck.coefficients)

    models: Tuple[Union[DroneLinearConfig, DroneNonlinearConfig, DroneEnduranceConfig], ...]
    if namespace.config == "linear":
        models = DroneLinearConfig.import_data()
    elif namespace.config == "non-linear":
        models = DroneNonlinearConfig.import_data()
    else:
        models = DroneEnduranceConfig.import_data()

    for model in models:
        if model.speed_type == namespace.speed_type and model.range_type == namespace.range_type:
            break
    else:
        raise RuntimeError("Cannot find a satisfying model from list", models)

    print(model.__class__.__name__)
    print(
        model.capacity,
        model.speed_type,
        model.range_type,
    )
    if isinstance(model, DroneLinearConfig):
        print(
            model.takeoff_speed,
            model.cruise_speed,
            model.landing_speed,
            model.altitude,
            model.battery,
            model.beta,
            model.gamma,
        )
    elif isinstance(model, DroneNonlinearConfig):
        print(
            model.takeoff_speed,
            model.cruise_speed,
            model.landing_speed,
            model.altitude,
            model.battery,
            model.k1,
            model.k2,
            model.c1,
            model.c2,
            model.c4,
            model.c5,
        )
    else:
        print(
            model.fixed_time,
            model.fixed_distance,
            model.drone_speed,
        )

    print(namespace.max_elite_set_size, namespace.reset_after, namespace.hamming_distance_factor)
