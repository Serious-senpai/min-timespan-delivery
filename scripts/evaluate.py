from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing_extensions import List, Literal, Tuple, Union, TYPE_CHECKING

from package import (
    DroneEnduranceConfig,
    DroneLinearConfig,
    DroneNonlinearConfig,
    Problem,
    ResultJSON,
    SolutionJSON,
    TruckConfig,
    euc_distance,
)


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        evaluate: Path
        config: Literal["linear", "non-linear", "endurance", "unlimited"]


parser = argparse.ArgumentParser(
    description="Evaluate a provided solution using another model",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("evaluate", type=Path, help="path to the solution JSON to evaluate")
parser.add_argument("-c", "--config", required=True, choices=["linear", "non-linear", "endurance", "unlimited"], help="the new energy consumption model to use for evaluation")


if __name__ == "__main__":
    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    print(namespace, file=sys.stderr)

    with namespace.evaluate.open("r", encoding="utf-8") as file:
        data: ResultJSON[SolutionJSON] = json.load(file)

    problem = Problem.import_data(data["problem"])

    truck = TruckConfig.import_data()

    models: Tuple[Union[DroneLinearConfig, DroneNonlinearConfig, DroneEnduranceConfig], ...]
    if namespace.config == "linear":
        models = DroneLinearConfig.import_data()
    elif namespace.config == "non-linear":
        models = DroneNonlinearConfig.import_data()
    else:
        models = DroneEnduranceConfig.import_data()

    for model in models:
        if model.speed_type == data["speed_type"] and model.range_type == data["range_type"]:
            break

    else:
        raise RuntimeError("Cannot find a satisfying model from list", models)

    for index in range(problem.customers_count):
        problem.dronable[index] = problem.dronable[index] and problem.demands[index] <= model.capacity

    if isinstance(model, DroneEnduranceConfig):
        for index in range(1, problem.customers_count + 1):
            problem.dronable[index] = (
                problem.dronable[index]
                and 2 * euc_distance(problem.x[index], problem.y[index]) <= model.fixed_time * model.drone_speed
                and problem.demands[index] <= model.capacity
            )

    print(
        problem.customers_count,
        data["trucks_count"],
        data["drones_count"],
    )

    print(*problem.x)
    print(*problem.y)
    print(*problem.demands)
    print(*map(int, problem.dronable))

    print(*problem.truck_service_time)
    print(*problem.drone_service_time)

    print(data["tabu_size_factor"])
    print(0)  # verbose = False

    print(truck.maximum_velocity, truck.capacity)
    print(len(truck.coefficients), *truck.coefficients)

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
            model.fixed_time if namespace.config == "endurance" else 10 ** 9,
            # model.fixed_distance,
            model.drone_speed,
        )

    print(data["max_elite_size"], data["reset_after_factor"], data["destroy_rate"])
    print(1)  # In evaluation mode

    def print_routes(routes: List[List[int]]) -> None:
        print(len(routes))
        for route in routes:
            print(len(route), *route)

    for routes in data["solution"]["truck_paths"]:
        print_routes(routes)

    for routes in data["solution"]["drone_paths"]:
        print_routes(routes)
