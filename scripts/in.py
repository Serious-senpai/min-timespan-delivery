from __future__ import annotations

import argparse
import sys
from typing_extensions import Literal, Tuple, Union, TYPE_CHECKING

from package import (
    DroneEnduranceConfig,
    DroneLinearConfig,
    DroneNonlinearConfig,
    Problem,
    TruckConfig,
    euc_distance,
)


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        tabu_size_factor: int
        config: Literal["linear", "non-linear", "endurance", "unlimited"]
        speed_type: Literal["low", "high"]
        range_type: Literal["low", "high"]
        trucks_count: int
        drones_count: int
        waiting_time_limit: float
        strategy: Literal["random", "cyclic", "vns"]
        fix_iteration: int
        reset_after_factor: int
        max_elite_size: int
        destroy_rate: int
        verbose: bool


parser = argparse.ArgumentParser(
    description="The min-timespan parallel technician-and-drone scheduling in door-to-door sampling service system.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("problem", type=str, help="the problem name in the archive")
parser.add_argument("-t", "--tabu-size-factor", default=1, type=int, help="tabu size of each neighborhood = a1 * base")
parser.add_argument("-c", "--config", default="endurance", choices=["linear", "non-linear", "endurance", "unlimited"], help="the energy consumption model to use")
parser.add_argument("--speed-type", default="high", choices=["low", "high"], help="speed type of drones")
parser.add_argument("--range-type", default="high", choices=["low", "high"], help="range type of drones")
parser.add_argument("--trucks-count", default=-1, type=int, help="the number of trucks to override, pass a negative value to use default")
parser.add_argument("--drones-count", default=-1, type=int, help="the number of drones to override, pass a negative value to use default")
parser.add_argument("--waiting-time-limit", type=float, default=3600, help="the waiting time limit for each customer")
parser.add_argument("--strategy", default="random", choices=["random", "cyclic", "vns"], help="tabu search neighborhood selection strategy")
parser.add_argument(
    "--fix-iteration",
    default=-1,
    type=int,
    help="fix the number of iterations, set this value will also set --reset-after-factor (i.e. reset does not occur)",
)
parser.add_argument("--reset-after-factor", default=30, type=int, help="the number of non-improved iterations before resetting the current solution = a2 * base")
parser.add_argument("--max-elite-size", default=10, type=int, help="the maximum size of the elite set = a3")
parser.add_argument("--destroy-rate", default=0, type=int, help="the perentage of an elite solution to destroy = a4")
parser.add_argument("-v", "--verbose", action="store_true", help="the verbose mode")


if __name__ == "__main__":
    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    print(namespace, file=sys.stderr)

    problem = Problem.import_data(namespace.problem)

    truck = TruckConfig.import_data()

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
        namespace.trucks_count if namespace.trucks_count > 0 else problem.trucks_count,
        namespace.drones_count if namespace.drones_count > 0 else problem.drones_count,
        namespace.waiting_time_limit,
    )

    print(*problem.x)
    print(*problem.y)
    print(*problem.demands)
    print(*map(int, problem.dronable))

    print(*problem.truck_service_time)
    print(*problem.drone_service_time)

    print(namespace.tabu_size_factor)
    print(int(namespace.verbose))

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

    print(
        namespace.strategy,
        namespace.fix_iteration,
        namespace.max_elite_size,
        namespace.reset_after_factor if namespace.fix_iteration < 0 else namespace.fix_iteration,
        namespace.destroy_rate,
    )
    print(0)  # Not in evaluation
