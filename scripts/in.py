from __future__ import annotations

import sys
from typing import Tuple, Union

from package import parser, DroneEnduranceConfig, DroneLinearConfig, DroneNonlinearConfig, Namespace, Problem, TruckConfig


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

    print(namespace.iterations)
    print(namespace.tabu_size_coefficient)
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
