from __future__ import annotations

import json
import re
from typing import Any, Dict, List

from package import parser, Namespace, Problem, ROOT


if __name__ == "__main__":
    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    data: Dict[str, Any] = {
        "problem": namespace.problem,
        "iterations": namespace.iterations,
        "tabu_size": namespace.tabu_size,
        "config": namespace.config,
        "speed_type": namespace.speed_type,
        "range_type": namespace.range_type,
        "cost": float(input()),
    }

    problem = Problem.import_data(namespace.problem)
    truck_paths: List[List[List[int]]] = [[] for _ in range(problem.trucks_count)]
    drone_paths: List[List[List[int]]] = [[] for _ in range(problem.drones_count)]

    for vehicles_count, vehicle_paths in zip([problem.trucks_count, problem.drones_count], [truck_paths, drone_paths]):
        for index in range(vehicles_count):
            new = True
            for customer in map(int, input().split()):
                if customer == 0:
                    if new:
                        vehicle_paths[index].append([0])
                    else:
                        vehicle_paths[index][-1].append(0)

                    new = not new

                else:
                    vehicle_paths[index][-1].append(customer)

    data["truck_paths"] = truck_paths
    data["drone_paths"] = drone_paths
    data["feasible"] = bool(int(input()))

    for _ in range(3):
        line = input()
        re_match = re.fullmatch(r"^(real|user|sys)\s+(\d+(?:\.\d*)?)$", line)
        if re_match is None:
            message = f"Unrecognized pattern: \"{line}\""
            raise RuntimeError(message)

        groups = re_match.groups()
        data[groups[0]] = float(groups[1])

    index = 0
    while ROOT.joinpath("result", f"{namespace.problem}-{index}.json").exists():
        index += 1

    with ROOT.joinpath("result", f"{namespace.problem}-{index}.json").open("w") as file:
        json.dump(data, file)
