from __future__ import annotations

import itertools
import json
import re
from typing import List

from package import parser, Namespace, Problem, SolutionJSON, ROOT


if __name__ == "__main__":
    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    problem = Problem.import_data(namespace.problem)
    truck_paths: List[List[List[int]]] = [[] for _ in range(problem.trucks_count)]
    drone_paths: List[List[List[int]]] = [[] for _ in range(problem.drones_count)]

    cost = float(input())

    for paths in itertools.chain(truck_paths, drone_paths):
        new = True
        for customer in map(int, input().split()):
            if customer == 0:
                if new:
                    paths.append([0])
                else:
                    paths[-1].append(0)

                new = not new

            else:
                paths[-1].append(customer)

    feasible = bool(int(input()))
    real = user = sys = -1.0

    for _ in range(3):
        line = input()
        re_match = re.fullmatch(r"^(real|user|sys)\s+(\d+(?:\.\d*)?)$", line)
        if re_match is None:
            message = f"Unrecognized pattern: \"{line}\""
            raise RuntimeError(message)

        groups = re_match.groups()
        value = float(groups[1])
        match groups[0]:
            case "real":
                real = value
            case "user":
                user = value
            case "sys":
                sys = value

    data: SolutionJSON = {
        "problem": namespace.problem,
        "iterations": namespace.iterations,
        "tabu_size": namespace.tabu_size,
        "config": namespace.config,
        "speed_type": namespace.speed_type,
        "range_type": namespace.range_type,
        "cost": cost,
        "truck_paths": truck_paths,
        "drone_paths": drone_paths,
        "feasible": feasible,
        "real": real,
        "user": user,
        "sys": sys,
    }

    index = 0
    while ROOT.joinpath("result", f"{namespace.problem}-{index}.json").exists():
        index += 1

    with ROOT.joinpath("result", f"{namespace.problem}-{index}.json").open("w") as file:
        json.dump(data, file)
