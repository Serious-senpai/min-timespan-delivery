from __future__ import annotations

import itertools
import json
import random
import re
import string
from typing import List

from package import parser, Namespace, Problem, SolutionJSON, ROOT


def random_str(length: int) -> str:
    return "".join(random.choices(string.ascii_uppercase + string.digits, k=length))


if __name__ == "__main__":
    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    problem = Problem.import_data(namespace.problem)
    truck_paths: List[List[List[int]]] = [[] for _ in range(problem.trucks_count)]
    drone_paths: List[List[List[int]]] = [[] for _ in range(problem.drones_count)]

    cost = float(input())
    capacity_violation = float(input())
    drone_energy_violation = float(input())
    waiting_time_violation = float(input())
    fixed_time_violation = float(input())
    fixed_distance_violation = float(input())

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
    last_improved = int(input())
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
        "capacity_violation": capacity_violation,
        "drone_energy_violation": drone_energy_violation,
        "waiting_time_violation": waiting_time_violation,
        "fixed_time_violation": fixed_time_violation,
        "fixed_distance_violation": fixed_distance_violation,
        "truck_paths": truck_paths,
        "drone_paths": drone_paths,
        "feasible": feasible,
        "last_improved": last_improved,
        "real": real,
        "user": user,
        "sys": sys,
    }
    print(data)

    index = random_str(8)
    while ROOT.joinpath("result", f"{namespace.problem}-{index}.json").exists():
        index = random_str(8)

    with ROOT.joinpath("result", f"{namespace.problem}-{index}.json").open("w") as file:
        json.dump(data, file)
