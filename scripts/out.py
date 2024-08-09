from __future__ import annotations

import itertools
import json
import random
import re
import string
import textwrap
from typing import List

from package import parser, Namespace, Problem, SolutionJSON, ROOT


def random_str(length: int) -> str:
    return "".join(random.choices(string.ascii_letters + string.digits, k=length))


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
        "trucks_count": problem.trucks_count,
        "drones_count": problem.drones_count,
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

    with ROOT.joinpath("result", f"{namespace.problem}-{index}-plot.py").open("w") as file:
        code = f"""
            from __future__ import annotations

            from typing import List

            from matplotlib import axes, pyplot


            x = {problem.x}
            y = {problem.y}
            dronable = {problem.dronable}
            customers_count = {problem.customers_count}

            _, ax = pyplot.subplots()
            assert isinstance(ax, axes.Axes)

            for paths in {drone_paths}:
                drone_x: List[float] = []
                drone_y: List[float] = []
                drone_u: List[float] = []
                drone_v: List[float] = []
                for path in paths:
                    for index in range(len(path) - 1):
                        current = path[index]
                        after = path[index + 1]

                        drone_x.append(x[current])
                        drone_y.append(y[current])
                        drone_u.append(x[after] - x[current])
                        drone_v.append(y[after] - y[current])

                ax.quiver(
                    drone_x,
                    drone_y,
                    drone_u,
                    drone_v,
                    color="cyan",
                    angles="xy",
                    scale_units="xy",
                    scale=1,
                    width=0.004,
                )

            for paths in {truck_paths}:
                truck_x: List[float] = []
                truck_y: List[float] = []
                truck_u: List[float] = []
                truck_v: List[float] = []
                for path in paths:
                    for index in range(len(path) - 1):
                        current = path[index]
                        after = path[index + 1]

                        truck_x.append(x[current])
                        truck_y.append(y[current])
                        truck_u.append(x[after] - x[current])
                        truck_v.append(y[after] - y[current])

                ax.quiver(
                    truck_x,
                    truck_y,
                    truck_u,
                    truck_v,
                    color="darkviolet",
                    angles="xy",
                    scale_units="xy",
                    scale=1,
                    width=0.004,
                )

            ax.scatter((0,), (0,), c="black", label="Depot")
            ax.scatter(
                [x[index] for index in range(1, 1 + customers_count) if dronable[index]],
                [y[index] for index in range(1, 1 + customers_count) if dronable[index]],
                c="darkblue",
                label="Dronable",
            )
            ax.scatter(
                [x[index] for index in range(1, 1 + customers_count) if not dronable[index]],
                [y[index] for index in range(1, 1 + customers_count) if not dronable[index]],
                c="red",
                label="Truck-only",
            )

            ax.annotate("0", (0, 0))
            for index in range(1, 1 + customers_count):
                ax.annotate(str(index), (x[index], y[index]))

            ax.grid(True)

            pyplot.legend()
            pyplot.show()
            pyplot.close()
        """
        code = f"# Auto-generated by {__file__}" + textwrap.dedent(code)
        file.write(code)
