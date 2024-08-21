from __future__ import annotations

import argparse
import json
import os
import random
import re
import string
import textwrap
from typing_extensions import List, Optional

from package import Problem, PrettySolutionJSON, PropagationJSON, ResultJSON, SolutionJSON, ROOT, prettify


def random_str(length: int) -> str:
    return "".join(random.choices(string.ascii_letters + string.digits, k=length))


class Namespace(argparse.Namespace):
    problem: str


def read_solution(*, cost: Optional[float] = None) -> SolutionJSON:
    if cost is None:
        cost = float(input())

    capacity_violation = float(input())
    drone_energy_violation = float(input())
    waiting_time_violation = float(input())
    fixed_time_violation = float(input())
    fixed_distance_violation = float(input())

    truck_paths: List[List[List[int]]] = eval(input())
    drone_paths: List[List[List[int]]] = eval(input())

    feasible = bool(int(input()))

    return {
        "cost": cost,
        "capacity_violation": capacity_violation,
        "drone_energy_violation": drone_energy_violation,
        "waiting_time_violation": waiting_time_violation,
        "fixed_time_violation": fixed_time_violation,
        "fixed_distance_violation": fixed_distance_violation,
        "truck_paths": truck_paths,
        "drone_paths": drone_paths,
        "feasible": feasible,
    }


parser = argparse.ArgumentParser(
    description="The min-timespan parallel technician-and-drone scheduling in door-to-door sampling service system.\nAlgorithm output transformer.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("problem", type=str, help="the problem name in the archive")


if __name__ == "__main__":
    working_dir = os.getcwd()

    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    iterations = int(input())
    tabu_size = int(input())
    config = input()
    speed_type = input()
    range_type = input()

    problem = Problem.import_data(namespace.problem)

    solution = read_solution()

    propagation: List[PropagationJSON[SolutionJSON]] = []
    while (cost := float(input())) != -1:
        s = read_solution(cost=cost)
        label = input()
        propagation.append({"solution": s, "label": label})

    propagation.reverse()

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

    data: ResultJSON[SolutionJSON] = {
        "problem": namespace.problem,
        "trucks_count": problem.trucks_count,
        "drones_count": problem.drones_count,
        "iterations": iterations,
        "tabu_size": tabu_size,
        "config": config,
        "speed_type": speed_type,
        "range_type": range_type,
        "solution": solution,
        "propagation": propagation,
        "last_improved": last_improved,
        "real": real,
        "user": user,
        "sys": sys,
    }
    print(data)

    index = random_str(8)
    while ROOT.joinpath("result", f"{namespace.problem}-{index}.json").exists():
        index = random_str(8)

    json_output = ROOT / "result" / f"{namespace.problem}-{index}.json"
    with json_output.open("w") as file:
        json.dump(data, file)

    print(f"Generated JSON at {json_output.relative_to(working_dir)}")

    pretty_data: ResultJSON[PrettySolutionJSON] = {
        **data,
        "solution": prettify(solution),
        "propagation": [{"solution": prettify(p["solution"]), "label": p["label"]} for p in propagation],
    }

    json_output = ROOT / "result" / f"{namespace.problem}-{index}-pretty.json"
    with json_output.open("w") as file:
        json.dump(pretty_data, file)

    print(f"Generated pretty JSON at {json_output.relative_to(working_dir)}")

    pyplot_output = ROOT / "result" / f"{namespace.problem}-{index}-plot.py"
    with pyplot_output.open("w") as file:
        code = f"""
            from __future__ import annotations

            from typing_extensions import List

            from matplotlib import pyplot


            x = {problem.x}
            y = {problem.y}
            dronable = {problem.dronable}
            customers_count = {problem.customers_count}

            truck_paths: List[List[List[int]]] = {solution["truck_paths"]}
            drone_paths: List[List[List[int]]] = {solution["drone_paths"]}

            _, ax = pyplot.subplots()

            for paths in drone_paths:
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

            for paths in truck_paths:
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

    print(f"Generated plot script at {pyplot_output.relative_to(working_dir)}")
