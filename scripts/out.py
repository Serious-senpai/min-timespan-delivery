from __future__ import annotations

import argparse
import json
import os
import random
import re
import string
import textwrap
from typing_extensions import List, Optional

from package import (
    HistoryJSON,
    NeighborhoodJSON,
    Problem,
    PrettySolutionJSON,
    ProgressJSON,
    PropagationJSON,
    ResultJSON,
    SolutionJSON,
    ROOT,
    csv_wrap,
    prettify,
)


def random_str(length: int) -> str:
    return "".join(random.choices(string.ascii_letters + string.digits, k=length))


class Namespace(argparse.Namespace):
    problem: str


def read_solution(*, cost: Optional[float] = None) -> SolutionJSON:
    if cost is None:
        cost = float(input())

    working_time = float(input())
    drone_energy_violation = float(input())
    capacity_violation = float(input())
    waiting_time_violation = float(input())
    fixed_time_violation = float(input())
    fixed_distance_violation = float(input())

    truck_paths: List[List[List[int]]] = eval(input())
    drone_paths: List[List[List[int]]] = eval(input())

    feasible = bool(int(input()))

    return {
        "cost": cost,
        "working_time": working_time,
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

    history: List[HistoryJSON[SolutionJSON]] = []
    while (cost := float(input())) != -1:
        s = read_solution(cost=cost)
        iteration = int(input())
        penalty_coefficients: List[float] = eval(input())

        s["cost"] = (
            s["working_time"]
            + s["drone_energy_violation"] * penalty_coefficients[0]
            + s["capacity_violation"] * penalty_coefficients[1]
            + s["waiting_time_violation"] * penalty_coefficients[2]
            + s["fixed_time_violation"] * penalty_coefficients[3]
            + s["fixed_distance_violation"] * penalty_coefficients[4]
        )

        history.append({"solution": s, "iteration": iteration, "penalty_coefficients": penalty_coefficients})

    progress: List[ProgressJSON[SolutionJSON]] = []
    while (cost := float(input())) != -1:
        s = read_solution(cost=cost)
        penalty_coefficients = eval(input())

        s["cost"] = (
            s["working_time"]
            + s["drone_energy_violation"] * penalty_coefficients[0]
            + s["capacity_violation"] * penalty_coefficients[1]
            + s["waiting_time_violation"] * penalty_coefficients[2]
            + s["fixed_time_violation"] * penalty_coefficients[3]
            + s["fixed_distance_violation"] * penalty_coefficients[4]
        )

        progress.append({"solution": s, "penalty_coefficients": penalty_coefficients})

    neighborhoods_size = int(input())
    neighborhoods: List[NeighborhoodJSON] = [{"label": input(), "pair": eval(input())} for _ in range(neighborhoods_size)]

    initialization_label = input()
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
        "history": history,
        "progress": progress,
        "neighborhoods": neighborhoods,
        "initialization_label": initialization_label,
        "last_improved": last_improved,
        "real": real,
        "user": user,
        "sys": sys,
    }
    print(solution)

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
        "history": [{"solution": prettify(h["solution"]), "iteration": h["iteration"], "penalty_coefficients": h["penalty_coefficients"]} for h in history],
        "progress": [{"solution": prettify(p["solution"]), "penalty_coefficients": p["penalty_coefficients"]} for p in progress],
    }

    json_output = ROOT / "result" / f"{namespace.problem}-{index}-pretty.json"
    with json_output.open("w") as file:
        json.dump(pretty_data, file, indent=4)

    print(f"Generated pretty JSON at {json_output.relative_to(working_dir)}")

    csv_output = ROOT / "result" / f"{namespace.problem}-{index}.csv"
    with csv_output.open("w") as file:
        file.write("sep=,\n")
        file.write("fitness,cost,a1,p1,a2,p2,a3,p3,a4,p4,a5,p5,Neighborhood,Pair,Truck routes,Drone routes\n")
        for p, neighborhood in zip(data["progress"], neighborhoods, strict=True):
            segments = [
                str(p["solution"]["cost"]),
                str(p["solution"]["working_time"]),
                str(p["penalty_coefficients"][0]),
                str(p["solution"]["drone_energy_violation"]),
                str(p["penalty_coefficients"][1]),
                str(p["solution"]["capacity_violation"]),
                str(p["penalty_coefficients"][2]),
                str(p["solution"]["waiting_time_violation"]),
                str(p["penalty_coefficients"][3]),
                str(p["solution"]["fixed_time_violation"]),
                str(p["penalty_coefficients"][4]),
                str(p["solution"]["fixed_distance_violation"]),
                csv_wrap(neighborhood["label"]),
                csv_wrap(neighborhood["pair"]),
                csv_wrap(p["solution"]["truck_paths"]),
                csv_wrap(p["solution"]["drone_paths"]),
            ]
            file.write(",".join(segments) + "\n")

    print(f"Generated CSV at {csv_output.relative_to(working_dir)}")

    pyplot_output = ROOT / "result" / f"{namespace.problem}-{index}-plot.py"
    with pyplot_output.open("w") as file:
        code = f"""
            from __future__ import annotations

            import argparse
            from typing_extensions import List, Literal, Tuple, TYPE_CHECKING

            from matplotlib import pyplot


            # Data region

            x = {problem.x}
            y = {problem.y}
            dronable = {problem.dronable}
            customers_count = {problem.customers_count}

            truck_paths: List[List[List[int]]] = {solution["truck_paths"]}
            drone_paths: List[List[List[int]]] = {solution["drone_paths"]}
            history: List[Tuple[float, bool, int]] = {[(h["solution"]["cost"], h["solution"]["feasible"], h["iteration"]) for h in history]}
            progress: List[Tuple[float, bool]] = {[(p["solution"]["cost"], p["solution"]["feasible"]) for p in progress]}
            penalty_coefficients: List[List[float]] = {[p["penalty_coefficients"] for p in progress]}

            # End of data region


            class Namespace(argparse.Namespace):
                if TYPE_CHECKING:
                    option: Literal["map", "history", "coeff"]


            def main(namespace: Namespace, /) -> None:
                _, ax = pyplot.subplots()
                if namespace.option == "map":
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

                elif namespace.option == "history":
                    history_costs: List[float] = []
                    history_feasible: List[bool] = []
                    for cost, feasible, iteration in history:
                        while len(history_costs) <= iteration:
                            history_costs.append(cost)

                        while len(history_feasible) <= iteration:
                            history_feasible.append(feasible)

                    while len(history_costs) < len(progress):
                        history_costs.append(history_costs[-1])

                    while len(history_feasible) < len(progress):
                        history_feasible.append(history_feasible[-1])

                    pyplot.plot(history_costs, label="History")
                    pyplot.axhline(y=cost, color="black", linestyle="--", label="Best")

                    pyplot.plot([p[0] for p in progress], label="Progress")

                    feasible_x = [i for i, feasible in enumerate(history_feasible) if feasible]
                    feasible_y = [history_costs[i] for i in feasible_x]
                    for iteration, (cost, feasible) in enumerate(progress):
                        if feasible:
                            feasible_x.append(iteration)
                            feasible_y.append(cost)

                    ax.scatter(feasible_x, feasible_y, s=7, c="green", label="Feasible")

                elif namespace.option == "coeff":
                    ax.set_yscale("log")
                    for i in range(5):
                        pyplot.plot([c[i] for c in penalty_coefficients], label="A%d" % i)

                else:
                    return

                pyplot.legend()
                pyplot.show()
                pyplot.close()


            if __name__ == "__main__":
                parser = argparse.ArgumentParser(
                    description="Plotting script for solution to problem {namespace.problem}",
                    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                )
                parser.add_argument("-o", "--option", default="history", choices=["map", "history", "coeff"], help="plotting option")

                namespace = Namespace()
                parser.parse_args(namespace=namespace)

                main(namespace)
        """
        code = f"# Auto-generated by {__file__}" + textwrap.dedent(code)
        file.write(code)

    print(f"Generated plot script at {pyplot_output.relative_to(working_dir)}")
