from __future__ import annotations

import argparse
import json
import os
import random
import string
import textwrap
from typing_extensions import List, Optional, Tuple

from package import (
    NeighborhoodJSON,
    Problem,
    PrettySolutionJSON,
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
    url: Optional[str]


def read_solution() -> Optional[SolutionJSON]:
    cost = float(input())
    if cost == -1:
        return None

    working_time = float(input())
    drone_energy_violation = float(input())
    capacity_violation = float(input())
    waiting_time_violation = float(input())
    fixed_time_violation = float(input())

    truck_paths: List[List[List[int]]] = eval(input())
    drone_paths: List[List[List[int]]] = eval(input())

    feasible = bool(int(input()))
    truck_working_time: List[float] = eval(input())
    drone_working_time: List[float] = eval(input())

    return {
        "cost": cost,
        "working_time": working_time,
        "capacity_violation": capacity_violation,
        "drone_energy_violation": drone_energy_violation,
        "waiting_time_violation": waiting_time_violation,
        "fixed_time_violation": fixed_time_violation,
        "truck_paths": truck_paths,
        "drone_paths": drone_paths,
        "feasible": feasible,
        "truck_working_time": truck_working_time,
        "drone_working_time": drone_working_time,
    }


parser = argparse.ArgumentParser(
    description="The min-timespan parallel technician-and-drone scheduling in door-to-door sampling service system.\nAlgorithm output transformer.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("problem", type=str, help="the problem name in the archive")
parser.add_argument("--url", type=str, required=False, help="the GitHub Actions job URL")


if __name__ == "__main__":
    working_dir = os.getcwd()

    namespace = Namespace()
    parser.parse_args(namespace=namespace)

    iterations = int(input())

    tabu_size_factor = float(input())
    reset_after_factor = int(input())

    tabu_size = int(input())
    reset_after = int(input())
    max_elite_size = int(input())
    destroy_rate = int(input())

    config = input()
    speed_type = input()
    range_type = input()
    waiting_time_limit = float(input())

    truck_maximum_speed = float(input())
    endurance_fixed_time = float(input())
    endurance_drone_speed = float(input())

    problem = Problem.import_data(namespace.problem)

    solution = read_solution()
    assert solution is not None

    propagation: List[PropagationJSON[SolutionJSON]] = []
    while True:
        s = read_solution()
        if s is None:
            break

        label = input()
        propagation.append({"solution": s, "label": label})

    propagation.reverse()

    history = [read_solution() for _ in range(int(input()))]
    progress = [read_solution() for _ in range(int(input()))]
    coefficients: List[List[float]] = eval(input())

    neighborhoods: List[NeighborhoodJSON] = [{"label": input(), "pair": eval(input())} for _ in range(int(input()))]

    initialization_label = input()
    last_improved = int(input())

    elite_set: List[List[float]] = [eval(input()) for _ in range(int(input()))]
    elapsed = float(input()) / 1000  # Convert ms to s
    strategy = input()

    data: ResultJSON[SolutionJSON] = {
        "problem": namespace.problem,
        "trucks_count": len(solution["truck_paths"]),
        "drones_count": len(solution["drone_paths"]),
        "iterations": iterations,
        "tabu_size_factor": tabu_size_factor,
        "reset_after_factor": reset_after_factor,
        "tabu_size": tabu_size,
        "reset_after": reset_after,
        "max_elite_size": max_elite_size,
        "destroy_rate": destroy_rate,
        "config": config,
        "speed_type": speed_type,
        "range_type": range_type,
        "waiting_time_limit": waiting_time_limit,
        "truck_maximum_speed": truck_maximum_speed,
        "endurance_fixed_time": endurance_fixed_time,
        "endurance_drone_speed": endurance_drone_speed,
        "solution": solution,
        "propagation": propagation,
        "history": history,
        "progress": progress,
        "coefficients": coefficients,
        "neighborhoods": neighborhoods,
        "initialization_label": initialization_label,
        "last_improved": last_improved,
        "elite_set": elite_set,
        "elapsed": elapsed,
        "url": namespace.url,
        "strategy": strategy,
    }
    print(solution)

    ROOT.joinpath("result").mkdir(parents=True, exist_ok=True)

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
        "history": [prettify(h) for h in history],
        "progress": [prettify(p) for p in progress],
    }

    json_output = ROOT / "result" / f"{namespace.problem}-{index}-pretty.json"
    with json_output.open("w") as file:
        json.dump(pretty_data, file, indent=4)

    print(f"Generated pretty JSON at {json_output.relative_to(working_dir)}")

    csv_output = ROOT / "result" / f"{namespace.problem}-{index}.csv"
    with csv_output.open("w") as file:
        file.write("sep=,\n")
        file.write("Fitness,Working time,a1,p1,a2,p2,a3,p3,a4,p4,Neighborhood,Pair,Truck routes,Drone routes,Elite set costs\n")
        for row, (_progress, _coefficients, _neighborhood, _elite_set) in enumerate(zip(progress, coefficients, neighborhoods, elite_set, strict=True), start=2):
            iteration = row - 2
            segments = [
                csv_wrap(f"=B{row} + C{row} * D{row} + E{row} * F{row} + G{row} * H{row} + I{row} * J{row} + K{row}"),
                str("" if _progress is None else _progress["working_time"]),
                str(_coefficients[0]),
                str("" if _progress is None else _progress["drone_energy_violation"]),
                str(_coefficients[1]),
                str("" if _progress is None else _progress["capacity_violation"]),
                str(_coefficients[2]),
                str("" if _progress is None else _progress["waiting_time_violation"]),
                str(_coefficients[3]),
                str("" if _progress is None else _progress["fixed_time_violation"]),
                csv_wrap(_neighborhood["label"]),
                csv_wrap(_neighborhood["pair"]),
                csv_wrap("" if _progress is None else _progress["truck_paths"]),
                csv_wrap("" if _progress is None else _progress["drone_paths"]),
                csv_wrap(sorted(_elite_set, reverse=True)),
            ]
            file.write(",".join(segments) + "\n")

    print(f"Generated CSV at {csv_output.relative_to(working_dir)}")

    history_plot: Tuple[List[int], List[float], List[Tuple[int, float]]] = ([], [], [])
    progress_plot: Tuple[List[int], List[float], List[Tuple[int, float]]] = ([], [], [])
    for solutions, solution_plot in ((history, history_plot), (progress, progress_plot)):
        for i, s in enumerate(solutions):
            if s is not None:
                solution_plot[0].append(i)
                solution_plot[1].append(s["cost"])
                if s["feasible"]:
                    solution_plot[2].append((i, s["cost"]))

    feasible_plot: Tuple[List[int], List[float]] = ([], [])
    for x, y in (history_plot[2] + progress_plot[2]):
        feasible_plot[0].append(x)
        feasible_plot[1].append(y)

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
            history_plot: Tuple[List[int], List[float], List[Tuple[int, float]]] = {history_plot}
            progress_plot: Tuple[List[int], List[float], List[Tuple[int, float]]] = {progress_plot}
            feasible_plot: Tuple[List[int], List[float]] = {feasible_plot}
            penalty_coefficients: List[List[float]] = {coefficients}

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
                            width=0.002,
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
                            width=0.002,
                        )

                    ax.scatter((x[0],), (y[0],), c="black", label="Depot")
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

                    ax.annotate("0", (x[0], y[0]))
                    for index in range(1, 1 + customers_count):
                        ax.annotate(str(index), (x[index], y[index]))

                    ax.set_aspect("equal")
                    ax.grid(True)

                elif namespace.option == "history":
                    pyplot.plot(history_plot[0], history_plot[1], label="Best")
                    pyplot.plot(progress_plot[0], progress_plot[1], label="Progress")

                    ax.scatter(feasible_plot[0], feasible_plot[1], s=7, c="green", label="Feasible")

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
