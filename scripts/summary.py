from __future__ import annotations

import itertools
import json
import re
from typing_extensions import Dict, Iterable, List

from package import ResultJSON, SolutionJSON, ROOT, csv_wrap


def compare() -> Dict[str, float]:
    result: Dict[str, float] = {}
    with ROOT.joinpath("problems", "[11]", "bestfoundsolutions-mincost.txt").open("r") as f:
        for match in re.finditer(r"^(\d+-\w+-[012]-\w+)\.txt\s+Cost\s+=\s+(\d+(?:\.\d+)?)$", f.read(), re.MULTILINE):
            result[match.group(1)] = float(match.group(2))

    return result


def result_reader() -> Iterable[ResultJSON[SolutionJSON]]:
    for file in sorted(ROOT.joinpath("result").iterdir(), key=lambda f: f.name):
        if file.is_file() and file.name.endswith(".json") and not file.name.endswith("-pretty.json"):
            print(file.absolute())
            with file.open("r") as f:
                yield json.load(f)


def hamming_distance(
    customers_count: int,
    truck_paths_1: List[List[List[int]]],
    drone_paths_1: List[List[List[int]]],
    truck_paths_2: List[List[List[int]]],
    drone_paths_2: List[List[List[int]]],
) -> int:
    repr_1 = [0] * customers_count
    for vehicle_paths in (truck_paths_1, drone_paths_1):
        for paths in vehicle_paths:
            for path in paths:
                for i in range(1, len(path) - 2):
                    repr_1[path[i]] = path[i + 1]

    repr_2 = [0] * customers_count
    for vehicle_paths in (truck_paths_2, drone_paths_2):
        for paths in vehicle_paths:
            for path in paths:
                for i in range(1, len(path) - 2):
                    repr_2[path[i]] = path[i + 1]

    return sum(1 for a, b in zip(repr_1, repr_2) if a != b)


if __name__ == "__main__":
    with ROOT.joinpath("problems", "[11]", "bestfoundsolutions-mincost.txt").open("r") as optimal_file:
        optimal_data = optimal_file.read()

    compare_11 = compare()

    with ROOT.joinpath("result", "summary.csv").open("w") as csv:
        csv.write("sep=,\n")
        headers = [
            "Problem",
            "Customers count",
            "Trucks count",
            "Drones count",
            "Iterations",
            "Tabu size",
            "Energy model",
            "Speed type",
            "Range type",
            "Cost",
            "[11]",
            "Improved [%]",
            "Capacity violation",
            "Energy violation",
            "Waiting time violation",
            "Fixed time violation",
            "Truck paths",
            "Drone paths",
            "Feasible",
            "Initialization",
            "Last improved",
            "Elapsed",
            "URL",
            "Minimum Hamming distance",
            "Suboptimal cost",
            "Suboptimal travel cost",
            "Suboptimal truck paths",
            "Suboptimal drone paths",
        ]
        csv.write(",".join(headers) + "\n")

        for row, result in enumerate(result_reader(), start=2):
            optimal_iter = iter(optimal_data.split("\n"))
            optimal_truck_paths: List[List[List[int]]] = [[] for _ in range(result["trucks_count"])]
            optimal_drone_paths: List[List[List[int]]] = [[] for _ in range(result["drones_count"])]
            for line in optimal_iter:
                if line == result["problem"] + ".txt":
                    next(optimal_iter)  # Skip "Cost = ..."

                    while True:
                        line = next(optimal_iter)
                        if line == "---":
                            break

                        tokens = line.split()
                        if match := re.fullmatch(r"T(\d+)", tokens[0]):
                            truck = int(match.group(1))
                            optimal_truck_paths[truck].append(list(map(int, tokens[2:])))
                        elif match := re.fullmatch(r"D(\d+)", tokens[0]):
                            drone = int(match.group(1))
                            optimal_drone_paths[drone].extend([0, c, 0] for c in map(int, tokens[2:]))

                    break

            n = int(result["problem"].split("-")[0]) + 1
            suboptimal_solution = result["solution"]
            min_distance = hamming_distance(
                n,
                optimal_truck_paths,
                optimal_drone_paths,
                suboptimal_solution["truck_paths"],
                suboptimal_solution["drone_paths"],
            )

            for solution in itertools.chain(result["progress"], result["history"], map(lambda p: p["solution"], result["propagation"])):
                if solution is not None:
                    d = hamming_distance(
                        n,
                        optimal_truck_paths,
                        optimal_drone_paths,
                        solution["truck_paths"],
                        solution["drone_paths"],
                    )
                    if (
                        d < min_distance
                        or (
                            d == min_distance
                            and (solution["cost"] < suboptimal_solution["cost"])
                            and (solution["feasible"] or not suboptimal_solution["feasible"])
                        )
                    ):
                        min_distance = d
                        suboptimal_solution = solution

            segments = [
                csv_wrap(result["problem"]),
                csv_wrap(f"=VALUE(LEFT(A{row}, SEARCH(\"\"-\"\", A{row}) - 1))"),
                str(result["trucks_count"]),
                str(result["drones_count"]),
                str(result["iterations"]),
                str(result["tabu_size"]),
                result["config"],
                result["speed_type"],
                result["range_type"],
                str(result["solution"]["cost"]),
                str(compare_11.get(result["problem"], "")),
                csv_wrap(f"=ROUND(100 * (K{row} - J{row}) / K{row}, 2)"),
                str(result["solution"]["capacity_violation"]),
                str(result["solution"]["drone_energy_violation"]),
                str(result["solution"]["working_time_violation"]),
                str(result["solution"]["fixed_time_violation"]),
                csv_wrap(result["solution"]["truck_paths"]),
                csv_wrap(result["solution"]["drone_paths"]),
                str(int(result["solution"]["feasible"])),
                result["initialization_label"],
                str(result["last_improved"]),
                str(result["elapsed"]),
                csv_wrap(result["url"]) if result["url"] is not None else "",
                str(min_distance),
                str(suboptimal_solution["cost"]),
                str(suboptimal_solution["travel_cost"]),
                csv_wrap(suboptimal_solution["truck_paths"]),
                csv_wrap(suboptimal_solution["drone_paths"]),
            ]
            csv.write(",".join(segments) + "\n")
