from __future__ import annotations

import json
from typing_extensions import Dict, Iterable, Tuple

from package import ResultJSON, SolutionJSON, ROOT, csv_wrap


def result_reader() -> Iterable[ResultJSON[SolutionJSON]]:
    for file in sorted(ROOT.joinpath("result").iterdir(), key=lambda f: f.name):
        if file.is_file() and file.name.endswith(".json") and not file.name.endswith("-pretty.json"):
            print(file.absolute())
            with file.open("r") as f:
                yield json.load(f)


def get_compare() -> Dict[Tuple[str, int, float, int, int], float]:
    compare: Dict[Tuple[str, int, float, int, int], float] = {}
    with ROOT.joinpath("problems", "[0]", "tsplib-results.csv").open("r") as f:
        iterator = iter(f.readlines())

        # Skip 2 lines
        next(iterator)
        next(iterator)

        for line in iterator:
            instance, dronable, drone_speed, drones_count, depot_location, _, fast, rrls = line.split(",")
            compare[instance, int(dronable), float(drone_speed), int(drones_count), int(depot_location) - 1] = min(float(fast), float(rrls))

    return compare


if __name__ == "__main__":
    compare = get_compare()
    with ROOT.joinpath("result", "summary.csv").open("w") as csv:
        csv.write("sep=,\n")
        headers = [
            "Problem",
            # "Customers count",
            "Trucks count",
            "Drones count",
            "Drone speed",
            "TSP group",
            "Depot location",
            "Dronable percentage",
            "Iterations",
            "Tabu size factor",
            "Reset after factor",
            "Diversification factor",
            "Tabu size",
            "Reset after",
            "Diversification",
            "Max elite set size",
            "Energy model",
            "Speed type",
            "Range type",
            "Cost",
            csv_wrap("min(Fast, RRLS)"),
            "Improved [%]",
            "Capacity violation",
            "Energy violation",
            "Waiting time violation",
            "Fixed distance violation",
            "Truck paths",
            "Drone paths",
            "Feasible",
            "Initialization",
            "Last improved",
            "Elapsed [s]",
            "URL",
        ]
        csv.write(",".join(headers) + "\n")

        for row, result in enumerate(result_reader(), start=2):
            tokens = result["problem"].split("_")
            instance = tokens[0]
            depot_location = int(tokens[1])
            dronable = int(tokens[2])
            compare_key = instance, dronable, result["drone_speed"], result["drones_count"], depot_location

            segments = [
                csv_wrap(result["problem"]),
                # csv_wrap(f"=VALUE(LEFT(A{row}, SEARCH(\"\".\"\", A{row}) - 1))"),
                str(result["trucks_count"]),
                str(result["drones_count"]),
                str(result["drone_speed"]),
                instance,
                str(depot_location),
                str(dronable),
                str(result["iterations"]),
                str(result["tabu_size_factor"]),
                str(result["reset_after_factor"]),
                str(result["diversification_factor"]),
                str(result["tabu_size"]),
                str(result["reset_after"]),
                str(result["diversification"]),
                str(result["max_elite_size"]),
                result["config"],
                result["speed_type"],
                result["range_type"],
                str(result["solution"]["cost"]),
                str(compare.get(compare_key)),
                csv_wrap(f"=ROUND(100 * (T{row} - S{row}) / T{row}, 2)"),
                str(result["solution"]["capacity_violation"]),
                str(result["solution"]["drone_energy_violation"]),
                str(result["solution"]["waiting_time_violation"]),
                str(result["solution"]["fixed_time_violation"]),
                csv_wrap(result["solution"]["truck_paths"]),
                csv_wrap(result["solution"]["drone_paths"]),
                str(int(result["solution"]["feasible"])),
                result["initialization_label"],
                str(result["last_improved"]),
                str(result["elapsed"]),
                csv_wrap(result["url"]) if result["url"] is not None else "",
            ]
            csv.write(",".join(segments) + "\n")
