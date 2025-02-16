from __future__ import annotations

import json
import re
from typing_extensions import Dict, Iterable, Optional

from package import MILPResultJSON, Problem, ResultJSON, SolutionJSON, ROOT, csv_wrap


def compare() -> Dict[str, MILPResultJSON]:
    result: Dict[str, MILPResultJSON] = {}
    for file in ROOT.joinpath("problems", "milp").iterdir():
        match = re.search(r"\d+\.\d+\.\d+", file.name)
        if file.is_file() and file.name.endswith(".json") and match is not None:
            problem = match.group()
            with file.open("r") as f:
                result[problem] = json.load(f)

    return result


def result_reader() -> Iterable[ResultJSON[SolutionJSON]]:
    for file in sorted(ROOT.joinpath("result").iterdir(), key=lambda f: f.name):
        if file.is_file() and file.name.endswith(".json") and not file.name.endswith("-pretty.json"):
            print(file.absolute())
            with file.open("r") as f:
                data = json.load(f)

            yield data


if __name__ == "__main__":
    milp = compare()

    with ROOT.joinpath("result", "summary.csv").open("w") as csv:
        csv.write("sep=,\n")
        headers = [
            "Problem",
            "Customers count",
            "Trucks count",
            "Drones count",
            "Iterations",
            "Tabu size factor",
            "Reset after factor",
            "Tabu size",
            "Reset after",
            "Max elite set size",
            "Destroy rate",
            "Energy model",
            "Speed type",
            "Range type",
            "Waiting time limit",
            "Truck maximum speed",
            "Endurance fixed time",
            "Endurance drone speed",
            "Cost",
            "MILP cost",
            "Improved [%]",
            "MILP performance",
            "MILP status",
            "Capacity violation",
            "Energy violation",
            "Waiting time violation",
            "Fixed time violation",
            "Truck paths",
            "Drone paths",
            "Truck working time",
            "Drone working time",
            "Feasible",
            "Initialization",
            "Last improved",
            "Elapsed [s]",
            "URL",
            "Faster [%]",
            "Weight per truck route",
            "Customers per truck route",
            "Truck route count",
            "Weight per drone route",
            "Customers per drone route",
            "Drone route count,"
        ]
        csv.write(",".join(headers) + "\n")

        for row, result in enumerate(result_reader(), start=2):
            milp_available = result["problem"] in milp
            milp_feasible = milp_available and milp[result["problem"]]["status"] != "INFEASIBLE"

            milp_time: Optional[float] = None
            if milp_feasible:
                milp_time = milp[result["problem"]]["Solve_Time"]  # type: ignore
            elif milp_available:
                milp_time = 36000.0  # 10 hours without any feasible solutions

            truck_route_count = sum(len(routes) for routes in result["solution"]["truck_paths"])
            drone_route_count = sum(len(routes) for routes in result["solution"]["drone_paths"])

            problem = Problem.import_data(result["problem"])
            truck_weight = sum(sum(sum(problem.demands[c] for c in route) for route in routes) for routes in result["solution"]["truck_paths"])
            drone_weight = sum(sum(sum(problem.demands[c] for c in route) for route in routes) for routes in result["solution"]["drone_paths"])
            truck_customers = sum(sum(len(route) - 2 for route in routes) for routes in result["solution"]["truck_paths"])
            drone_customers = sum(sum(len(route) - 2 for route in routes) for routes in result["solution"]["drone_paths"])

            segments = [
                csv_wrap(result["problem"]),
                csv_wrap(f"=VALUE(LEFT(A{row}, SEARCH(\"\".\"\", A{row}) - 1))"),
                str(result["trucks_count"]),
                str(result["drones_count"]),
                str(result["iterations"]),
                str(result["tabu_size_factor"]),
                str(result["reset_after_factor"]),
                str(result["tabu_size"]),
                str(result["reset_after"]),
                str(result["max_elite_size"]),
                str(result["destroy_rate"]),
                result["config"],
                result["speed_type"],
                result["range_type"],
                str(result["waiting_time_limit"]),
                str(result["truck_maximum_speed"]),
                str(result["endurance_fixed_time"]),
                str(result["endurance_drone_speed"]),
                str(result["solution"]["cost"] / 60),
                str(milp[result["problem"]]["Optimal"]) if milp_feasible else "",  # type: ignore
                csv_wrap(f"=ROUND(100 * (T{row} - S{row}) / T{row}, 2)") if milp_feasible else "",
                str(milp_time) if milp_time is not None else "",
                str(milp[result["problem"]]["status"]) if milp_available else "",
                str(result["solution"]["capacity_violation"]),
                str(result["solution"]["drone_energy_violation"]),
                str(result["solution"]["waiting_time_violation"]),
                str(result["solution"]["fixed_time_violation"]),
                csv_wrap(result["solution"]["truck_paths"]),
                csv_wrap(result["solution"]["drone_paths"]),
                csv_wrap(result["solution"]["truck_working_time"]),
                csv_wrap(result["solution"]["drone_working_time"]),
                str(int(result["solution"]["feasible"])),
                result["initialization_label"],
                str(result["last_improved"]),
                str(result["elapsed"]),
                csv_wrap(result["url"]) if result["url"] is not None else "",
                csv_wrap(f"=ROUND(100 * (V{row} - AI{row}) / V{row}, 2)") if milp_time is not None else "",
                str(truck_weight / truck_route_count if truck_route_count > 0 else 0),
                str(truck_customers / truck_route_count if truck_route_count > 0 else 0),
                str(truck_route_count),
                str(drone_weight / drone_route_count if drone_route_count > 0 else 0),
                str(drone_customers / drone_route_count if drone_route_count > 0 else 0),
                str(drone_route_count),
            ]
            csv.write(",".join(segments) + "\n")
