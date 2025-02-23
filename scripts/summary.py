from __future__ import annotations

import itertools
import json
import re
import sqlite3
from string import ascii_lowercase
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


COLUMN_CHARS = set(ascii_lowercase + "_")


def header_to_column(header: str) -> str:
    header = header.lower().replace(" ", "_")
    header = "".join(c for c in header if c in COLUMN_CHARS)
    return header


if __name__ == "__main__":
    milp = compare()

    path = ROOT / "result" / "summary.csv"
    print(f"Writing to {path}")
    with path.open("w") as csv:
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
            "Drone route count",
            "Strategy",
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
                result["strategy"],
            ]
            csv.write(",".join(segments) + "\n")

    path = ROOT / "result" / "summary.db"
    print(f"Writing to {path}")

    path.unlink(missing_ok=True)
    with sqlite3.connect(path) as connection:
        cursor = connection.cursor()

        columns = [
            "problem TEXT NOT NULL",
            "customers_count INTEGER NOT NULL",
            "trucks_count INTEGER NOT NULL",
            "drones_count INTEGER NOT NULL",
            "iterations INTEGER NOT NULL",
            "tabu_size_factor INTEGER NOT NULL",
            "reset_after_factor INTEGER NOT NULL",
            "tabu_size INTEGER NOT NULL",
            "reset_after INTEGER NOT NULL",
            "max_elite_set_size INTEGER NOT NULL",
            "destroy_rate INTEGER NOT NULL",
            "energy_model TEXT NOT NULL",
            "speed_type TEXT NOT NULL",
            "range_type TEXT NOT NULL",
            "waiting_time_limit INTEGER NOT NULL",
            "truck_maximum_speed REAL",
            "endurance_fixed_time REAL",
            "endurance_drone_speed REAL",
            "cost REAL NOT NULL",
            "milp_cost REAL",
            "milp_performance REAL",
            "milp_status TEXT",
            "capacity_violation REAL NOT NULL",
            "energy_violation REAL NOT NULL",
            "waiting_time_violation REAL NOT NULL",
            "fixed_time_violation REAL NOT NULL",
            "truck_paths TEXT NOT NULL",
            "drone_paths TEXT NOT NULL",
            "truck_working_time TEXT NOT NULL",
            "drone_working_time TEXT NOT NULL",
            "feasible INTEGER NOT NULL",
            "initialization TEXT NOT NULL",
            "last_improved INTEGER NOT NULL",
            "elapsed REAL NOT NULL",
            "url TEXT",
            "weight_per_truck_route REAL NOT NULL",
            "customers_per_truck_route REAL NOT NULL",
            "truck_route_count INTEGER NOT NULL",
            "weight_per_drone_route REAL NOT NULL",
            "customers_per_drone_route REAL NOT NULL",
            "drone_route_count INTEGER NOT NULL",
            "strategy TEXT NOT NULL",
        ]

        query = "CREATE TABLE summary(" + ", ".join(columns) + ")"
        print(query)
        cursor.execute(query)

        query = "INSERT INTO summary VALUES (" + ", ".join(itertools.repeat("?", len(columns))) + ")"
        for result in result_reader():
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

            cursor.execute(
                query,
                (
                    result["problem"],
                    int(result["problem"].split(".")[0]),
                    result["trucks_count"],
                    result["drones_count"],
                    result["iterations"],
                    result["tabu_size_factor"],
                    result["reset_after_factor"],
                    result["tabu_size"],
                    result["reset_after"],
                    result["max_elite_size"],
                    result["destroy_rate"],
                    result["config"],
                    result["speed_type"],
                    result["range_type"],
                    result["waiting_time_limit"],
                    result["truck_maximum_speed"],
                    result["endurance_fixed_time"],
                    result["endurance_drone_speed"],
                    result["solution"]["cost"] / 60,
                    milp[result["problem"]]["Optimal"] if milp_feasible else None,  # type: ignore
                    milp_time,
                    str(milp[result["problem"]]["status"]) if milp_available else "",
                    result["solution"]["capacity_violation"],
                    result["solution"]["drone_energy_violation"],
                    result["solution"]["waiting_time_violation"],
                    result["solution"]["fixed_time_violation"],
                    str(result["solution"]["truck_paths"]),
                    str(result["solution"]["drone_paths"]),
                    str(result["solution"]["truck_working_time"]),
                    str(result["solution"]["drone_working_time"]),
                    int(result["solution"]["feasible"]),
                    result["initialization_label"],
                    result["last_improved"],
                    result["elapsed"],
                    result["url"],
                    truck_weight / truck_route_count if truck_route_count > 0 else 0,
                    truck_customers / truck_route_count if truck_route_count > 0 else 0,
                    truck_route_count,
                    drone_weight / drone_route_count if drone_route_count > 0 else 0,
                    drone_customers / drone_route_count if drone_route_count > 0 else 0,
                    drone_route_count,
                    result["strategy"],
                )
            )
