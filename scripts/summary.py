from __future__ import annotations

import json
from typing_extensions import Any, List

from package import ResultJSON, SolutionJSON, ROOT


def wrap(value: Any) -> str:
    return f"\"{value}\""


if __name__ == "__main__":
    ROOT.joinpath("result", "summary.json").unlink(missing_ok=True)

    results: List[ResultJSON[SolutionJSON]] = []
    for file in sorted(ROOT.joinpath("result").iterdir(), key=lambda f: f.name):
        if file.is_file() and file.name.endswith(".json") and not file.name.endswith("-pretty.json"):
            with file.open("r") as f:
                data = json.load(f)

            result = ResultJSON[SolutionJSON](**data)  # type: ignore  # will throw at runtime if fields are incompatible
            results.append(result)

    with ROOT.joinpath("result", "summary.csv").open("w") as csv:
        csv.write("sep=,\n")
        csv.write("Problem,Customers count,Trucks count,Drones count,Iterations,Tabu size,Energy model,Speed type,Range type,Cost,Capacity violation,Energy violation,Waiting time violation,Fixed time violation,Fixed distance violation,Truck paths,Drone paths,Feasible,Last improved,real,user,sys\n")
        for row, result in enumerate(results, start=2):
            segments = [
                wrap(result["problem"]),
                wrap(f"=VALUE(LEFT(A{row}, SEARCH(\"\".\"\", A{row}) - 1))"),
                str(result["trucks_count"]),
                str(result["drones_count"]),
                str(result["iterations"]),
                str(result["tabu_size"]),
                result["config"],
                result["speed_type"],
                result["range_type"],
                str(result["solution"]["cost"]),
                str(result["solution"]["capacity_violation"]),
                str(result["solution"]["drone_energy_violation"]),
                str(result["solution"]["waiting_time_violation"]),
                str(result["solution"]["fixed_time_violation"]),
                str(result["solution"]["fixed_distance_violation"]),
                wrap(result["solution"]["truck_paths"]),
                wrap(result["solution"]["drone_paths"]),
                str(int(result["solution"]["feasible"])),
                str(result["last_improved"]),
                str(result["real"]),
                str(result["user"]),
                str(result["sys"]),
            ]
            csv.write(",".join(segments) + "\n")

    with ROOT.joinpath("result", "summary.json").open("w") as f:
        json.dump(results, f, indent=4)
