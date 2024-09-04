from __future__ import annotations

import json
import re
from typing_extensions import Dict, Iterable

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


if __name__ == "__main__":
    ROOT.joinpath("result", "summary.json").unlink(missing_ok=True)

    compare_11 = compare()

    with ROOT.joinpath("result", "summary.csv").open("w") as csv:
        csv.write("sep=,\n")
        csv.write("Problem,Customers count,Trucks count,Drones count,Iterations,Tabu size,Energy model,Speed type,Range type,Cost,[11],Improved [%],Capacity violation,Energy violation,Waiting time violation,Fixed time violation,Truck paths,Drone paths,Feasible,Initialization,Last improved,real,user,sys\n")
        for row, result in enumerate(result_reader(), start=2):
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
                str(result["real"]),
                str(result["user"]),
                str(result["sys"]),
            ]
            csv.write(",".join(segments) + "\n")
