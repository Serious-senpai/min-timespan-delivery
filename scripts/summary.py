from __future__ import annotations

import json
import re
from collections import defaultdict
from typing import Any, DefaultDict, List, TypedDict

from package import SolutionJSON, ROOT


DSAASolution = TypedDict("DSAASolution", {"best feasible": str, "best feasible score": str})


def wrap(value: Any) -> str:
    return f"\"{value}\""


if __name__ == "__main__":
    solutions: List[SolutionJSON] = []
    for file in sorted(ROOT.joinpath("result").iterdir(), key=lambda f: f.name):
        if file.is_file() and file.suffix == ".json":
            with file.open("r") as f:
                data = json.load(f)

            solution = SolutionJSON(**data)  # type: ignore  # will throw at runtime if fields are incompatible
            solutions.append(solution)

    compare: DefaultDict[str, float] = defaultdict(lambda: float("inf"))
    for file in ROOT.joinpath("problems", "solution").iterdir():
        if file.is_file() and file.suffix == ".json":
            with file.open("r") as f:
                data = json.load(f)

            assert isinstance(data, dict)
            for key, value in data.items():
                dsaa = DSAASolution(**value)
                assert isinstance(key, str)

                matcher = re.fullmatch(r"(\d+\.\d+\.\d+)\.txt\.\d+", key)
                assert matcher is not None

                problem = matcher.group(1)
                compare[problem] = min(compare[problem], 60 * float(dsaa["best feasible score"]))

    with ROOT.joinpath("result", "summary.csv").open("w") as csv:
        csv.write("Problem,Customers count,Iterations,Tabu size,Energy model,Speed type,Range type,Cost,DSAA,Improved [%],Capacity violation,Energy violation,Waiting time violation,Fixed time violation,Fixed distance violation,Truck paths,Drone paths,Feasible,Last improved,real,user,sys\n")
        for row, solution in enumerate(solutions, start=2):
            segments = [
                solution["problem"],
                wrap(f"=VALUE(LEFT(A{row}, SEARCH(\"\".\"\", A{row}) - 1))"),
                str(solution["iterations"]),
                str(solution["tabu_size"]),
                solution["config"],
                solution["speed_type"],
                solution["range_type"],
                str(solution["cost"]),
                str(compare.get(solution["problem"], "")),
                wrap(f"=ROUND(100 * (I{row} - H{row}) / I{row}, 2)") if solution["problem"] in compare else "",
                str(solution["capacity_violation"]),
                str(solution["drone_energy_violation"]),
                str(solution["waiting_time_violation"]),
                str(solution["fixed_time_violation"]),
                str(solution["fixed_distance_violation"]),
                wrap(solution["truck_paths"]),
                wrap(solution["drone_paths"]),
                str(int(solution["feasible"])),
                str(solution["last_improved"]),
                str(solution["real"]),
                str(solution["user"]),
                str(solution["sys"]),
            ]
            csv.write(",".join(segments) + "\n")
