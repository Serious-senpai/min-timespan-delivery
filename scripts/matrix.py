from __future__ import annotations

import json
import sys
from typing import List, TypedDict


Config = TypedDict(
    "Config",
    {
        "instance": str,
        "step": int,
        "dronable": int,
        "drone-speed": int,
        "drones-count": int,
        "depot-location": int,
    }
)


instances = ["att48", "berlin52", "eil101", "gr120", "gr229", "pr152"]
steps = [0, 1]

dronable = [0, 20, 40, 60, 80, 100]
drone_speed = [1, 2, 3, 4, 5]
drones_count = [1, 2, 3, 4, 5]
depot_location = [0, 1]


DRONABLE_DEFAULT = 80
DRONE_SPEED_DEFAULT = 2
DRONES_COUNT_DEFAULT = 1
DEPOT_LOCATION_DEFAULT = 0


include: List[Config] = []
for instance in instances:
    for step in steps:

        _drone_speed = DRONE_SPEED_DEFAULT
        _drones_count = DRONES_COUNT_DEFAULT
        _depot_location = DEPOT_LOCATION_DEFAULT
        for _dronable in dronable:
            include.append(
                {
                    "instance": instance,
                    "step": step,
                    "dronable": _dronable,
                    "drone-speed": _drone_speed,
                    "drones-count": _drones_count,
                    "depot-location": _depot_location,
                }
            )

        _dronable = DRONABLE_DEFAULT
        _drones_count = DRONES_COUNT_DEFAULT
        _depot_location = DEPOT_LOCATION_DEFAULT
        for _drone_speed in drone_speed:
            if _drone_speed != 2:
                include.append(
                    {
                        "instance": instance,
                        "step": step,
                        "dronable": _dronable,
                        "drone-speed": _drone_speed,
                        "drones-count": _drones_count,
                        "depot-location": _depot_location,
                    }
                )

        _dronable = DRONABLE_DEFAULT
        _drone_speed = DRONE_SPEED_DEFAULT
        _depot_location = DEPOT_LOCATION_DEFAULT
        for _drones_count in drones_count:
            if _drones_count != 1:
                include.append(
                    {
                        "instance": instance,
                        "step": step,
                        "dronable": _dronable,
                        "drone-speed": _drone_speed,
                        "drones-count": _drones_count,
                        "depot-location": _depot_location,
                    }
                )

        _dronable = DRONABLE_DEFAULT
        _drone_speed = DRONE_SPEED_DEFAULT
        _drones_count = DRONES_COUNT_DEFAULT
        for _depot_location in depot_location:
            if _depot_location != 0:
                include.append(
                    {
                        "instance": instance,
                        "step": step,
                        "dronable": _dronable,
                        "drone-speed": _drone_speed,
                        "drones-count": _drones_count,
                        "depot-location": _depot_location,
                    }
                )


path = sys.argv[1]
print(f"Writing output to {path}")
with open(path, "w") as file:
    file.write("include=")
    file.write(json.dumps(include))
