from __future__ import annotations

import argparse
from typing import Literal, TYPE_CHECKING


__all__ = ("Namespace", "parser")


class Namespace(argparse.Namespace):
    if TYPE_CHECKING:
        problem: str
        iterations: int
        tabu_size_coefficient: int
        config: Literal["linear", "non-linear", "endurance"]
        speed_type: Literal["low", "high"]
        range_type: Literal["low", "high"]
        verbose: bool


parser = argparse.ArgumentParser(
    description="The min-timespan parallel technician-and-drone scheduling in door-to-door sampling service system",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("problem", type=str, help="the problem name in the archive")
parser.add_argument("-i", "--iterations", default=1000, type=int, help="the number of iterations to run the algorithm for")
parser.add_argument(
    "-t", "--tabu-size-coefficient",
    default=2,
    type=int,
    help="the tabu size coefficient each neighborhood, the final tabu size will be the product of this coefficient and the square root of customers count",
)
parser.add_argument("-c", "--config", default="linear", choices=["linear", "non-linear", "endurance"], help="the energy consumption model to use")
parser.add_argument("--speed-type", default="low", choices=["low", "high"], help="speed type of drones")
parser.add_argument("--range-type", default="low", choices=["low", "high"], help="range type of drones")
parser.add_argument("-v", "--verbose", action="store_true", help="the verbose mode")
