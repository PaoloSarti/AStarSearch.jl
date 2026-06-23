"""Collatz A* benchmark using python-astar (jrialland/python-astar)."""

from __future__ import annotations

import math
import statistics
import time

from astar import AStar


class CollatzSolver(AStar):
    def neighbors(self, n: int):
        if n % 2 == 0:
            return [n // 2, 3 * n + 1]
        return [3 * n + 1]

    def heuristic_cost_estimate(self, current: int, goal: int) -> float:
        return math.floor(math.log2(current)) if current > 0 else 0

    def distance_between(self, n1: int, n2: int) -> float:
        return 1


SOLVER = CollatzSolver()
NUMBERS = [1, 34, 523, 1345, 8765]
MIN_RUNS = 3
BUDGET_S = 5.0  # max seconds per number


def bench(n: int) -> tuple[float, float]:
    """Return (median_ns, min_ns); runs until BUDGET_S elapsed (min MIN_RUNS)."""
    times = []
    deadline = time.perf_counter() + BUDGET_S
    while len(times) < MIN_RUNS or time.perf_counter() < deadline:
        t0 = time.perf_counter_ns()
        SOLVER.astar(n, 1)
        t1 = time.perf_counter_ns()
        times.append(t1 - t0)
    return statistics.median(times), min(times)


if __name__ == "__main__":
    print(f"{'n':>6}  {'median':>12}  {'min':>12}")
    print("-" * 36)
    for n in NUMBERS:
        median_ns, min_ns = bench(n)
        print(f"{n:>6}  {median_ns / 1e3:>10.1f}µs  {min_ns / 1e3:>10.1f}µs")
