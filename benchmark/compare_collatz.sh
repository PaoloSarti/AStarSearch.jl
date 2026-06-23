#!/usr/bin/env bash
# Run the Collatz A* benchmark in both Julia and Python and print results.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BENCH_DIR="$REPO_ROOT/benchmark"

echo "========================================"
echo "  Collatz A* benchmark comparison"
echo "========================================"
echo

echo "--- Julia (AStarSearch.jl) ---"
julia --project="$REPO_ROOT" --startup-file=no "$BENCH_DIR/collatz_standalone.jl"

echo
echo "--- Python (python-astar) ---"
cd "$BENCH_DIR/python"
uv run python collatz_bench.py
