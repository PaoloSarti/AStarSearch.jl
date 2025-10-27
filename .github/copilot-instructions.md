<!-- Copilot instructions for AStarSearch.jl -->
# AStarSearch.jl — Guidance for AI coding agents

This repository implements a generic A* search and several uninformed search algorithms in Julia. Use these short, concrete rules so you can be productive without prior context.

- Primary entry points: `src/AStarSearch.jl` (module), `src/a_star.jl` (`astar`), and `src/uninformed_search.jl` (`depthfirst`, `breadthfirst`, `iterative_deepening`). Look at `src/common.jl` for default helpers (`defaultheuristic`, `defaultcost`, `reconstructpath`).

- Big picture: the package exports a single generic API: astar(neighbours, start, goal; ...). The algorithm is state-agnostic — callers pass in `neighbours` (state -> iterable), optional `heuristic(state, goal)`, `cost(state, neighbour)`, and `hashfn(state)`. Tests show composite-state examples (15-puzzle) where a custom `hash(::State)` is important for correctness and performance.

- Data flow and types:
  - `neighbours(state)` yields adjacent states (arrays or iterables).
  - `astar` uses `AStarNode{TState, TCost}` where `TCost<:Number`; heuristics and costs must have compatible numeric types (see README "Breaking Changes" and `a_star.jl` comments).
  - `hashfn` must return a stable compact key (Int/UInt/String). If state contains arrays, the default `hash` may vary — tests use a custom `Base.hash(::State)` that compresses a 15-puzzle board into a UInt64.

- Important implementation details to preserve when editing code:
  - `AStarNode` ordering uses `Base.isless` comparing `f = g + h`.
  - `opennodedict` maps `hashfn(state)` -> node; open set is a heap (`DataStructures.heappush!/heappop!`). When updating a node's g/f values the code calls `heapify!(openheap)`.
  - The algorithm tracks `best_node` and `best_heuristic` to return a best-effort path on timeout or no-path.

- Tests & developer workflows:
  - Run tests with the package environment: `julia --project=. -e "using Pkg; Pkg.test()"` (this runs `test/runtests.jl`). Tests use `Aqua` and `JuliaFormatter` checks; CI runs the same via `.github/workflows/ci.yml`.
  - Example problems are in `test/15puzzle.jl` and `test/maze.jl`. Use them as canonical examples of how to implement `neighbours`, `heuristic`, and `hash` for composite states.

- Conventions and patterns:
  - API-first, generic functions: prefer adding functionality by keeping the same astar signature and adding optional keyword args.
  - Avoid changing exported function signatures; if needed, add new optional keyword args and deprecation warnings.
  - `enable_closedset` defaults to true in most functions; some searches intentionally disable it (e.g., iterative deepening uses depth-first internally and sets closedset differently).

- Common pitfalls and how to fix them:
  - Mismatched numeric types: ensure `heuristic` and `cost` return the same numeric type (tests assume integer costs by default). If changing defaultcost/defaultheuristic, update tests accordingly.
  - Unstable hashing for array-containing states: add or recommend a `Base.hash(::YourState)` that returns an integer/UInt stable across runs (see `test/15puzzle.jl`).

- When editing tests/examples, follow existing style in `test/runtests.jl`: use `include("file.jl")` inside the testset, and preserve use of `Aqua` checks for project consistency.

- Quick file pointers (examples):
  - `test/15puzzle.jl` — composite state type, custom `Base.hash`, `neighbours` via `availablemoves`/`nextstates`, non-trivial heuristic (`heuristicmanhattanandswaps`). Use as canonical example for new problems.
  - `test/maze.jl` — simple CartesianIndex-based neighbours and Manhattan heuristic.
  - `src/a_star.jl` — core algorithm and `AStarResult` struct.

- Editing guidance for PRs produced by AI agents:
  - Run `Pkg.test()` locally and make sure `Aqua` checks still pass.
  - Keep changes minimal and focused: editing the core algorithm requires adding clear benchmarks or test updates (see `benchmark/`).
  - Preserve public API and exports in `src/AStarSearch.jl`.

If anything here is unclear or you want more examples (benchmarks, code snippets from the tests), tell me which area to expand and I will iterate.
