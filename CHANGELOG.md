# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.6.0] - 2025-10-24
### Removed
- Removed `maxdepth` parameter to improve memory usage, as `maxcost` is more powerful

## [0.5.0] - 2025-10-24
### Removed
- Removed `AbstractAStarSearch` base struct - the `astar` function is now the only supported interface by this package

## [0.4.0] - 2025-10-24
### Changed
- Changed base `astar` API to require `neighbours` function as first argument
- Changed second and third arguments to be starting state and goal state
- Made `heuristic`, `cost`, and `isgoal` functions optional keyword arguments
- Updated all functions to expect 2 arguments (current state and goal/next state for `cost` function)
- Renamed main method from `search` to `astar` in subtyping API

## [0.3.0] - 2025-10-24
### Changed
- Introduced more strict type checking, requiring uniformity of types between cost and heuristics to improve performance
  - By default, cost is `Int64`, so float heuristics need to be adjusted
  - Either provide a cost function that returns a float, or cast the heuristic to `Int64`

[Unreleased]: https://github.com/PaoloSarti/AStarSearch.jl/compare/v0.6.0...HEAD
[0.6.0]: https://github.com/PaoloSarti/AStarSearch.jl/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/PaoloSarti/AStarSearch.jl/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/PaoloSarti/AStarSearch.jl/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/PaoloSarti/AStarSearch.jl/releases/tag/v0.3.0