"""
    AStarSearch

A Julia package for efficient pathfinding and state-space search using the A* algorithm
and uninformed search algorithms.

# Main Features
- Generic A* implementation that works with any state type
- Memory-efficient implementation with optional closed set
- Uninformed search algorithms (DFS, BFS, iterative deepening)
- Customizable heuristic, cost, and neighbor functions
- Support for timeout and maximum cost constraints

# Exports
- Main function: `astar`
- Helper functions: `defaultheuristic`, `defaultcost`, `defaultisgoal`, `reconstructpath`
- Types: `AStarNode`, `AStarResult`, `UninformedSearchResult`
- Uninformed search: `depthfirst`, `breadthfirst`, `iterative_deepening`
"""
module AStarSearch

export astar, depthfirst, breadthfirst, iterative_deepening
export defaultheuristic, defaultcost, defaultisgoal, reconstructpath
export AStarNode, AStarResult, UninformedSearchResult

include("common.jl")
include("uninformed_search.jl")
include("a_star.jl")

end # module
