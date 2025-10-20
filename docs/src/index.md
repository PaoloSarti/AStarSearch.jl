# AStarSearch.jl

A generic implementation of the A* search algorithm and several uninformed search algorithms in Julia.

## Features

- Generic A* search implementation that works with any state type
- Customizable heuristic, cost, and neighbor functions
- Memory efficient with optional closed set
- Additional uninformed search algorithms:
  - Depth-first search
  - Breadth-first search
  - Iterative deepening

## Quick Start

```julia
using AStarSearch

# Define how to get neighbors for your state
neighbors(state) = # your neighbor generation logic

# Optional: Define custom heuristic (defaults to 0)
heuristic(state, goal) = # your heuristic function

# Optional: Define custom cost function (defaults to 1)
cost(state, neighbor) = # your cost function

# Run A* search
path = astar(neighbors, start_state, goal_state; 
             heuristic=heuristic, 
             cost=cost)
```

## Installation

```julia
using Pkg
Pkg.add("AStarSearch")
```

For more detailed examples and usage, check out the Examples section.
