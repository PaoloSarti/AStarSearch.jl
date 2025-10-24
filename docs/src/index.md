# AStarSearch.jl

A generic implementation of the [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) and several uninformed search algorithms in Julia. The package provides tools for exploring state spaces efficiently using both informed (A*) and uninformed search strategies.

## Understanding A* Search

A* is a pathfinding and graph traversal algorithm that finds the shortest path between nodes in a graph. It achieves better performance than traditional approaches like Dijkstra's algorithm by using heuristics to guide its search.

### Core Concepts

#### State Space Exploration
The algorithm works by exploring a state space, where:
- Each **state** represents a configuration or position in your problem
- **Transitions** between states are determined by your `neighbours` function
- The goal is to find an optimal path from start state to goal state

#### How A* Works
1. **Open Set**: Contains states to be explored, ordered by their estimated total cost
2. **Closed Set**: Tracks already visited states to avoid cycles
3. **Cost Function (g)**: Actual cost from start to current state
4. **Heuristic Function (h)**: Estimated cost from current state to goal
5. **Total Estimated Cost (f)**: f = g + h

At each step, A* selects the most promising state (lowest f value) from the open set, which guides the search toward efficient paths.

### Algorithm Properties

- **Optimality**: If the heuristic is admissible (never overestimates), A* guarantees the optimal path
- **Memory Usage**: Uses a closed set to avoid revisiting states (optional in this implementation)
- **Performance**: Efficiency depends heavily on the quality of the heuristic function

## Features

- Generic implementation that works with any state type
- Fully customizable components:
  - `neighbours` function for state space generation
  - `heuristic` function for search guidance
  - `cost` function for transition costs
- Memory efficient with optional closed set
- Additional uninformed search algorithms:
  - Depth-first search
  - Breadth-first search
  - Iterative deepening

## Installation

```julia
using Pkg
Pkg.add("AStarSearch")
```

## Implementation Details

### State Space Generation

The `neighbours` function is the core of state space generation. It takes a state and returns an iterable of adjacent states:

```julia
function neighbours(state)
    # Generate and return all possible next states
    # Example: for a grid, return adjacent cells
    # Example: for a puzzle, return valid moves
    return next_states
end
```

### Search Guidance

Two key functions guide the search:

1. **Heuristic Function**
```julia
function heuristic(current, goal)
    # Estimate remaining cost to goal
    # Must be admissible (never overestimate)
    # Example: Manhattan distance for grid problems
    return estimated_cost
end
```

2. **Cost Function**
```julia
function cost(current, next)
    # Actual cost of transition between states
    # Example: Distance, time, or energy required
    return transition_cost
end
```

### Using the Algorithm

Basic usage with all optional parameters:

```julia
using AStarSearch

result = astar(neighbours,      # State generation function
               start_state,     # Initial state
               goal_state;      # Target state
               heuristic=h,     # Optional: guide search (default: 0)
               cost=c,          # Optional: transition cost (default: 1)
               hashfn=hash,     # Optional: state identification
               timeout=Inf,     # Optional: time limit
               maxcost=Inf)     # Optional: cost limit

# Result contains:
# - status: :success, :timeout, or :nopath
# - path: Array of states from start to goal/best found
# - cost: Total path cost
# - closedsetsize: Number of explored states
# - opensetsize: Remaining states to explore
```

### Important Implementation Details

#### Hash Function and State Management

The hash function plays a crucial role in A*'s efficiency and correctness:

```julia
function hashfn(state)
    # Return a stable, compact representation
    # Usually one of: UInt, Int, String
    return hash_value
end
```

Key considerations for `hashfn`:
- Used for both open set lookups and closed set checking
- Must be fast (called frequently during search)
- Must return consistent values for equal states
- Should produce compact representations to save memory

!!! warning "Julia Arrays and Hashing"
    States containing arrays require special attention. In Julia, the default `hash` 
    for structs with array fields may return different values for the same state 
    across multiple calls. Always provide a custom `hashfn` for such states that 
    ensures stable hashing.

Example of a stable hash function for a puzzle state:
```julia
struct PuzzleState
    board::Matrix{Int}
end

# Custom hash that produces consistent values
function Base.hash(state::PuzzleState, h::UInt)
    # Convert board to a single number for stable hashing
    value = 0
    for (i, n) in enumerate(state.board)
        value = value * 16 + n
    end
    return hash(value, h)
end
```

#### Performance vs Optimality Trade-offs

A* guarantees optimal paths when using admissible heuristics, but you can trade optimality for speed:

1. **Inflated Heuristics**
   ```julia
   # Original admissible heuristic
   h(state, goal) = manhattan_distance(state, goal)
   
   # Inflated for speed (may not find optimal path)
   h_fast(state, goal) = 1.5 * manhattan_distance(state, goal)
   ```

2. **Early Termination**
   - Use `maxcost` to limit path cost
   - Set `timeout` for time-constrained searches
   - Both return best-effort paths when limits are hit

3. **Memory vs Speed**
   - `enable_closedset=true` (default): Prevents revisiting states
   - `enable_closedset=false`: Uses less memory but may explore states multiple times
   - Consider disabling closed set for tree-like state spaces with no cycles

### Important Considerations

1. **Heuristic Design**
   - Must be admissible (never overestimate) for optimal paths
   - More accurate heuristics generally mean faster search
   - Can be intentionally inflated to trade optimality for speed

2. **State Representation**
   - Choose state types that can be efficiently compared and hashed
   - For composite states, provide a custom `hashfn`
   - Array-containing states need stable hash functions

3. **Memory Management**
   - Closed set prevents cycles but uses memory
   - Can be disabled with `enable_closedset=false`
   - Consider using `maxcost` to limit search space

For detailed examples and specific use cases, check out the Examples section.
