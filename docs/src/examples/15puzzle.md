# 15-Puzzle Example

The 15-puzzle is a classic sliding puzzle that consists of a 4×4 grid with 15 numbered tiles and one empty space. The goal is to rearrange the tiles from a given starting configuration to reach a specific goal state by sliding tiles into the empty space.

## Problem Setup

First, we define the state structure and some helper functions:

```julia
struct State
    table::Array{Int8, 2}
end

# Define movement directions
UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

# Goal state configuration
GOALSTATE = State([
    1  2  3  4
    5  6  7  8
    9  10 11 12
    13 14 15 0
])
FINALSTATEINDEXES = [findfirst(x -> x == i, GOALSTATE.table) for i = 1:15]
```

## State Manipulation

We need functions to find valid moves and generate next states:

```julia
# Find empty tile position
emptyposition(s::State) = findfirst(x -> x == 0, s.table)

# Check if a position is valid on the board
isvalidindex(c) = (0 < c[1] < 5) && (0 < c[2] < 5)

# Get available moves from current empty position
availablemoves(e::CartesianIndex) = 
    [direction for direction in DIRECTIONS if isvalidindex(e + direction)]
availablemoves(s::State) = s |> emptyposition |> availablemoves

function move(s::State, direction::CartesianIndex)
    next = copy(s.table)
    cur = emptyposition(s)
    emptydest = cur + direction
    next[cur], next[emptydest] = next[emptydest], next[cur]
    return State(next)
end

function move!(s::State, direction::CartesianIndex)
    cur = emptyposition(s)
    emptydest = cur + direction
    s.table[cur], s.table[emptydest] = s.table[emptydest], s.table[cur]
    return nothing
end

function randominitialstate(n)
    s = copy(GOALSTATE)
    for _ = 1:n
        curavailablemoves = availablemoves(s)
        selectedmove = rand(curavailablemoves)
        move!(s, selectedmove)
    end
    return s
end

# Generate next states by applying available moves
nextstates(s::State) = map(x -> move(s, x), availablemoves(s))
```

## Performance and correctness Considerations

For efficient state comparison and storage in the closed set, we provide a custom hash function that compresses the entire board state into a single 64-bit integer:

```julia
Base.copy(s::State) = State(copy(s.table))
Base.:(==)(s1::State, s2::State) = s1.table == s2.table

function Base.hash(s::State)
    h = UInt64(0)
    for j = 1:4, i = 1:4
        h += s.table[i, j]
        h <<= 4
    end
    return h
end
```

## Heuristic Function

For efficient pathfinding, we use a combination of Manhattan distance and swap count:

```julia
manhattan(c1::CartesianIndex, c2::CartesianIndex) = c1 - c2 |> x -> x.I .|> abs |> sum

function countswaps(s::State)
  acc = 0
  for j = 1:3, i = 1:3
    # horizontal swap
    if (s.table[i, j] == GOALSTATE.table[i + 1, j]) &&
        (s.table[i + 1, j] == GOALSTATE.table[i, j])
        acc += 1
    end
    #vertical swap
    if (s.table[i, j] == GOALSTATE.table[i, j + 1]) &&
        (s.table[i, j + 1] == GOALSTATE.table[i, j])
        acc += 1
    end
  end
  return acc
end

function heuristicmanhattanandswaps(s::State)
    # Manhattan distance component
    manhattan_sum = sum(manhattan(findfirst(x -> x == i, s.table), 
                                FINALSTATEINDEXES[i]) for i = 1:15)
    
    # Swap count component
    swaps = countswaps(s)
    
    return manhattan_sum + 2*swaps
end

# For A* search interface
heuristic(s, goal) = heuristicmanhattanandswaps(s)
```

## Solving the Puzzle

Now we can solve the puzzle using A* search:

```julia
using AStarSearch

# Example starting state
start = State([
    1 2 3 4
    5 7 11 8
    9 6 0 12
    13 10 14 15
])

function solve_15_puzzle(start::State, heuristic=heuristic; kwargs...)
    return astar(nextstates, start, GOALSTATE; heuristic=heuristic, kwargs...)
end

# Find solution path
result = solve_15_puzzle(start)

function print_result(result)
    if result.status == :success
        println("Solution found in $(result.cost) moves!")
        # Path contains the sequence of states from start to goal
        for state in result.path
            display(state.table)
            println()
        end
    else
        println("No solution found.")
    end
end

print_result(result)
```

This example demonstrates how AStarSearch.jl can be used to solve complex puzzles while maintaining good performance through careful state representation and heuristic design.
