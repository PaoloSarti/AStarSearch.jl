# AStarSearch.jl
[![Build Status](https://github.com/PaoloSarti/AStarSearch.jl/workflows/CI/badge.svg)](https://github.com/PaoloSarti/AStarSearch.jl/actions?query=workflow%3ACI+branch%3Amaster)
[![codecov](https://codecov.io/gh/PaoloSarti/AStarSearch.jl/branch/main/graph/badge.svg?token=So4UrAd64G)](https://codecov.io/gh/PaoloSarti/AStarSearch.jl)


[A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in Julia.


This module exports the `astar` function that provides a generic implementation of the algorithm.
The type of the state is totally unrestricted, just provide the functions that give neighbour states and optionally an heuristic given a state and the goal and the algorithm will find the best path.

## Installation
In the Julia Pkg REPL, type: `add AStarSearch`

## Usage

`astar(neighbours, start, goal;
       heuristic=defaultheuristic, cost=defaultcost, isgoal=defaultisgoal, hashfn=hash, timeout=Inf, maxcost=Inf, maxdepth=Inf)`

Execute the A* algorithm to get the best path from the start state to reach a goal condition.
Only the first 3 arguments are mandatory, all the others are optional.

It returns a structure in which the `status` field is a Symbol that can be either:
- `:success`: the algorithm found a path from start to goal
- `:timeout`: the algorithm timed out, a partial path to the best state is returned in the `path` field
- `:nopath`: the algorithm didn't find any path to a goal, the path to the best state is still returned

The other fields are:
- `path`: an array of states from the start state to the goal or the best found state
- `cost`: the cost of the returned path
- `closedsetsize`: how many states the algorithm tested if they were a goal (size of the closed set)
- `opensetsize`: how many states were still in the open set when the algorithm ended

## Arguments
- `neighbours`: a function that takes a state and returns the neighbour states as an array (or iterable)
- `start`: the starting state, the type of the state is completely unrestricted
- `goal`: the goal state, the type is unrestricted, usually it's the same as the start
- `heuristic`: a function that given a state and the goal returns an estimate of the cost to reach goal. This estimate should be optimistic if you want to be sure to get the best path. Notice that the best path could be very expensive to find, so if you want a good but not guaranteed optimal path, you could multiply your heuristic by a constant, the algorithm will usually be much faster
- `cost`: a function that takes the current state and a neighbour and returns the cost to do that state transition. By default all transitions cost 1
- `isgoal`: a function that takes a state and the goal and evaluates if the goal is reached (by default ==)
- `hashfn`: a function that takes a state and returns a compact representation to use as dictionary key (usually one of UInt, Int, String), by default it is the base hash function. This is a very important field for composite states in order to avoid duplications. *WARNING* states with arrays as fields might return a different hash every time! If this is the case, please pass an hashfn that always returns the same value for the same state!
- `timeout`: timeout in number of seconds after which the algorithm stops returning the best partial path to the state with the lowest heuristic, by default it is unrestricted. Please notice that the algorithm wil run _AT LEAST_ the specified time.
- `maxcost`: a maximum bound of the accumulated cost of the path, this can result in a :nopath result even if a path to the goal (with a greater cost) exists. By default it is Inf
- `maxdepth`: the maximum depth the algorithm is allowed to go down while expanding the search state, the same considerations as the `maxcost` parameter apply. By default it is Inf

### Examples
It's a very general algorithm so you can solve shortest paths in mazes but also all sorts of puzzles such as the [15 Puzzle](https://en.wikipedia.org/wiki/15_puzzle).
Both the maze example and the 15 Puzzle solver are in the `test` folder.

If you want to find the best path in a maze using the manhattan heuristic you can do the following:
```julia
using Test
using AStarSearch

# Directions are seen as cartesian indexes so that they can be added to a position to get the next position
UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

# manhattan distance between positions in the maze matrix
manhattan(a::CartesianIndex, b::CartesianIndex) = sum(abs.((b - a).I))
# check to be in the maze and filter out moves that go into walls
getmazeneighbours(maze, state) = filter(x -> (1 <= x[1] <= size(maze)[1]) && (1 <= x[2] <= size(maze)[2]) && (!maze[x]), [state + d for d in DIRECTIONS])

# 0 = free cell, 1 = wall
maze = [0 0 1 0 0;
        0 1 0 0 0;
        0 1 0 0 1;
        0 0 0 1 1;
        1 0 1 0 0] .== 1
start = CartesianIndex(1, 1)
goal = CartesianIndex(1, 5)

function solvemaze(maze, start, goal)
  currentmazeneighbours(state) = getmazeneighbours(maze, state)
  return astar(currentmazeneighbours, start, goal, heuristic=manhattan)
end

res = solvemaze(maze, start, goal)
@test res.status == :success
@test res.path ==  CartesianIndex{2}[
    CartesianIndex(1, 1),
    CartesianIndex(2, 1),
    CartesianIndex(3, 1),
    CartesianIndex(4, 1),
    CartesianIndex(4, 2),
    CartesianIndex(4, 3),
    CartesianIndex(3, 3),
    CartesianIndex(2, 3),
    CartesianIndex(2, 4),
    CartesianIndex(1, 4),
    CartesianIndex(1, 5)]
@test res.cost == 10
```

### Breaking Changes
#### 0.3.0
The 0.3.0 release introduces a more strict type checking, requiring uniformity of types between the cost and the heuristics, to improve performance.  
If you get type errors, it will probably be because by default the cost is Int64, and you provided a Float heuristic.  
You can either provide the cost function that returns a float, or cast the heuristic to Int64.

#### 0.4.0
Since this release the base `astar` API changes, requiring the `neighbours` function as the first argument, and the second and third argument are the starting state and the goal state. All the other functions, `heuristic`, `cost`, and `isgoal`, are optional keyword arguments, and they now all expect 2 arguments (current state and goal/next state in the case of the `cost` function).
The subtyping API is the same, but the main method was renamed `astar`, instead of `search`

#### 0.5.0
Removed the `AbstractAStarSearch` base struct, the `astar` function is now the only supported interface by this package.
