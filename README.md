# AStar.jl
[![Build Status](https://github.com/PaoloSarti/AStar.jl/workflows/CI/badge.svg)](https://github.com/PaoloSarti/AStar.jl/actions?query=workflow%3ACI+branch%3Amaster)
[![codecov](https://codecov.io/gh/PaoloSarti/AStar.jl/branch/main/graph/badge.svg?token=So4UrAd64G)](https://codecov.io/gh/PaoloSarti/AStar.jl)
[A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in Julia.


This module exports the `astar` function that provides a generic implementation of the algorithm.
The type of the state is totally unrestricted, just provide the functions that give neighbour states and an heuristic given a state and the algorithm will find the best path.


This package was inspired by [this](https://www.npmjs.com/package/a-star) JavaScript package for its generic implementation

## Installation
In the Julia Pkg REPL, type: `add https://github.com/PaoloSarti/AStar.jl`

## Usage

`astar(start, isgoal, getneighbours, heuristic;
          distance = defaultdistance, timeout = Inf, hashfn = defaulthash)`

Execute the A* algorithm to get the best path from the start state to reach a goal condition.

### Result
It returns a structure in which the `status` field is a Symbol that can be either:
- `:success`: the algorithm found a path from start to goal
- `:timeout`: the algorithm timed out, a partial path to the best state is returned in the `path` field
- `:nopath`: the algorithm didn't find any path to a goal, the path to the best state is still returned

The other fields are:
- `path`: an array of states from the start state to the goal or the best found state
- `cost`: the cost of the returned path
- `exploredstates`: how many states the algorithm tested if they were a goal (size of the closed set)
- `opensetsize`: how many states were still in the open set when the algorithm ended

### Arguments
- `start`: the starting state, the type of the state is completely unrestricted
- `isgoal`: a function to evaluate if a state satisfies a goal condition
- `getneighbours`: a function that takes a state and returns the neighbour states as an array (or iterable)
- `heuristic`: a function that given a state returns an estimate of the distance to the goal. This estimate should be optimistic if you want to be sure to get the best path.
- `distance`: a function that takes the current state and a neighbour and returns the cost to do that state transition. By default all transitions cost 1
- `timeout`: timeout in number of seconds after which the algorithm stops returning the best partial path to the state with the lowest heuristic, by default it is unrestricted. Please notice that the algorithm wil run _AT LEAST_ the specified time
- `hashfn`: a function that takes a state and returns a compact representation to use as dictionary key (usually a string), by default it is just the identity function as the state is used directly as key

### Examples
It's a very general algorithm so you can solve shortest paths in mazes but also all sorts of puzzles such as the [15 Puzzle](https://en.wikipedia.org/wiki/15_puzzle).
Both the maze example and the 15 Puzzle solver are in the `test` folder.

If you want to find the best path in a maze using the manhattan heuristic you can do the following:
```julia
using Test
using AStar

# Directions are seen as cartesian indexes so that they can be added to a position to get the next position
UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

# manthattan distance between positions in the maze matrix
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

isgoal(state) = state == goal
getneighbours(state) = getmazeneighbours(maze, state)
heuristic(state) = manhattan(state, goal)

res = astar(start, isgoal, getneighbours, heuristic)
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
