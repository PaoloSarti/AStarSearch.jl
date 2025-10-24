# AStarSearch.jl
[![Build Status](https://github.com/PaoloSarti/AStarSearch.jl/workflows/CI/badge.svg)](https://github.com/PaoloSarti/AStarSearch.jl/actions?query=workflow%3ACI+branch%3Amaster)
[![codecov](https://codecov.io/gh/PaoloSarti/AStarSearch.jl/branch/main/graph/badge.svg?token=So4UrAd64G)](https://codecov.io/gh/PaoloSarti/AStarSearch.jl)
[![Aqua QA](https://raw.githubusercontent.com/JuliaTesting/Aqua.jl/master/badge.svg)](https://github.com/JuliaTesting/Aqua.jl)
[![Documentation](https://img.shields.io/badge/docs-stable-blue.svg)](https://paolosarti.github.io/AStarSearch.jl)


[A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in Julia. Other [State Space Search](https://en.wikipedia.org/wiki/State_space_search) algorithms are also implemented as a baseline.


This package exports the `astar` function that provides a generic implementation of the algorithm.
The type of the state is totally unrestricted, just provide the functions that give neighbour states and optionally an heuristic given a state and the goal and the algorithm will find the best path.

Other implemented and exported functions to explore your state space are also exported: `depthfirst`, `breadthfirst`, `iterative_deepening`. These other functions are intended to be almost a transparent drop in replacement for the `astar` function, but they won't be able to use the heuristic.

## Installation
In the Julia Pkg REPL, type: `add AStarSearch`

## Usage

`astar(neighbours, start, goal;
       heuristic=defaultheuristic, cost=defaultcost, isgoal=defaultisgoal, hashfn=hash, timeout=Inf, maxcost=Inf)`

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
- `enable_closedset`: keep track of already visited nodes to avoid visiting them again, you might want to disable this if you know there isn't any loop in the state space graph (by default true)

### Uninformed Search Algorithms
This package exports also some uninformed search algorithms to compare the performance of astar, or to use when you don't have an explicit heuristic or cost. The interface of each function is almost identical, you can use any algorithm as a drop-in replacement for `astar`, but the parameters about cost and heuristic wil be ignored (`heuristic`, `cost`, `maxcost`), they have instead a `maxdepth` parameter to limit the depth of the search.

The exported functions are:
- `depthfirst`
- `breadthfirst`
- `iterative_deepening`

#### Depth First
This algorithm expands the neighbours as far as it can until there are no more neighbours for a state or the `maxdepth` limit is reached. The `enable_closedset` is enabled by default to avoid loops, but it might hide some paths to the goal if multiple paths are possible.

#### Breadth First
This algorithm searches first all the states at the same depth, ensuring that the best solution is always found, but it is usually slower.

#### Iterative Deepening
This algorithm iteratively executes depth first search at different levels of `maxdepth` having the same properties as breadth first, with almost the speed of depth first search. But also here the `enable_closedset` is disabled by default as it uses depth first internally.

### Examples
It's a very general algorithm so you can solve shortest paths in mazes but also all sorts of puzzles such as the [15 Puzzle](https://en.wikipedia.org/wiki/15_puzzle).
Both the maze example and the 15 Puzzle solver are in the `test` folder.

If you want to find the best path in a maze using the manhattan heuristic you can do the following:
```julia
using Test
using AStarSearch

# Directions are seen as cartesian indexes so that they can be added to a position to get the next position
const UP = CartesianIndex(-1, 0)
const DOWN = CartesianIndex(1, 0)
const LEFT = CartesianIndex(0, -1)
const RIGHT = CartesianIndex(0, 1)
const DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

# manhattan distance between positions in the maze matrix
manhattan(a::CartesianIndex, b::CartesianIndex) = sum(abs.((b - a).I))
# check to be in the maze and filter out moves that go into walls
function mazeneighbours(maze, p)
  res = CartesianIndex[]
  for d in DIRECTIONS
      n = p + d
      if 1 ≤ n[1] ≤ size(maze)[1] && 1 ≤ n[2] ≤ size(maze)[2] && !maze[n]
          push!(res, n)
      end
  end
  return res
end

function solvemaze(maze, start, goal)
  currentmazeneighbours(state) = mazeneighbours(maze, state)
  # Here you can use any of the exported search functions, they all share the same interface, but they won't use the heuristic and the cost
  return astar(currentmazeneighbours, start, goal, heuristic=manhattan)
end

# 0 = free cell, 1 = wall
maze = [0 0 1 0 0;
        0 1 0 0 0;
        0 1 0 0 1;
        0 0 0 1 1;
        1 0 1 0 0] .== 1
start = CartesianIndex(1, 1)
goal = CartesianIndex(1, 5)

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

### Changelog
See [CHANGELOG.md](CHANGELOG.md) for a list of notable changes across versions.
