# AStarSearch.jl
[![Build Status](https://github.com/PaoloSarti/AStarSearch.jl/workflows/CI/badge.svg)](https://github.com/PaoloSarti/AStarSearch.jl/actions?query=workflow%3ACI+branch%3Amaster)
[![codecov](https://codecov.io/gh/PaoloSarti/AStarSearch.jl/branch/main/graph/badge.svg?token=So4UrAd64G)](https://codecov.io/gh/PaoloSarti/AStarSearch.jl)


[A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in Julia.


This module exports the `astar` function that provides a generic implementation of the algorithm.
The type of the state is totally unrestricted, just provide the functions that give neighbour states and an heuristic given a state and the algorithm will find the best path.

To ease the definition of more complex problems in which you usually want to search from the best path from start to end, given also some parameters from the problem instance, the `AbstractAStarSearch` type is introduced. You should define the neighbour function heuristic on the instance of your concrete AStarSearch type, with also that type in the signature, and then use the `search` function on that problem instance, given the start and goal state.

## Installation
In the Julia Pkg REPL, type: `add AStarSearch`

## Usage

`astar(start, isgoal, getneighbours, heuristic;
          cost = defaultcost, timeout = Inf, hashfn = defaulthash, maxcost = Inf, maxdepth = Inf)`

Execute the A* algorithm to get the best path from the start state to reach a goal condition.
Only the first 4 arguments are mandatory, all the others are optional.

### Result
It returns a structure in which the `status` field is a Symbol that can be either:
- `:success`: the algorithm found a path from start to goal
- `:timeout`: the algorithm timed out, a partial path to the best state is returned in the `path` field
- `:nopath`: the algorithm didn't find any path to a goal, the path to the best state is still returned

The other fields are:
- `path`: an array of states from the start state to the goal or the best found state
- `cost`: the cost of the returned path
- `closedsetsize`: how many states the algorithm tested if they were a goal (size of the closed set)
- `opensetsize`: how many states were still in the open set when the algorithm ended

### Arguments
- `start`: the starting state, the type of the state is completely unrestricted
- `isgoal`: a function to evaluate if a state satisfies a goal condition
- `getneighbours`: a function that takes a state and returns the neighbour states as an array (or iterable)
- `heuristic`: a function that given a state returns an estimate of the cost to reach goal. This estimate should be optimistic if you want to be sure to get the best path. Notice that the best path could be very expensive to find, so if you want a good but not guaranteed optimal path, you could multiply your heuristic by a constant, the algorithm will usually be much faster
- `cost`: a function that takes the current state and a neighbour and returns the cost to do that state transition. By default all transitions cost 1
- `timeout`: timeout in number of seconds after which the algorithm stops returning the best partial path to the state with the lowest heuristic, by default it is unrestricted. Please notice that the algorithm wil run _AT LEAST_ the specified time.
- `hashfn`: a function that takes a state and returns a compact representation to use as dictionary key (usually one of UInt, Int, String), by default it is just the identity function as the state is used directly as key. This is a very important field for composite states in order to avoid duplications
- `maxcost`: a maximum bound of the accumulated cost of the path, this can result in a :nopath result even if a path to the goal (with a greater cost) exists. By default it is Inf
- `maxdepth`: the maximum depth the algorithm is allowed to go down while expanding the search state, the same considerations as the `maxcost` parameter apply. By default it is Inf

### AbstractAStarSearch
Abstract Type that can be subtyped by concrete structures that represent a parametrizable problem.

Define a structure as subtype of AbstractAStarSearch, then you have to define:
- `neighbours(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType)` -> returns an array of neighbour states
- `heuristic(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType, goal::YourStateType)` -> returns an estimate of the cost to get to the end

And optionally you can redefine:
- `isgoal(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType, goal::YourStateType)` -> returns bool (by default it's current == goal)
- `cost(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType, neighbour::YourStateType)` -> returns the cost between the current state and a neighbour (by default = 1)

Then you can find the optimal path with:
`search(aastarsearch::YourAStarSearchStruct{YourStateType}, start::YourStateType, goal::YourStateType; timeout = Inf, maxcost = Inf, maxdepth = Inf)`


The other optional parameters are documented in the `astar` function above.

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

isgoal(state) = state == goal
getneighbours(state) = getmazeneighbours(maze, state)
mazeheuristic(state) = manhattan(state, goal)

res = astar(start, isgoal, getneighbours, mazeheuristic)
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

#### Maze Example by subtyping AbstractAStar
Given that `manhattan` and `getmazeneighbours` are defined as above, you can design the same solution like this:

```julia
import AStarSearch: neighbours, heuristic

struct MazeSolver <: AbstractAStarSearch{CartesianIndex{2}}
  maze:: BitArray{2}
end

neighbours(ms::MazeSolver, state::CartesianIndex{2}) = getmazeneighbours(ms.maze, state)
heuristic(ms::MazeSolver, state::CartesianIndex{2}, goal::CartesianIndex{2}) = manhattan(state, goal)

# 0 = free cell, 1 = wall
maze = [0 0 1 0 0;
        0 1 0 0 0;
        0 1 0 0 1;
        0 0 0 1 1;
        1 0 1 0 0] .== 1
mz = MazeSolver(maze)
start = CartesianIndex(1, 1)
goal = CartesianIndex(1, 5)

res = search(mz, start, goal)
```

The same results like above are returned.

### Breaking Changes
#### 0.3.0
The 0.3.0 release introduces a more strict type checking, requiring uniformity of types between the cost and the heuristics, to improve performance.  
If you get type errors, it will probably be because by default the cost is Int64, and you provided a Float heuristic.  
You can either provide the cost function that returns a float, or cast the heuristic to Int64.
