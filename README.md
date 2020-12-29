# AStar.jl
[A* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in Julia.


This module exports the `astar` function that provides a generic implementation of the algorithm.
The type of the state is totally unrestricted, just provide the functions that give neighbour states and an heuristic given a state and the algorithm will find the best path.


This package was inspired by [this](https://www.npmjs.com/package/a-star) implementation in JavaScript for its generic implementation

## Usage

astar(start, isgoal, getneighbours, heuristic;
          distance = defaultdistance, timeout = Inf, hashfn = defaulthash)

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
