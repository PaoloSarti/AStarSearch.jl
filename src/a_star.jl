import Base
using DataStructures

"""
    AStarNode{TState, TCost}

A node in the A* search tree, containing the state data and search metadata.

# Fields
- `data::TState`: The actual state data this node represents
- `g::TCost`: The cost from start to this node
- `f::TCost`: The total estimated cost (g + h) through this node to goal
- `parent::Union{AStarNode{TState, TCost}, Nothing}`: The predecessor node in the search tree
"""
mutable struct AStarNode{TState, TCost <: Number}
  data::TState
  g::TCost
  f::TCost
  parent::Union{AStarNode{TState, TCost}, Nothing}
end

"""
    isless(n1::AStarNode, n2::AStarNode)

Compare nodes by their f-values for priority queue ordering.
Nodes with lower f-values (estimated total cost) are considered smaller.
"""
Base.isless(n1::AStarNode, n2::AStarNode) = Base.isless(n1.f, n2.f)

"""
    AStarResult{TState, TCost}

Result of an A* search, containing the solution path and search statistics.

# Fields
- `status::Symbol`: Search result - `:success`, `:timeout`, or `:nopath`
- `path::Vector{TState}`: Sequence of states from start to goal (or best found)
- `cost::TCost`: Total cost of the path
- `closedsetsize::Int64`: Number of states explored (size of closed set)
- `opensetsize::Int64`: Number of states still to explore when search ended
"""
struct AStarResult{TState, TCost <: Number}
  status::Symbol
  path::Vector{TState}
  cost::TCost
  closedsetsize::Int64
  opensetsize::Int64
end

mutable struct AStarSearchState{TState, TCost <: Number, THash}
  openheap::Vector{AStarNode{TState, TCost}}
  opennodedict::Dict{THash, AStarNode{TState, TCost}}
  closedset::Set{THash}
  start_time::Float64
  best_node::AStarNode{TState, TCost}
  best_heuristic::TCost

  function AStarSearchState(
    start_node::AStarNode{TState, TCost},
    start_hash::THash,
    start_heuristic::TCost,
  ) where {TState, TCost, THash}
    start_time = time()
    closedset = Set{THash}()
    openheap = [start_node]
    opennodedict = Dict(start_hash => start_node)
    return new{TState, TCost, THash}(
      openheap,
      opennodedict,
      closedset,
      start_time,
      start_node,
      start_heuristic,
    )
  end
end

function _astar!(
  astar_state::AStarSearchState{TState, TCost, THash},
  neighbours,
  goal,
  heuristic,
  cost,
  isgoal,
  hashfn,
  timeout,
  maxcost,
  enable_closedset,
) where {TState, TCost <: Number, THash}
  while !isempty(astar_state.openheap)
    node = heappop!(astar_state.openheap)

    if isgoal(node.data, goal)
      return AStarResult{TState, TCost}(
        :success,
        reconstructpath(node),
        node.g,
        length(astar_state.closedset),
        length(astar_state.openheap),
      )
    end

    nodehash = hashfn(node.data)
    delete!(astar_state.opennodedict, nodehash)

    if timeout < Inf && time() - astar_state.start_time >= timeout
      return AStarResult{TState, TCost}(
        :timeout,
        reconstructpath(astar_state.best_node),
        astar_state.best_node.g,
        length(astar_state.closedset),
        length(astar_state.openheap),
      )
    end

    if enable_closedset
      push!(astar_state.closedset, nodehash)
    end

    nodeheuristic = node.f - node.g
    if nodeheuristic < astar_state.best_heuristic
      astar_state.best_heuristic = nodeheuristic
      astar_state.best_node = node
    end

    neighbour_states = neighbours(node.data)

    for neighbour in neighbour_states
      neighbourhash = hashfn(neighbour)
      if enable_closedset && neighbourhash in astar_state.closedset
        continue
      end

      gfromthisnode = node.g + cost(node.data, neighbour)

      if gfromthisnode > maxcost
        continue
      end

      if neighbourhash in keys(astar_state.opennodedict)
        neighbournode = astar_state.opennodedict[neighbourhash]
        if gfromthisnode < neighbournode.g
          neighbourheuristic = neighbournode.f - neighbournode.g
          neighbournode.g = gfromthisnode
          neighbournode.f = gfromthisnode + neighbourheuristic
          neighbournode.parent = node
          heapify!(astar_state.openheap)
        end
      else
        neighbourheuristic = heuristic(neighbour, goal)
        neighbournode = AStarNode{TState, TCost}(
          neighbour,
          gfromthisnode,
          gfromthisnode + neighbourheuristic,
          node,
        )
        heappush!(astar_state.openheap, neighbournode)
        push!(astar_state.opennodedict, neighbourhash => neighbournode)
      end
    end
  end

  return AStarResult{TState, TCost}(
    :nopath,
    reconstructpath(astar_state.best_node),
    astar_state.best_node.g,
    length(astar_state.closedset),
    length(astar_state.openheap),
  )
end

"""
  astar(neighbours, start, goal;
        heuristic=defaultheuristic, cost=defaultcost, isgoal=defaultisgoal, hashfn=hash, timeout=Inf, maxcost=Inf)

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

# Arguments
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
"""
function astar(
  neighbours,
  start,
  goal;
  heuristic = defaultheuristic,
  cost = defaultcost,
  isgoal = defaultisgoal,
  hashfn = hash,
  timeout = Inf,
  maxcost = Inf,
  enable_closedset = true,
  kwargs...,
)
  start_heuristic = heuristic(start, goal)
  start_cost = zero(start_heuristic)
  start_node = AStarNode(start, start_cost, start_heuristic, nothing)
  start_hash = hashfn(start)

  astar_state = AStarSearchState(start_node, start_hash, start_heuristic)

  return _astar!(
    astar_state,
    neighbours,
    goal,
    heuristic,
    cost,
    isgoal,
    hashfn,
    timeout,
    maxcost,
    enable_closedset,
  )
end
