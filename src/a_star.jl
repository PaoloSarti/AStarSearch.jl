import Base
using DataStructures

"""Node of the state tree to explore"""
mutable struct Node{TState, TCost <: Number}
  data::TState
  g::TCost
  f::TCost
  parent::Union{Node{TState, TCost}, Nothing}
end

"order by f = g + h"
Base.isless(n1::Node, n2::Node) = Base.isless(n1.f, n2.f)

"Results structure"
struct AStarResult{TState, TCost <: Number}
  status::Symbol
  path::Array{TState,1}
  cost::TCost
  closedsetsize::Int64
  opensetsize::Int64
end

"By default every transition from a state to each neighbour costs 1"
defaultcost(s1, s2) = one(Int64)

"By default, the herustic returns 0 (Breadth First Search)"
defaultheuristic(state, goal) = zero(Int64)

defaultisgoal(state, goal) = state == goal

"reconstruct the path of states up to the found final node"
function reconstructpath(n::Node)
  res = [n.data]
  while !isnothing(n.parent)
    n = n.parent
    push!(res, n.data)
  end
  return reverse!(res)
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
"""
function astar(neighbours, start, goal;
               heuristic=defaultheuristic, cost=defaultcost, isgoal=defaultisgoal, hashfn=hash, timeout=Inf, maxcost=Inf)
  starttime = time()
  bestheuristic = heuristic(start, goal)
  startcost = zero(cost(start, start))
  nodetype = typeof(start)
  costtype = typeof(startcost)
  startnode = Node{nodetype, costtype}(start, startcost, bestheuristic, nothing)
  bestnode = startnode
  starthash = hashfn(start)

  closedset = Set{typeof(starthash)}()
  openheap = Node{nodetype, costtype}[]
  heappush!(openheap, startnode)
  opennodedict = Dict(starthash=>startnode)

  while !isempty(openheap)
    if timeout < Inf && time() - starttime > timeout
      return AStarResult{nodetype, costtype}(:timeout, reconstructpath(bestnode), bestnode.g, length(closedset), length(openheap))
    end

    node = heappop!(openheap)
    nodehash = hashfn(node.data)
    delete!(opennodedict, nodehash)

    if isgoal(node.data, goal)
      return AStarResult{nodetype, costtype}(:success, reconstructpath(node), node.g, length(closedset), length(openheap))
    end

    push!(closedset, nodehash)

    nodeheuristic = node.f - node.g
    if nodeheuristic < bestheuristic
      bestheuristic = nodeheuristic
      bestnode = node
    end

    neighbour_states = neighbours(node.data)

    for neighbour in neighbour_states
      neighbourhash = hashfn(neighbour)
      if neighbourhash in closedset
        continue
      end

      gfromthisnode = node.g + cost(node.data, neighbour)

      if gfromthisnode > maxcost
        continue
      end

      if neighbourhash in keys(opennodedict)
        neighbournode = opennodedict[neighbourhash]
        if gfromthisnode < neighbournode.g
          neighbourheuristic = neighbournode.f - neighbournode.g
          neighbournode.g = gfromthisnode
          neighbournode.f = gfromthisnode + neighbourheuristic
          neighbournode.parent = node
          heapify!(openheap)
        end
      else
        neighbourheuristic = heuristic(neighbour, goal)
        neighbournode = Node{nodetype, costtype}(neighbour, gfromthisnode, gfromthisnode + neighbourheuristic, node)
        heappush!(openheap, neighbournode)
        push!(opennodedict, neighbourhash => neighbournode)
      end
    end
  end

  return AStarResult{nodetype, costtype}(:nopath, reconstructpath(bestnode), bestnode.g, length(closedset), length(openheap))
end
