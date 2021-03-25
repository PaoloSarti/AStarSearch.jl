using DataStructures

"""Node of the state tree to explore

It has to be mutable because nodes can be updated in the open set heap, and to do so we need to keep a handle of the node in the heap"""
mutable struct Node{TCost <: Number}
  data::Any
  depth::Int32
  g::TCost
  h::TCost
  f::TCost
  parent::Union{Node{TCost}, Nothing}
  heaphandle::Union{Int64,Nothing}
end

"Results structure"
struct AStarResult{TState, TCost <: Number}
  status::Symbol
  path::Array{TState,1}
  cost::TCost
  closedsetsize::Int64
  opensetsize::Int64
end

"order by f = g + h"
nodeorderingkey(n::Node) = n.f

"don't perform hashing by default, rely on the state structure itself"
defaulthash(x) = x

"By default every transition from a state to each neighbour costs 1"
defaultcost(s1, s2) = one(Int64)

"reconstruct the path of states up to the found final node"
function reconstructpath(n::Node)
  res = [n.data]
  while !isnothing(n.parent)
    n = n.parent
    push!(res, n.data)
  end
  return reverse(res)
end

"""
  astar(start, isgoal, getneighbours, heuristic;
          cost = defaultcost, timeout = Inf, hashfn = defaulthash, maxcost = Inf, maxdepth = Inf)

Execute the A* algorithm to get the best path from the start state to reach a goal condition.
Only the first 4 arguments are mandatory, all the others are optional.

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
- `start`: the starting state, the type of the state is completely unrestricted
- `isgoal`: a function to evaluate if a state satisfies a goal condition
- `getneighbours`: a function that takes a state and returns the neighbour states as an array (or iterable)
- `heuristic`: a function that given a state returns an estimate of the cost to reach goal. This estimate should be optimistic if you want to be sure to get the best path. Notice that the best path could be very expensive to find, so if you want a good but not guaranteed optimal path, you could multiply your heuristic by a constant, the algorithm will usually be much faster
- `cost`: a function that takes the current state and a neighbour and returns the cost to do that state transition. By default all transitions cost 1
- `timeout`: timeout in number of seconds after which the algorithm stops returning the best partial path to the state with the lowest heuristic, by default it is unrestricted. Please notice that the algorithm wil run _AT LEAST_ the specified time.
- `hashfn`: a function that takes a state and returns a compact representation to use as dictionary key (usually one of UInt, Int, String), by default it is just the identity function as the state is used directly as key. This is a very important field for composite states in order to avoid duplications
- `maxcost`: a maximum bound of the accumulated cost of the path, this can result in a :nopath result even if a path to the goal (with a greater cost) exists. By default it is Inf
- `maxdepth`: the maximum depth the algorithm is allowed to go down while expanding the search state, the same considerations as the `maxcost` parameter apply. By default it is Inf
"""
function astar(start, isgoal, getneighbours, heuristic;
               cost = defaultcost, timeout = Inf, hashfn = defaulthash, maxcost = Inf, maxdepth = Inf)
  starttime = time()
  startheuristic = heuristic(start)
  startcost = zero(cost(start, start))
  startnode = Node(start, zero(Int32), startcost, startheuristic, startheuristic, nothing, nothing)
  bestnode = startnode
  starthash = hashfn(start)

  closedset = Set{typeof(starthash)}()
  openheap = MutableBinaryHeap(Base.By(nodeorderingkey), Node{typeof(startcost)}[])
  starthandle = push!(openheap, startnode)
  startnode.heaphandle = starthandle
  opennodedict = Dict(starthash=>startnode)

  while !isempty(openheap)
    if time() - starttime > timeout
      return AStarResult{typeof(start), typeof(startcost)}(:timeout, reconstructpath(bestnode), bestnode.g, length(closedset), length(openheap))
    end

    node = pop!(openheap)
    nodehash = hashfn(node.data)
    delete!(opennodedict, nodehash)

    if isgoal(node.data)
      return AStarResult{typeof(start), typeof(startcost)}(:success, reconstructpath(node), node.g, length(closedset), length(openheap))
    end

    push!(closedset, nodehash)

    if node.h < bestnode.h
      bestnode = node
    end

    neighbours = getneighbours(node.data)

    for neighbour in neighbours
      neighbourhash = hashfn(neighbour)
      if neighbourhash in closedset
        continue
      end

      gfromthisnode = node.g + cost(node.data, neighbour)

      if gfromthisnode > maxcost
        continue
      end

      if node.depth > maxdepth
        continue
      end

      if neighbourhash in keys(opennodedict)
        neighbournode = opennodedict[neighbourhash]
        if gfromthisnode < neighbournode.g
          neighbournode.depth = node.depth + one(Int32)
          neighbournode.g = gfromthisnode
          neighbournode.parent = node
          neighbournode.f = gfromthisnode + neighbournode.h
          update!(openheap, neighbournode.heaphandle, neighbournode)
        end
      else
        neighbourheuristic = heuristic(neighbour)
        neighbournode = Node(neighbour, node.depth + one(Int32), gfromthisnode, neighbourheuristic, gfromthisnode + neighbourheuristic, node, nothing)
        neighbourhandle = push!(openheap, neighbournode)
        neighbournode.heaphandle = neighbourhandle
        push!(opennodedict, neighbourhash => neighbournode)
      end
    end
  end

  return AStarResult{typeof(start), typeof(startcost)}(:nopath, reconstructpath(bestnode), bestnode.g, length(closedset), length(openheap))
end

"""Abstract Type that can be subtyped by concrete structures that represent a parametrizable problem.

Define a structure as subtype of AbstractAStarSearch, then you have to define:
- neighbours(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType) -> Returns an array of neighbour states
- heuristic(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType, goal::YourStateType) -> returns an estimate of the cost to get to the end

And optionally you can redefine:
- isgoal(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType, goal::YourStateType) -> returns bool (by default it's current == goal)
- cost(astarsearch::YourAStarSearchStruct{YourStateType}, current::YourStateType, neighbour::YourStateType) -> returns the cost between the current state and a neighbour (by default = 1)

Then you can find the optimal path with:
search(aastarsearch::YourAStarSearchStruct{YourStateType}, start::YourStateType, goal::YourStateType; timeout = Inf, maxcost = Inf, maxdepth = Inf)
"""
abstract type AbstractAStarSearch{T} end
isgoal(astarsearch::AbstractAStarSearch{T}, current::T, goal::T) where T = current == goal
neighbours(astarsearch::AbstractAStarSearch{T}, current::T) where T = throw("neighbours not implemented! Implement it for your AbstractAStarSearch subtype!")
heuristic(astarsearch::AbstractAStarSearch{T}, current::T, goal::T) where T = throw("heuristic not implemented! Implement it for your AbstractAStarSearch subtype!")
cost(astarsearch::AbstractAStarSearch{T}, current::T, neighbour::T) where T = defaultcost(current, neighbour)

"""search(aastarsearch::AbstractAStarSearch{T}, start::T, goal::T; timeout = Inf, maxcost = Inf, maxdepth = Inf)

Once defined the AbstractAStarSearch subtype for your problem (with at least the neighbours and heuristic methods),
execute the A* search algorithm given the problem instance, the start state and the goal state.

The other optional parameters are documented in the "astar" function.
"""
function search(aastarsearch::AbstractAStarSearch{T}, start::T, goal::T; timeout = Inf, maxcost = Inf, maxdepth = Inf) where T
  _isgoal(x::T) = isgoal(aastarsearch, x, goal)
  _neighbours(x::T) = neighbours(aastarsearch, x)
  _heuristic(x::T) = heuristic(aastarsearch, x, goal)
  _cost(current::T, neighbour::T) = cost(aastarsearch, current, neighbour)
  _hashfn(x::T) = hash(x)
  return astar(start, _isgoal, _neighbours, _heuristic, cost = _cost, hashfn = _hashfn; timeout, maxcost, maxdepth)
end
