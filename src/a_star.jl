using DataStructures

"""Node of the state tree to explore

It has to be mutable because nodes can be updated in the open set heap, and to do so we need to keep a handle of the node in the heap"""
mutable struct Node
  data
  g
  h
  f
  parent
  heaphandle
end

"Results structure"
struct AStarResult
  status
  path
  cost
  exploredstates
  opensetsize
end

"order by f = g + h"
nodeorderingkey(n::Node) = n.f

"don't perform hashing by default, rely on the state structure itself"
defaulthash(x) = x

"By default each state and every neighbour are distant 1 move"
defaultdistance(s1, s2) = 1

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
          distance = defaultdistance, timeout = Inf, hashfn = defaulthash)

Execute the A* algorithm to get the best path from the start state to reach a goal condition.

It returns a structure in which the `status` field is a Symbol that can be either:
- `:success`: the algorithm found a path from start to goal
- `:timeout`: the algorithm timed out, a partial path to the best state is returned in the `path` field
- `:nopath`: the algorithm didn't find any path to a goal, the path to the best state is still returned

The other fields are:
- `path`: an array of states from the start state to the goal or the best found state
- `cost`: the cost of the returned path
- `exploredstates`: how many states the algorithm tested if they were a goal (size of the closed set)
- `opensetsize`: how many states were still in the open set when the algorithm ended

# Arguments
- `start`: the starting state, the type of the state is completely unrestricted
- `isgoal`: a function to evaluate if a state satisfies a goal condition
- `getneighbours`: a function that takes a state and returns the neighbour states as an array (or iterable)
- `heuristic`: a function that given a state returns an estimate of the distance to the goal. This estimate should be optimistic if you want to be sure to get the best path.
- `distance`: a function that takes the current state and a neighbour and returns the cost to do that state transition. By default all transitions cost 1
- `timeout`: timeout in number of seconds after which the algorithm stops returning the best partial path to the state with the lowest heuristic, by default it is unrestricted. Please notice that the algorithm wil run _AT LEAST_ the specified time
- `hashfn`: a function that takes a state and returns a compact representation to use as dictionary key (usually a string), by default it is just the identity function as the state is used directly as key

"""
function astar(start, isgoal, getneighbours, heuristic;
               distance = defaultdistance, timeout = Inf, hashfn = defaulthash)
  starttime = time()
  startheuristic = heuristic(start)
  startnode = Node(start, zero(distance(start, start)), startheuristic, startheuristic, nothing, nothing)
  bestnode = startnode
  starthash = hashfn(start)

  closedset = Set{typeof(starthash)}()
  openheap = MutableBinaryHeap(Base.By(nodeorderingkey), Node[])
  starthandle = push!(openheap, startnode)
  startnode.heaphandle = starthandle
  opennodedict = Dict(starthash=>startnode)

  while !isempty(openheap)
    if time() - starttime > timeout
      return AStarResult(:timeout, reconstructpath(bestnode), bestnode.g, length(closedset), length(openheap))
    end

    node = pop!(openheap)
    nodehash = hashfn(node.data)
    delete!(opennodedict, nodehash)

    if isgoal(node.data)
      return AStarResult(:success, reconstructpath(node), node.g, length(closedset), length(openheap))
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

      gfromthisnode = node.g + distance(node.data, neighbour)
      
      if neighbourhash in keys(opennodedict)
        neighbournode = opennodedict[neighbourhash]
        if gfromthisnode < neighbournode.g
          neighbournode.g = gfromthisnode
          neighbournode.parent = node
          neighbournode.f = gfromthisnode + neighbournode.h
          update!(openheap, neighbournode.heaphandle, neighbournode)
        end
      else
        neighbourheuristic = heuristic(neighbour)
        neighbournode = Node(neighbour, gfromthisnode, neighbourheuristic, gfromthisnode + neighbourheuristic, node, nothing)
        neighbourhandle = push!(openheap, neighbournode)
        neighbournode.heaphandle = neighbourhandle
        push!(opennodedict, neighbourhash => neighbournode)
      end
    end
  end

  return AStarResult(:nopath, reconstructpath(bestnode), bestnode.g, length(closedset), length(openheap))
end
