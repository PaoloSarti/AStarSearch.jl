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

"reconstruct the path of states up to the found final node"
function reconstructpath(n::Node)
  res = [n.data]
  while !isnothing(n.parent)
    n = n.parent
    push!(res, n.data)
  end
  return reverse(res)
end

function astar(start, isgoal, getneighbours, distance, heuristic, timeout = Inf, hashfn = defaulthash)
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

    neighboursdata = getneighbours(node.data)

    for neighbour in neighboursdata
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
