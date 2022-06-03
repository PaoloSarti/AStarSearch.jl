using DataStructures

"Results structure"
struct UninformedSearchResult{TState}
  status::Symbol
  path::Vector{TState}
  closedsetsize::Int64
  opensetsize::Int64
end

struct UninformedSearchNode{TState}
  data::TState
  depth::Int64
  parent::Union{UninformedSearchNode{TState}, Nothing}
end

mutable struct DepthFirstSearchState{TState, THash}
  openset::Stack{UninformedSearchNode{TState}}
  closedset::Set{THash}
  start_time::Float64
end

function _depthfirst!(
  search_state::DepthFirstSearchState{TState, THash},
  neighbours,
  start,
  goal,
  isgoal,
  hashfn,
  timeout,
  maxdepth,
  enable_closedset,
) where {TState, THash}
  while !isempty(search_state.openset)
    node = pop!(search_state.openset)
    node_data = node.data
    if isgoal(node_data, goal)
      return UninformedSearchResult(
        :success,
        reconstructpath(node),
        length(search_state.closedset),
        length(search_state.openset),
      )
    end

    if timeout < Inf && time() - search_state.start_time >= timeout
      return UninformedSearchResult(
        :timeout,
        reconstructpath(node),
        length(search_state.closedset),
        length(search_state.openset),
      )
    end

    neighbours_data = collect(neighbours(node_data))
    # reverse so that the neighbours are checked in order
    reverse!(neighbours_data)
    for neighbour in neighbours_data
      new_depth = node.depth + 1
      if (enable_closedset && hashfn(neighbour) in search_state.closedset) ||
         new_depth > maxdepth
        continue
      end

      neighbour_node = UninformedSearchNode(neighbour, new_depth, node)
      push!(search_state.openset, neighbour_node)
    end

    if enable_closedset
      push!(search_state.closedset, hashfn(node_data))
    end
  end

  return UninformedSearchResult(
    :nopath,
    [start],
    length(search_state.closedset),
    length(search_state.openset),
  )
end

"""
Execute a depth-first search given the neighbours function, a start and a goal.
It takes the same parameters as astar, but it ignores those about cost and heuristic.
Use the `maxdepth` parameter to limit the search to a certain depth.
"""
function depthfirst(
  neighbours,
  start,
  goal;
  isgoal = defaultisgoal,
  hashfn = hash,
  timeout = Inf,
  maxdepth = typemax(Int64),
  enable_closedset = true,
  kwargs...,
)
  start_time = time()
  start_hash = hashfn(start)

  start_node = UninformedSearchNode(start, 0, nothing)
  stack = Stack{typeof(start_node)}()
  push!(stack, start_node)
  search_state = DepthFirstSearchState(stack, Set{typeof(start_hash)}(), start_time)

  return _depthfirst!(
    search_state,
    neighbours,
    start,
    goal,
    isgoal,
    hashfn,
    timeout,
    maxdepth,
    enable_closedset,
  )
end

"""
Execute iteratively a depth first search starting from depth 0 to maxdepth.
"""
function iterative_deepening(
  neighbours,
  start,
  goal;
  isgoal = defaultisgoal,
  hashfn = hash,
  timeout = Inf,
  maxdepth = typemax(Int64),
  enable_closedset = true,
  kwargs...,
)
  start_time = time()
  end_time = start_time + timeout
  closedsetsize = 0
  opensetsize = 0

  for depth = 0:maxdepth
    depth_first_timeout = max(end_time - time(), 0.0)
    res = depthfirst(
      neighbours,
      start,
      goal;
      isgoal,
      hashfn,
      enable_closedset,
      maxdepth = depth,
      timeout = depth_first_timeout,
      kwargs...,
    )
    if res.status in (:success, :timeout)
      return res
    end
    closedsetsize = res.closedsetsize
    opensetsize = res.opensetsize
  end
  return UninformedSearchResult(:nopath, [start], closedsetsize, opensetsize)
end

mutable struct BreadthFirstSearchState{TState, THash}
  openset::Deque{UninformedSearchNode{TState}}
  closedset::Set{THash}
  start_time::Float64
end

function _breadthfirst!(
  search_state::BreadthFirstSearchState{TState, THash},
  neighbours,
  start,
  goal,
  isgoal,
  hashfn,
  timeout,
  maxdepth,
  enable_closedset,
) where {TState, THash}
  while !isempty(search_state.openset)
    node = popfirst!(search_state.openset)
    node_data = node.data
    if isgoal(node_data, goal)
      return UninformedSearchResult(
        :success,
        reconstructpath(node),
        length(search_state.closedset),
        length(search_state.openset),
      )
    end

    if timeout < Inf && time() - search_state.start_time >= timeout
      return UninformedSearchResult(
        :timeout,
        reconstructpath(node),
        length(search_state.closedset),
        length(search_state.openset),
      )
    end

    for neighbour in neighbours(node_data)
      new_depth = node.depth + 1
      if (enable_closedset && hashfn(neighbour) in search_state.closedset) ||
         new_depth > maxdepth
        continue
      end

      neighbour_node = UninformedSearchNode(neighbour, new_depth, node)
      push!(search_state.openset, neighbour_node)
    end

    if enable_closedset
      push!(search_state.closedset, hashfn(node_data))
    end
  end

  return UninformedSearchResult(
    :nopath,
    [start],
    length(search_state.closedset),
    length(search_state.openset),
  )
end

"""
Executes a breadth first search given the neightbours function, the start and goal state.
"""
function breadthfirst(
  neighbours,
  start,
  goal;
  isgoal = defaultisgoal,
  hashfn = hash,
  timeout = Inf,
  maxdepth = typemax(Int64),
  enable_closedset = true,
  kwargs...,
)
  start_time = time()
  start_hash = hashfn(start)

  start_node = UninformedSearchNode(start, 0, nothing)
  deque = Deque{typeof(start_node)}()
  push!(deque, start_node)
  search_state = BreadthFirstSearchState(deque, Set{typeof(start_hash)}(), start_time)

  return _breadthfirst!(
    search_state,
    neighbours,
    start,
    goal,
    isgoal,
    hashfn,
    timeout,
    maxdepth,
    enable_closedset,
  )
end
