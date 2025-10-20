"""
    defaultcost(state, neighbor) -> Int64

Default cost function used by A* search. Returns 1 for any state transition,
making all moves equally costly. Override this for weighted edges.
"""
defaultcost(s1, s2) = one(Int64)

"""
    defaultheuristic(state, goal) -> Int64

Default heuristic function used by A* search. Always returns 0, effectively
reducing A* to Dijkstra's algorithm (or Breadth-First Search with unit costs).
Override this with a problem-specific heuristic for better performance.
"""
defaultheuristic(state, goal) = zero(Int64)

"""
    defaultisgoal(state, goal) -> Bool

Default goal test function used by A* search. Uses `==` to compare states.
Override this for custom goal conditions or pattern matching.
"""
defaultisgoal(state, goal) = state == goal

"""
    reconstructpath(node::AStarNode) -> Vector

Reconstructs the solution path by following parent pointers from the goal node
back to the start node. Returns a vector of states in start-to-goal order.

# Arguments
- `node`: The final node (usually goal node) from which to reconstruct the path

# Returns
- Vector of states representing the path from start to the final node
"""
function reconstructpath(n)
  res = [n.data]
  while !isnothing(n.parent)
    n = n.parent
    push!(res, n.data)
  end
  return reverse!(res)
end
