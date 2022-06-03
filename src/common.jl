"By default every transition from a state to each neighbour costs 1"
defaultcost(s1, s2) = one(Int64)

"By default, the herustic returns 0 (Breadth First Search)"
defaultheuristic(state, goal) = zero(Int64)

defaultisgoal(state, goal) = state == goal

"reconstruct the path of states up to the found final node"
function reconstructpath(n)
  res = [n.data]
  while !isnothing(n.parent)
    n = n.parent
    push!(res, n.data)
  end
  return reverse!(res)
end