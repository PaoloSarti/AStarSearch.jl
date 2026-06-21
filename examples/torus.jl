using AStarSearch
using Random

# ---- Grid dimensions (change here for other sizes) ----
const ROWS = 4
const COLS = 4

# ---- State ----

struct TorusState
  grid::Matrix{Int8}
end

Base.copy(s::TorusState) = TorusState(copy(s.grid))
Base.:(==)(a::TorusState, b::TorusState) = a.grid == b.grid
Base.hash(s::TorusState) = hash(s.grid)

function Base.show(io::IO, s::TorusState)
  w = ndigits(ROWS * COLS) + 1
  for i = 1:ROWS
    println(io, join(lpad.(Int.(s.grid[i, :]), w), ""))
  end
  return nothing
end

# ---- Goal state ----
# Goal: grid[i,j] = (i-1)*COLS + j
const GOAL_STATE = TorusState(Matrix{Int8}(reshape(1:(ROWS * COLS), COLS, ROWS)'))

# ---- Moves ----
# A move shifts an entire row (left/right) or column (up/down) cyclically.

function nextstates(s::TorusState)
  states = Vector{TorusState}(undef, 2 * (ROWS + COLS))
  k = 1
  for i = 1:ROWS
    g = copy(s.grid)
    g[i, :] = circshift(g[i, :], -1)
    states[k] = TorusState(g)
    k += 1
    g = copy(s.grid)
    g[i, :] = circshift(g[i, :], 1)
    states[k] = TorusState(g)
    k += 1
  end
  for j = 1:COLS
    g = copy(s.grid)
    g[:, j] = circshift(g[:, j], -1)
    states[k] = TorusState(g)
    k += 1
    g = copy(s.grid)
    g[:, j] = circshift(g[:, j], 1)
    states[k] = TorusState(g)
    k += 1
  end
  return states
end

# ---- Heuristic ----
# h(s) = ⌈total_horizontal_dist / COLS⌉ + ⌈total_vertical_dist / ROWS⌉
# Admissible and consistent: one row/col shift reduces the corresponding
# total distance by at most COLS/ROWS respectively.

function heuristic(s::TorusState, goal::TorusState)
  goal_pos = Vector{CartesianIndex{2}}(undef, ROWS * COLS)
  for r = 1:ROWS, c = 1:COLS
    goal_pos[Int(goal.grid[r, c])] = CartesianIndex(r, c)
  end
  total_h = 0
  total_v = 0
  for r = 1:ROWS, c = 1:COLS
    t = Int(s.grid[r, c])
    gpos = goal_pos[t]
    dr = abs(r - gpos[1])
    dc = abs(c - gpos[2])
    total_v += min(dr, ROWS - dr)
    total_h += min(dc, COLS - dc)
  end
  return cld(total_h, COLS) + cld(total_v, ROWS)
end

# ---- Random puzzle generation ----

function random_torus(n_moves::Int; seed::Union{Int, Nothing} = nothing)
  seed !== nothing && Random.seed!(seed)
  g = copy(GOAL_STATE.grid)
  all_ops = vcat(
    [(:row, i, d) for i = 1:ROWS for d in (-1, 1)],
    [(:col, j, d) for j = 1:COLS for d in (-1, 1)],
  )
  for _ = 1:n_moves
    (type, idx, dir) = rand(all_ops)
    if type == :row
      g[idx, :] = circshift(g[idx, :], dir)
    else
      g[:, idx] = circshift(g[:, idx], dir)
    end
  end
  return TorusState(g)
end

# ---- Move description (for solution printing) ----

function describe_move(s1::TorusState, s2::TorusState)
  for i = 1:ROWS
    if s1.grid[i, :] != s2.grid[i, :]
      return circshift(s1.grid[i, :], -1) == s2.grid[i, :] ? "Shift row $i LEFT" :
             "Shift row $i RIGHT"
    end
  end
  for j = 1:COLS
    if s1.grid[:, j] != s2.grid[:, j]
      return circshift(s1.grid[:, j], -1) == s2.grid[:, j] ? "Shift col $j UP" :
             "Shift col $j DOWN"
    end
  end
  return "(no change)"
end

# ---- Solver ----
#
# h_weight    (≥ 1.0): Weighted A* — inflates heuristic by this factor.
#             Guarantees solution cost ≤ h_weight × optimal.
#             h_weight = 1.0 → standard A* (optimal).
#
# cost_weight (≤ 1.0): Deflates per-step cost contribution.
#             At 0.0 → pure greedy best-first (no optimality guarantee).

function solve(
  start::TorusState;
  h_weight::Float64 = 1.0,
  cost_weight::Float64 = 1.0,
  timeout::Float64 = 30.0,
  verbose::Bool = true,
)
  h0 = heuristic(start, GOAL_STATE)
  if verbose
    mode = if h_weight == 1.0 && cost_weight == 1.0
      "standard A* (optimal)"
    elseif cost_weight == 0.0
      "greedy best-first (h only)"
    else
      "weighted: h×$(h_weight), g×$(cost_weight)"
    end
    println("[$mode]  h₀ = $h0")
    println(start)
  end

  weighted_heuristic = (s, g) -> h_weight * Float64(heuristic(s, g))
  weighted_cost = (_, _) -> cost_weight

  result = astar(
    nextstates,
    start,
    GOAL_STATE;
    heuristic = weighted_heuristic,
    cost = weighted_cost,
    timeout = timeout,
  )

  n_moves = length(result.path) - 1

  if result.status == :success
    if verbose
      println(
        "Solved in $n_moves move(s)  [scaled cost=$(round(result.cost, digits = 2))]  states explored: $(result.closedsetsize)\n",
      )
      for i = 2:length(result.path)
        move_desc = describe_move(result.path[i - 1], result.path[i])
        println("Move $(i-1): $move_desc")
        println(result.path[i])
      end
    end
  elseif result.status == :timeout
    verbose && println("Timeout after $(timeout)s — partial path: $n_moves moves")
  else
    verbose && println("No solution found: $(result.status)")
  end

  return result
end

if abspath(PROGRAM_FILE) == @__FILE__
  println("=" ^ 50)
  println("TEST 1: already at goal (expect 0 moves)")
  println("=" ^ 50)
  solve(copy(GOAL_STATE))

  println("=" ^ 50)
  println("TEST 2: example from problem (row 1 shifted right ×2, expect 2 moves)")
  println("=" ^ 50)
  example = TorusState(Int8.([
    3 4 1 2
    5 6 7 8
    9 10 11 12
    13 14 15 16
  ]))
  solve(example)

  println("=" ^ 50)
  println("TEST 3: random puzzle scrambled with 10 moves (seed=42)")
  println("=" ^ 50)
  solve(random_torus(10; seed = 42))

  println("=" ^ 50)
  println("TEST 4: random puzzle scrambled with 15 moves (seed=7)")
  println("=" ^ 50)
  solve(random_torus(15; seed = 7))

  println()
  println("=" ^ 50)
  println("SPEEDUP COMPARISON — 20-move scramble (seed=99)")
  println("=" ^ 50)
  hard = random_torus(20; seed = 99)

  for (label, hw, cw) in [
    ("standard A* (optimal)", 1.0, 1.0),
    ("WA*  h×2   (≤2× optimal)", 2.0, 1.0),
    ("WA*  h×5   (≤5× optimal)", 5.0, 1.0),
    ("reduced cost ×0.5", 1.0, 0.5),
    ("greedy best-first (cost=0)", 1.0, 0.0),
  ]
    print(rpad(label, 38))
    r = solve(hard; h_weight = hw, cost_weight = cw, verbose = false)
    n = length(r.path) - 1
    println("  moves=$n  explored=$(r.closedsetsize)  status=$(r.status)")
  end
end
