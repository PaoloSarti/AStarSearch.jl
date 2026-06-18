# Torus Puzzle Example

The **Torus Puzzle** is a sliding-tile puzzle played on an N×M grid containing tiles 1…N·M.
Unlike the classic 15-puzzle there is **no empty cell** — every cell is occupied.
Rows and columns wrap around (cyclic shifts), so the grid behaves like the surface of a torus.

The complete, runnable code can be found in [`examples/torus.jl`](https://github.com/PaoloSarti/AStarSearch.jl/blob/main/examples/torus.jl).

## Problem Setup

### Goal State

Tile `t` belongs at row `⌈t/M⌉`, column `((t-1) mod M) + 1`.
For a 4×4 grid:

```
 1  2  3  4
 5  6  7  8
 9 10 11 12
13 14 15 16
```

### Valid Moves

There are `2N + 2M` moves from any state (16 for a 4×4 grid):

| Move              | Effect                        |
|:----------------- |:----------------------------- |
| Shift row i LEFT  | `[a, b, c, d] → [b, c, d, a]` |
| Shift row i RIGHT | `[a, b, c, d] → [d, a, b, c]` |
| Shift col j UP    | top element wraps to bottom   |
| Shift col j DOWN  | bottom element wraps to top   |

### State Representation

```julia
struct TorusState
  grid::Matrix{Int8}
end

Base.copy(s::TorusState) = TorusState(copy(s.grid))
Base.:(==)(a::TorusState, b::TorusState) = a.grid == b.grid
Base.hash(s::TorusState) = hash(s.grid)

const ROWS = 4
const COLS = 4

const GOAL_STATE = TorusState(Matrix{Int8}(reshape(1:(ROWS * COLS), COLS, ROWS)'))
const GOAL_POS =
  [CartesianIndex(div(t - 1, COLS) + 1, mod(t - 1, COLS) + 1) for t = 1:(ROWS * COLS)]
```

## Successor Generation

Each call generates all `2·ROWS + 2·COLS` successor states via `circshift`:

```julia
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
```

## Heuristic Function

### Key Observation

Row shifts and column shifts address **independent dimensions**:

  - A **row shift** moves COLS tiles one step horizontally.
  - A **col shift** moves ROWS tiles one step vertically.

### Derivation

For each tile `t` at position `(r, c)` with goal position `(gr, gc)`:

```
dv(t) = min(|r - gr|, ROWS - |r - gr|)   # toroidal row distance
dh(t) = min(|c - gc|, COLS - |c - gc|)   # toroidal col distance
```

One row shift reduces the total horizontal distance by at most COLS, so:

```
h(s) = ⌈Σ dh(t) / COLS⌉  +  ⌈Σ dv(t) / ROWS⌉
```

This heuristic is **admissible** (never overestimates) and **consistent** (monotone).

```julia
function heuristic(s::TorusState, ::TorusState)
  total_h = 0
  total_v = 0
  for r = 1:ROWS, c = 1:COLS
    t = Int(s.grid[r, c])
    gpos = GOAL_POS[t]
    dr = abs(r - gpos[1])
    dc = abs(c - gpos[2])
    total_v += min(dr, ROWS - dr)
    total_h += min(dc, COLS - dc)
  end
  return cld(total_h, COLS) + cld(total_v, ROWS)
end
```

## Solving the Puzzle

```julia
using AStarSearch

result = astar(nextstates, start, GOAL_STATE; heuristic = heuristic)

if result.status == :success
  println("Solved in $(length(result.path) - 1) moves!")
else
  println("No solution found: $(result.status)")
end
```

## Speedup via Weighted A\*

For harder puzzles (20+ moves scrambled), standard A\* may be slow.
AStarSearch.jl lets you inflate the heuristic to trade optimality for speed:

```julia
# 2× faster, solution at most 2× optimal
result = astar(
  nextstates,
  start,
  GOAL_STATE;
  heuristic = (s, g) -> 2.0 * Float64(heuristic(s, g)),
  cost = (_, _) -> 1.0,
)
```

You can also deflate the step cost toward `0.0` for pure greedy best-first search,
which is the fastest but gives no optimality guarantee.

| `h_weight` | `cost_weight` | Behaviour                         |
|:---------- |:------------- |:--------------------------------- |
| 1.0        | 1.0           | Standard A\* — optimal            |
| w > 1      | 1.0           | Weighted A\* — at most w× optimal |
| 1.0        | 0 < c < 1     | Greedy-leaning, no formal bound   |
| any        | 0.0           | Pure greedy best-first — fastest  |

## Random Puzzle Generation

```julia
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

# 10-move scramble, reproducible
start = random_torus(10; seed = 42)
result = astar(nextstates, start, GOAL_STATE; heuristic = heuristic)
```
