@testset "Torus Puzzle" begin
  import Base
  using Random

  TROWS = 4
  TCOLS = 4

  struct TorusState
    grid::Matrix{Int8}
  end

  Base.copy(s::TorusState) = TorusState(copy(s.grid))
  Base.:(==)(a::TorusState, b::TorusState) = a.grid == b.grid
  Base.hash(s::TorusState) = hash(s.grid)

  TGOAL = TorusState(Matrix{Int8}(reshape(1:(TROWS * TCOLS), TCOLS, TROWS)'))

  function torus_nextstates(s::TorusState)
    states = Vector{TorusState}(undef, 2 * (TROWS + TCOLS))
    k = 1
    for i = 1:TROWS
      g = copy(s.grid)
      g[i, :] = circshift(g[i, :], -1)
      states[k] = TorusState(g)
      k += 1
      g = copy(s.grid)
      g[i, :] = circshift(g[i, :], 1)
      states[k] = TorusState(g)
      k += 1
    end
    for j = 1:TCOLS
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

  function torus_heuristic(s::TorusState, goal::TorusState)
    goal_pos = Vector{CartesianIndex{2}}(undef, TROWS * TCOLS)
    for r = 1:TROWS, c = 1:TCOLS
      goal_pos[Int(goal.grid[r, c])] = CartesianIndex(r, c)
    end
    total_h = 0
    total_v = 0
    for r = 1:TROWS, c = 1:TCOLS
      t = Int(s.grid[r, c])
      gpos = goal_pos[t]
      dr = abs(r - gpos[1])
      dc = abs(c - gpos[2])
      total_v += min(dr, TROWS - dr)
      total_h += min(dc, TCOLS - dc)
    end
    return cld(total_h, TCOLS) + cld(total_v, TROWS)
  end

  function random_torus(n_moves::Int; seed::Union{Int, Nothing} = nothing)
    seed !== nothing && Random.seed!(seed)
    g = copy(TGOAL.grid)
    all_ops = vcat(
      [(:row, i, d) for i = 1:TROWS for d in (-1, 1)],
      [(:col, j, d) for j = 1:TCOLS for d in (-1, 1)],
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

  @testset "Already at goal" begin
    res = astar(torus_nextstates, copy(TGOAL), TGOAL; heuristic = torus_heuristic)
    @test res.status == :success
    @test res.cost == 0
    @test length(res.path) == 1
  end

  @testset "2-move example (row 1 shifted right ×2)" begin
    example = TorusState(Int8.([
      3 4 1 2
      5 6 7 8
      9 10 11 12
      13 14 15 16
    ]))
    res = astar(torus_nextstates, example, TGOAL; heuristic = torus_heuristic)
    @test res.status == :success
    @test res.cost == 2
  end

  @testset "10-move scramble (seed=42)" begin
    start = random_torus(10; seed = 42)
    res = astar(torus_nextstates, start, TGOAL; heuristic = torus_heuristic, timeout = 60.0)
    @test res.status == :success
  end
end
