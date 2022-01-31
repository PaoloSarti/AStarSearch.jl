
@testset "Maze" begin

UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

function mazeneighbours(maze, p)
  res = CartesianIndex[]
  for d in DIRECTIONS
      n = p + d
      if 1 ≤ n[1] ≤ size(maze)[1] && 1 ≤ n[2] ≤ size(maze)[2] && !maze[n]
          push!(res, n)
      end
  end
  return res
end

manhattan(p, s) = abs(p[1] - s[1]) + abs(p[2] - s[2])

solvemaze(m, s, g) = astar(p -> mazeneighbours(m, p), s, g, heuristic=manhattan)

@testset "Maze goal reachable" begin
  maze = [0 0 1 0 0;
          0 1 0 0 0;
          0 1 0 0 1;
          0 0 0 1 1;
          1 0 1 0 0] .== 1
  start = CartesianIndex(1, 1)
  goal = CartesianIndex(1, 5)

  res = solvemaze(maze, start, goal)
  @test res.status == :success
  @test res.path ==  CartesianIndex{2}[CartesianIndex(1, 1), CartesianIndex(2, 1), CartesianIndex(3, 1), CartesianIndex(4, 1), CartesianIndex(4, 2), CartesianIndex(4, 3), CartesianIndex(3, 3), CartesianIndex(2, 3), CartesianIndex(2, 4), CartesianIndex(1, 4), CartesianIndex(1, 5)]
  @test res.cost == 10
end

@testset "Maze Goal Unreachable" begin
  maze = [0 0 1 0 0;
          0 1 0 0 0;
          0 1 0 0 1;
          1 0 0 1 1;
          1 0 1 0 0] .== 1
  start = CartesianIndex(1, 1)
  goal = CartesianIndex(1, 5)

  res = solvemaze(maze, start, goal)
  @test res.status == :nopath
  @test res.path == CartesianIndex{2}[CartesianIndex(1, 1), CartesianIndex(1, 2)]
  @test res.cost == 1
end

end
