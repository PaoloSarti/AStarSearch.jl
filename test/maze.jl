
@testset "Maze" begin

UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

manhattan(a::CartesianIndex, b::CartesianIndex) = sum(abs.((b - a).I))
getmazeneighbours(maze, state) = filter(x -> (1 <= x[1] <= size(maze)[1]) && (1 <= x[2] <= size(maze)[2]) && (!maze[x]), [state + d for d in DIRECTIONS])

@testset "Maze goal reachable" begin
  maze = [0 0 1 0 0;
          0 1 0 0 0;
          0 1 0 0 1;
          0 0 0 1 1;
          1 0 1 0 0] .== 1
  start = CartesianIndex(1, 1)
  goal = CartesianIndex(1, 5)

  neighbours(state) = getmazeneighbours(maze, state)

  res = astar(neighbours, start, goal, heuristic=manhattan)
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

  neighbours(state) = getmazeneighbours(maze, state)

  res = astar(neighbours, start, goal, heuristic=manhattan)
  @test res.status == :nopath
  @test res.path == CartesianIndex{2}[CartesianIndex(1, 1), CartesianIndex(1, 2)]
  @test res.cost == 1
end

end
