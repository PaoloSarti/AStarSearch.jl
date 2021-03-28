
@testset "Maze Subtyping" begin

@testset "Maze goal reachable" begin
  maze = [0 0 1 0 0;
          0 1 0 0 0;
          0 1 0 0 1;
          0 0 0 1 1;
          1 0 1 0 0] .== 1
  
  mz = MazeSolver(maze)
  start = CartesianIndex(1, 1)
  goal = CartesianIndex(1, 5)

  res = search(mz, start, goal)
  @test res.status == :success
  @test res.path ==  CartesianIndex{2}[CartesianIndex(1, 1), CartesianIndex(2, 1), CartesianIndex(3, 1), CartesianIndex(4, 1), CartesianIndex(4, 2), CartesianIndex(4, 3), CartesianIndex(3, 3), CartesianIndex(2, 3), CartesianIndex(2, 4), CartesianIndex(1, 4), CartesianIndex(1, 5)]
  @test res.cost == 10
end

end
