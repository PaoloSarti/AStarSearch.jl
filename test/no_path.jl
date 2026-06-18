@testset "no_path" begin
  # Never any path
  no_path_neighbours(n) = Int64[]

  res = astar(no_path_neighbours, 1, 2)
  @test res.status == :nopath
  @test res.path == [1]

  for algorithm in [depthfirst, breadthfirst, iterative_deepening]
    res = algorithm(no_path_neighbours, 1, 2, maxdepth = 1)
    @test res.status == :nopath
    @test res.path == [1]
  end
end
