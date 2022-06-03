@testset "no_path" begin
  # Never any path
  no_path_neighbours(n) = Int64[]

  for algorithm in [astar, depthfirst, breadthfirst, iterative_deepening]
    res = algorithm(no_path_neighbours, 1, 2, maxdepth = 1)
    @test res.status == :nopath
    @test res.path == [1]
  end
end
