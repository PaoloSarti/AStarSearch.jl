@testset "collatz" begin

  # A little puzzle inspired by the collatz conjecture, reach one by using the collatz conjecture moves (3n + 1 can be used even if the number is even)
  neighbours(n) = n % 2 == 0 ? [n ÷ 2, 3n + 1] : [3n + 1]

  # The best case is if the number is a power of 2, then the moves are just the log2, otherwise this is a very optimistic heuristic (it doesn't overestimate)
  heuristic(n, goal) = floor(Int64, log2(n))

  @testset "12" begin
    start = 12
    res = astar(neighbours, start, 1; heuristic)
    @test res.status == :success
    @test res.path == [12, 6, 3, 10, 5, 16, 8, 4, 2, 1]
    @test res.cost == 9
  end

  @testset "12 with maxcost" begin
    start = 12
    res = astar(neighbours, start, 1; heuristic, maxcost = 3)
    @test res.status == :nopath
    @test res.path == [12, 6, 3]
    @test res.cost == 2
  end

  @testset "12 with cost function" begin
    start = 12

    # dividing costs 1, multiplying costs 4
    cost(state, neighbour) = state / neighbour == 2 ? 1 : 4

    res = astar(neighbours, start, 1; heuristic, cost)
    @test res.status == :success
    @test res.path == [12, 6, 3, 10, 5, 16, 8, 4, 2, 1]
    @test res.cost == 15
  end

  @testset "maxdepth ignored" begin
    start = 12

    res = astar(neighbours, start, 1; heuristic, maxdepth = 0)
    @test res.status == :success
    @test res.path == [12, 6, 3, 10, 5, 16, 8, 4, 2, 1]
    @test res.cost == 9
  end

  @testset "depthfirst maxdepth" begin
    start = 12

    res = depthfirst(neighbours, start, 1; maxdepth = 0)
    @test res.status == :nopath
    @test res.path == [start]
    @test res.cost == 0
  end

  @testset "depthfirst timeout" begin
    start = 12

    res = depthfirst(neighbours, start, 1; timeout = 0)
    @test res.status == :timeout
    @test res.path == [start]
    @test res.cost == 0
  end


  @testset "depthfirst success" begin
    start = 12

    res = depthfirst(neighbours, start, 1)
    @test res.status == :success
    @test res.path == [12, 6, 3, 10, 5, 16, 8, 4, 2, 1]
    @test res.cost == 9
  end
end
