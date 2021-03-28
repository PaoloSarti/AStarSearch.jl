@testset "AbstractAStarSearch" begin
  @testset "neighbours throws if you don't define it for you type" begin
    nim = NotImplementedMethodsAstarSearch() # see definition in mazesolver_definition
    state = 0
    @test_throws String neighbours(nim, state)
  end

  @testset "heuristic throws if you don't define it for you type" begin
    nim = NotImplementedMethodsAstarSearch() # see definition in mazesolver_definition
    state = 0
    goal = 10
    @test_throws String heuristic(nim, state, goal)
  end

end