@testset "ReachNumber" begin
  heuristic(state, goal) = abs(goal - state)

  @testset "ReachNumberGoalDefaultHeuristic" begin
    start = 0
    goal = 10

    getneighbours(state) = [state - 1, state + 1]

    res = astar(getneighbours, start, goal)
    @test res.status == :success
    @test res.path == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    @test res.cost == 10
  end

  @testset "ReachNumberGoal" begin
    start = 0
    goal = 10

    getneighbours(state) = [state - 1, state + 1]

    res = astar(getneighbours, start, goal; heuristic)
    @test res.status == :success
    @test res.path == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    @test res.cost == 10
  end

  @testset "ReachNumberGoal Negative" begin
    start = 10
    goal = 0

    getneighbours(state) = [state - 1, state + 1]

    res = astar(getneighbours, start, goal; heuristic)
    @test res.status == :success
    @test res.path == [10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
    @test res.cost == 10
  end

  @testset "Timeout" begin
    start = 1
    goal = 3

    getneighbours(state) = [state + 2, state + 1]

    res = astar(getneighbours, start, goal; heuristic, timeout = -1)# timeout immediately
    @test res.status == :timeout
    @test res.path == [1]
    @test res.cost == 0
  end

  @testset "Multiple paths" begin
    start = 1
    goal = 4

    getneighbours(state) = [state + 2, state + 1, state - 1]

    res = astar(getneighbours, start, goal; heuristic)
    @test res.status == :success
    @test res.path == [1, 3, 4]
    @test res.cost == 2
  end

  @testset "Cost argument" begin
    start = 0
    goal = 10

    getneighbours(state) = [state - 1, state + 1]
    cost(a, b) = a % 2 == 0 ? abs(b - a) : 2 * abs(b - a)

    res = astar(getneighbours, start, goal; heuristic, cost)
    @test res.status == :success
    @test res.path == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    @test res.cost == 15
  end

  @testset "Multiple paths, a double step costs 3, exercise the new path" begin
    start = 1
    goal = 4

    getneighbours(state) = [state + 2, state + 1, state - 1]
    cost(a, b) = b - a == 2 ? 3 : 1

    res = astar(getneighbours, start, goal; heuristic, cost)
    @test res.status == :success
    @test res.path == [1, 2, 3, 4]
    @test res.cost == 3
  end
end
