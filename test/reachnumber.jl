@testset "ReachNumberGoal" begin
  start = 0
  goal = 10

  isgoal(state) = state == goal
  getneighbours(state) = [state - 1, state + 1]
  heuristic(state) = abs(goal - state)

  res = astar(start, isgoal, getneighbours, heuristic)
  @test res.status == :success
  @test res.path == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
  @test res.cost == 10
end

@testset "Timeout" begin
  start = 1
  goal = 3

  isgoal(state) = state == goal
  getneighbours(state) = [state + 2, state + 1]
  heuristic(state) = goal - state

  res = astar(start, isgoal, getneighbours, heuristic, timeout=0)
  @test res.status == :timeout
  @test res.path == [1]
  @test res.cost == 0
end

@testset "Multiple paths" begin
  start = 1
  goal = 4

  isgoal(state) = state == goal
  getneighbours(state) = [state + 2, state + 1, state - 1]
  heuristic(state) = abs(goal - state)

  res = astar(start, isgoal, getneighbours, heuristic)
  @test res.status == :success
  @test res.path == [1, 3, 4]
  @test res.cost == 2
end

@testset "Distance argument" begin
  start = 0
  goal = 10

  isgoal(state) = state == goal
  getneighbours(state) = [state - 1, state + 1]
  heuristic(state) = abs(goal - state)
  distance(a, b) = a % 2 == 0 ? abs(b - a) : 2 * abs(b - a)

  res = astar(start, isgoal, getneighbours, heuristic; distance)
  @test res.status == :success
  @test res.path == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
  @test res.cost == 15
end