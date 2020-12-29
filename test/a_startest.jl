using Test

include("../src/a_star.jl")

@testset "ReachNumberGoal" begin
  start = 0
  goal = 10

  isgoal(state) = state == goal
  getneighbours(state) = [state - 1, state + 1]
  distance(previousstate, neighbourstate) = 1
  heuristic(state) = abs(goal - state)

  res = astar(start, isgoal, getneighbours, distance, heuristic, 1000)
  @test res.status == :success
  @test res.path == [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
  @test res.cost == 10
end

@testset "Timeout" begin
  start = 1
  goal = 3

  isgoal(state) = state == goal
  getneighbours(state) = [state + 2, state + 1]
  distance(previousstate, neighbourstate) = 1
  heuristic(state) = goal - state

  res = astar(start, isgoal, getneighbours, distance, heuristic, 0)
  @test res.status == :timeout
  @test res.path == [1]
  @test res.cost == 0
end

@testset "Multiple paths" begin
  start = 1
  goal = 4

  isgoal(state) = state == goal
  getneighbours(state) = [state + 2, state + 1, state - 1]
  distance(previousstate, neighbourstate) = 1
  heuristic(state) = abs(goal - state)

  res = astar(start, isgoal, getneighbours, distance, heuristic, 1000)
  @test res.status == :success
  @test res.path == [1, 3, 4]
  @test res.cost == 2
end

@testset "Maze" begin
  maze = [0 0 1 0 0;
          0 1 0 0 0;
          0 1 0 0 1;
          0 0 0 1 1;
          1 0 1 0 0] .== 1
  start = CartesianIndex(1, 1)
  goal = CartesianIndex(1, 5)

  directions = [CartesianIndex(1, 0), CartesianIndex(0, 1), CartesianIndex(-1, 0), CartesianIndex(0, -1)]

  isgoal(state) = state == goal
  getneighbours(state) = filter(x -> (1 <= x[1] <= size(maze)[1]) && (1 <= x[2] <= size(maze)[2]) && (!maze[x]), [state + d for d in directions])
  distance(previousstate, neighbourstate) = 1
  heuristic(state) = sum(abs.((goal - state).I))

  res = astar(start, isgoal, getneighbours, distance, heuristic, 1000)
  @test res.status == :success
  @test res.path == CartesianIndex{2}[CartesianIndex(1, 1), CartesianIndex(2, 1), CartesianIndex(3, 1), CartesianIndex(4, 1), CartesianIndex(4, 2), CartesianIndex(4, 3), CartesianIndex(3, 3), CartesianIndex(3, 4), CartesianIndex(2, 4), CartesianIndex(2, 5), CartesianIndex(1, 5)]
  @test res.cost == 10
end

@testset "Maze Unreachable" begin
  maze = [0 0 1 0 0;
          0 1 0 0 0;
          0 1 0 0 1;
          1 0 0 1 1;
          1 0 1 0 0] .== 1
  start = CartesianIndex(1, 1)
  goal = CartesianIndex(1, 5)

  directions = [CartesianIndex(1, 0), CartesianIndex(0, 1), CartesianIndex(-1, 0), CartesianIndex(0, -1)]

  isgoal(state) = state == goal
  getneighbours(state) = filter(x -> (1 <= x[1] <= size(maze)[1]) && (1 <= x[2] <= size(maze)[2]) && (!maze[x]), [state + d for d in directions])
  distance(previousstate, neighbourstate) = 1
  heuristic(state) = sum(abs.((goal - state).I))

  res = astar(start, isgoal, getneighbours, distance, heuristic, 1000)
  @test res.status == :nopath
  @test res.path == CartesianIndex{2}[CartesianIndex(1, 1), CartesianIndex(1, 2)]
  @test res.cost == 1
end
