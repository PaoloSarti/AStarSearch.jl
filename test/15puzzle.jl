@testset "15 Puzzle" begin

import Base
using DataStructures

struct State
  table::Array{Int8,2}
end

Base.copy(s::State) = State(copy(s.table))
Base.:(==)(s1::State, s2::State) = s1.table == s2.table

function Base.hash(s::State)
  # the entire board state can be compressed in a single 64 bit UInt
  h = UInt64(0)
  # by column
  for j in 1:4, i in 1:4
    h += s.table[i, j]
    h <<= 4
  end
  return h
end

GOALSTATE = State([
  1  2  3  4;
  5  6  7  8;
  9 10 11 12;
  13 14 15 0
])

UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)

DIRECTIONS = [UP, DOWN, LEFT, RIGHT]
FINALSTATEINDEXES = [findfirst(x -> x == i, GOALSTATE.table) for i in 1:15]

emptyposition(s::State) = findfirst(x -> x == 0, s.table)

isvalidindex(c) = (0 < c[1] < 5) && (0 < c[2] < 5)

availablemoves(e::CartesianIndex) = [direction for direction in DIRECTIONS if isvalidindex(e + direction)]
availablemoves(s::State) = s |> emptyposition |> availablemoves
nextstates(s::State) = map(x -> move(s, x), availablemoves(s))

function move(s::State, direction::CartesianIndex)
  next = copy(s.table)
  cur = emptyposition(s)
  emptydest = cur + direction
  next[cur], next[emptydest] = next[emptydest], next[cur]
  return State(next)
end

function move!(s::State, direction::CartesianIndex)
  cur = emptyposition(s)
  emptydest = cur + direction
  s.table[cur], s.table[emptydest] = s.table[emptydest], s.table[cur]
  return nothing
end

function randominitialstate(n)
  s = copy(GOALSTATE)
  for _ in 1:n
    curavailablemoves = availablemoves(s)
    selectedmove = rand(curavailablemoves)
    move!(s, selectedmove)
  end
  return s
end

manhattan(c1::CartesianIndex, c2::CartesianIndex) = c1 - c2 |> x -> x.I .|> abs |> sum

heuristicmanhattan(s::State) = sum(manhattan(findfirst(x -> x == i, s.table), FINALSTATEINDEXES[i]) for i in 1:15)

function countswaps(s::State)
  acc = 0
  for j in 1:3, i in 1:3
    # horizontal swap
    if (s.table[i, j] == GOALSTATE.table[i + 1, j]) && (s.table[i + 1, j] == GOALSTATE.table[i, j])
      acc += 1
    end
    #vertical swap
    if (s.table[i, j] == GOALSTATE.table[i, j + 1]) && (s.table[i, j + 1] == GOALSTATE.table[i, j])
      acc += 1
    end
  end
  return acc
end

function heuristicmanhattanandswaps(s::State)
  hm = heuristicmanhattan(s)
  swaps = countswaps(s)
  return hm + 2swaps
end

# the goal is fixed in this problem
heuristic(s, goal) = heuristicmanhattanandswaps(s)

@testset "Test goal" begin
  res = astar(nextstates, GOALSTATE, GOALSTATE, heuristic=heuristic)
  @test res.status == :success
  @test map(x -> x.table, res.path) == [
    Int8[1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 15 0]
  ]
  @test res.cost == 0
end

@testset "Test state 1" begin
  start = State([
    1   2   3   4;
    5   7  11   8;
    9   6   0  12;
   13  10  14  15
  ])

  res = astar(nextstates, start, GOALSTATE, heuristic=heuristic)
  @test res.status == :success
  @test map(x -> x.table, res.path) == [
    Int8[1 2 3 4; 5 7 11 8; 9 6 0 12; 13 10 14 15],
    Int8[1 2 3 4; 5 7 0 8; 9 6 11 12; 13 10 14 15],
    Int8[1 2 3 4; 5 0 7 8; 9 6 11 12; 13 10 14 15],
    Int8[1 2 3 4; 5 6 7 8; 9 0 11 12; 13 10 14 15],
    Int8[1 2 3 4; 5 6 7 8; 9 10 11 12; 13 0 14 15],
    Int8[1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 0 15],
    Int8[1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 15 0]
  ]
  @test res.cost == 6
end

@testset "Test maxcost argument" begin
  start = State([
    1   2   3   4;
    5   7  11   8;
    9   6   0  12;
   13  10  14  15
  ])

  res = astar(nextstates, start, GOALSTATE, heuristic=heuristic, maxcost=5)
  @test res.status == :nopath
  @test map(x -> x.table, res.path) == [
    Int8[1 2 3 4; 5 7 11 8; 9 6 0 12; 13 10 14 15],
    Int8[1 2 3 4; 5 7 0 8; 9 6 11 12; 13 10 14 15],
    Int8[1 2 3 4; 5 0 7 8; 9 6 11 12; 13 10 14 15],
    Int8[1 2 3 4; 5 6 7 8; 9 0 11 12; 13 10 14 15],
    Int8[1 2 3 4; 5 6 7 8; 9 10 11 12; 13 0 14 15],
    Int8[1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 0 15]
  ]
  @test res.cost == 5
end

@testset "Test randominitialstate" begin
  start = randominitialstate(3)
  res = astar(nextstates, start, GOALSTATE, heuristic=heuristic, maxcost=5)
  @test res.status == :success
end

end
