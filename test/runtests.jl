using Test

include("../src/a_star.jl")
#include here the MazeSolver definition as it cannot be defined inside a @testset
include("mazesolver_definition.jl")

@testset "AStar" begin
  include("reachnumber.jl")
  include("maze.jl")
  include("mazesolver.jl")
  include("15puzzle.jl")
  include("collatz.jl")
end
