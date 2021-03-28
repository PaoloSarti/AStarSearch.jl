using Test

include("../src/a_star.jl")
#include here the AbstractAStarSearch subtypes definition as they cannot be defined inside a @testset
include("abstractastarsearch_subtypes_definition.jl")

@testset "AStar" begin
  include("reachnumber.jl")
  include("maze.jl")
  include("abstractastarsearch.jl")
  include("mazesolver.jl")
  include("15puzzle.jl")
  include("collatz.jl")
end
