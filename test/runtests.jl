using Test

include("../src/a_star.jl")

@testset "AStar" begin
  include("reachnumber.jl")
  include("maze.jl")
  include("15puzzle.jl")
end
