using Test

include("../src/common.jl")
include("../src/uninformed_search.jl")
include("../src/a_star.jl")

@testset "AStar" begin
  include("reachnumber.jl")
  include("maze.jl")
  include("15puzzle.jl")
  include("collatz.jl")
  include("no_path.jl")
end
