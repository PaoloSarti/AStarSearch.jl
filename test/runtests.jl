using Test
using AStarSearch
import Aqua
import JuliaFormatter

@testset "AStar" begin
  @test JuliaFormatter.format(
    ".",
    indent = 2,
    whitespace_typedefs = true,
    whitespace_ops_in_indices = true,
    remove_extra_newlines = true,
    always_use_return = true,
    whitespace_in_kwargs = true,
    trailing_comma = true,
    format_markdown = true,
  )

  Aqua.test_ambiguities(AStarSearch)
  Aqua.test_project_toml_formatting(AStarSearch)
  Aqua.test_undefined_exports(AStarSearch)
  Aqua.test_unbound_args(AStarSearch)
  Aqua.test_deps_compat(AStarSearch)

  include("reachnumber.jl")
  include("maze.jl")
  include("15puzzle.jl")
  include("collatz.jl")
  include("no_path.jl")
end
