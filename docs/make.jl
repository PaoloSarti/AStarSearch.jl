using Documenter
using Documenter.Remotes: GitHub

push!(LOAD_PATH, "../src/")
using AStarSearch

makedocs(
    sitename = "AStarSearch.jl",
    warnonly = true,
    format = Documenter.HTML(),
    modules = [AStarSearch],
    authors = "Paolo Sarti and contributors",
    repo = GitHub("PaoloSarti", "AStarSearch.jl"),
    pages = [
        "Home" => "index.md",
        "API Reference" => "api.md",
        "Examples" => [
            "15-Puzzle" => "examples/15puzzle.md",
            "Maze Pathfinding" => "examples/maze.md"
        ]
    ]
)

deploydocs(
    repo = "github.com/PaoloSarti/AStarSearch.jl"
)