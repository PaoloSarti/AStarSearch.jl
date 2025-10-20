using Documenter

push!(LOAD_PATH, "../src/")
using AStarSearch

makedocs(
    sitename = "AStarSearch.jl",
    format = Documenter.HTML(),
    modules = [AStarSearch],
    pages = [
        "Home" => "index.md",
        "API Reference" => "api.md",
        "Examples" => [
            "15-Puzzle" => "examples/15puzzle.md",
            "Maze Pathfinding" => "examples/maze.md"
        ]
    ]
)

# Documenter can also automatically deploy documentation to gh-pages.
# See https://juliadocs.github.io/Documenter.jl/stable/man/hosting/#GitHub-Pages
deploydocs(
    repo = "github.com/PaoloSarti/AStarSearch.jl",
    devbranch = "main",
    target = "build",
    push_preview = true
)