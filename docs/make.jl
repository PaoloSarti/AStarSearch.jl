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

# We're using GitHub Actions for deployment, so we don't need deploydocs
# The documentation will be deployed by the upload-pages-artifact action