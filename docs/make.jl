using Documenter

push!(LOAD_PATH, "../src/")
using AStarSearch

makedocs(
    sitename = "AStarSearch.jl",
    format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true",
        canonical = "https://paolosarti.github.io/AStarSearch.jl",
        edit_link = "main"
    ),
    modules = [AStarSearch],
    authors = "Paolo Sarti and contributors",
    repo = "https://github.com/PaoloSarti/AStarSearch.jl/blob/{commit}{path}#L{line}",
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
    repo = "github.com/PaoloSarti/AStarSearch.jl",
    devbranch = "main",
    versions = ["stable" => "v^", "v#.#", "dev" => "dev"]
)