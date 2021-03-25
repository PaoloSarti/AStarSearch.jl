
UP = CartesianIndex(-1, 0)
DOWN = CartesianIndex(1, 0)
LEFT = CartesianIndex(0, -1)
RIGHT = CartesianIndex(0, 1)
DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

manhattan(a::CartesianIndex, b::CartesianIndex) = sum(abs.((b - a).I))

"""
Define a Maze Solver as a subtype of AbstractAStarSearch and keep the maze itself as attribute of the solver
"""
struct MazeSolver <: AbstractAStarSearch{CartesianIndex{2}}
  maze:: BitArray{2}
end
neighbours(ms::MazeSolver, x::CartesianIndex{2}) = filter(x -> (1 <= x[1] <= size(ms.maze)[1]) && (1 <= x[2] <= size(ms.maze)[2]) && (!ms.maze[x]), [x + d for d in DIRECTIONS])
heuristic(ms::MazeSolver, x::CartesianIndex{2}, goal::CartesianIndex{2}) = manhattan(x, goal)
