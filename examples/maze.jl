using Random
using GLMakie

using AStarSearch

const UP = CartesianIndex(-1, 0)
const DOWN = CartesianIndex(1, 0)
const LEFT = CartesianIndex(0, -1)
const RIGHT = CartesianIndex(0, 1)
const DIRECTIONS = [UP, DOWN, LEFT, RIGHT]

function mazeneighbours(maze, p)
  res = CartesianIndex[]
  for d in DIRECTIONS
    n = p + d
    if 1 ≤ n[1] ≤ size(maze)[1] && 1 ≤ n[2] ≤ size(maze)[2] && !maze[n]
      push!(res, n)
    end
  end
  return res
end

manhattan(p, s) = abs(p[1] - s[1]) + abs(p[2] - s[2])

solvemaze(m, s, g) = astar(p -> mazeneighbours(m, p), s, g, heuristic = manhattan)

function random_maze(rows::Int = 30, cols::Int = 30)
  # visited map
  vis = fill(false, rows, cols)

  # walls, true = wall, false = passage
  maze = trues(rows * 2 + 1, cols * 2 + 1)

  function walk(p)
    vis[p] = true
    # Carve the current cell
    maze[p * 2] = false

    next_cells_list = mazeneighbours(vis, p)
    shuffle!(next_cells_list)

    for np in next_cells_list
      if vis[np]
        continue
      end
      # Carve passage between (x, y) and (xx, yy)
      maze[np * 2] = false
      # Carve the wall between current cell and next cell
      maze[p * 2 + (np - p)] = false
      walk(np)
    end
  end

  # start strictly inside the maze
  walk(CartesianIndex(rand(1:rows), rand(1:cols)))
  return maze
end

function draw_maze(maze::BitMatrix, path::Vector{CartesianIndex{2}} = CartesianIndex{2}[])
  h, w = size(maze)
  result = ""
  path_set = Set(path)

  for i = 1:h
    for j = 1:w
      if maze[i, j]  # wall
        if i % 2 == 1 && j % 2 == 1
          result *= "+"
        elseif i % 2 == 1
          result *= "-"
        else
          result *= "|"
        end
      else  # passage
        if CartesianIndex(i, j) in path_set
          result *= "*"
        else
          result *= " "
        end
      end
    end
    result *= "\n"
  end

  return result
end

maze_goal(maze) = CartesianIndex(size(maze, 1) - 1, size(maze, 2) - 1)

function maze_with_path(maze::BitMatrix, path::Vector{CartesianIndex{2}})
  int_maze = Int8.(maze)
  for p in path
    int_maze[p] = 2
  end
  return int_maze
end

function solve_maze(rows::Int = 30, cols::Int = 30)
  maze = random_maze(rows, cols)

  start = CartesianIndex(2, 2)
  goal = maze_goal(maze)

  res = solvemaze(maze, start, goal)
  return println(draw_maze(maze, res.path))
end

function graphic_maze_solution(
  rows::Int = 100,
  cols::Int = 100,
  filename::String = "solved_maze.png",
)
  maze = random_maze(rows, cols)

  start = CartesianIndex(2, 2)
  goal = maze_goal(maze)

  res = solvemaze(maze, start, goal)

  int_maze = maze_with_path(maze, res.path)

  fig = Figure()
  ax =
    Axis(fig[1, 1]; title = "Maze Solution", xticksvisible = false, yticksvisible = false)
  heatmap!(ax, int_maze'; colormap = :winter, interpolate = false)
  fig

  return save(filename, fig)
end

if abspath(PROGRAM_FILE) == @__FILE__
  # solve_maze(20, 20)
  graphic_maze_solution(100, 100, "solved_maze.png")
end
