using DataStructures: PriorityQueue, enqueue!, dequeue!

# I want to expect that the state has the following functions defined:
# - neighbors(state) -> Vector{StateType}
# - is_goal(state, goal) -> Bool
# - cost(state, neighbor) -> Number
# - Base.hash(state) -> Int

function reconstruct_path(
    came_from::Dict{StateType, StateType},
    current_state::StateType,
) where {StateType}
    total_path = [current_state]
    while haskey(came_from, current_state)
        current_state = came_from[current_state]
        push!(total_path, current_state)
    end
    return reverse!(total_path)
end

function a_star(
    heuristic::Function,
    start_state::StateType,
    goal_state::StateType,
) where {StateType}
    start_heuristic = heuristic(start_state, goal_state)
    open_set = PriorityQueue(start_state => start_heuristic)
    came_from = Dict{StateType, StateType}()
    g_score = Dict{StateType, Number}(start_state => 0)
    f_score = Dict{StateType, Number}(start_state => start_heuristic)

    while !isempty(open_set)
        current_state = dequeue!(open_set)

        if is_goal(current_state, goal_state)
            return reconstruct_path(came_from, current_state)
        end

        for neighbor in neighbours(current_state)
            tentative_g_score = g_score[current_state] + cost(current_state, neighbor)

            if !haskey(g_score, neighbor) || tentative_g_score < g_score[neighbor]
                came_from[neighbor] = current_state
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_state)

                if !haskey(open_set, neighbor)
                    enqueue!(open_set, neighbor, f_score[neighbor])
                end
            end
        end
    end
end


# example

struct S
    x::Int
end

function neighbours(state::S)
    return [S(state.x - 1), S(state.x + 1)]
end

function is_goal(state::S, goal::S)
    return state.x == goal.x
end

function cost(state::S, neighbour_state::S)
    return 1
end

function heuristic(state::S, goal::S)
    return abs(goal.x - state.x)
end


start = S(0)
goal = S(10)

path = a_star(heuristic, start, goal)

for state in path
    println(state.x)
end
