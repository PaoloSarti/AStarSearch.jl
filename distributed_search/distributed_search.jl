using Redis

conn = RedisConnection()

function neighbours(state)
    return [state + 1, state + 2]
end

function heuristic(state, goal)
    return abs(goal - state)
end

function cost(s1, s2)
    return 1
end

function parse_state(state_str)
    return parse(Int, state_str)
end

function serialize_state(state)
    return string(state)
end

function greedy_worker(
    conn::RedisConnection,
    parse_state::Function,
    serialize_state::Function,
    neighbours::Function,
    heuristic::Function,
    goal,
    start_list_key::String,
    openset_key::String)
    while true
        start_signal = blpop(conn, start_list_key, 60)  # Wait for a start signal with timeout
        if start_signal === nothing
            println("No start signal received yet.")
        else
            println("Received start signal: ", start_signal)
            break
        end
    end

    while true
        node = zpopmin(conn, openset_key)
        if node === nothing
            println("No more nodes to process. Exiting.")
            break
        end

        state = parse_state(node[1])
        println("Processing state: ", state)

        if state == goal
            println("Goal reached: ", state)
            break
        end

        for neighbour in neighbours(state)
            h = heuristic(neighbour, goal)
            zadd(conn, openset_key, h, serialize_state(neighbour))
            println("Added neighbour: ", neighbour, " with heuristic: ", h)
        end
    end
end

start = 0
goal = 10000000
task = @async greedy_worker(
    conn,
    parse_state,
    serialize_state,
    neighbours,
    heuristic,
    goal,
    "start_signals",
    "openset"
)
zadd(conn, "openset", heuristic(start, goal), serialize_state(start))
wait(task)
