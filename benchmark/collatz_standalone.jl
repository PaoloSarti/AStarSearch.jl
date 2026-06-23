using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using AStarSearch
using Statistics
using Printf

neighbours(n) = n % 2 == 0 ? [n ÷ 2, 3n + 1] : [3n + 1]
heuristic(n, _) = floor(Int64, log2(n))

astar_collatz(x) = astar(neighbours, x, 1; heuristic)

const NUMBERS = [1, 34, 523, 1345, 8765]
const MIN_RUNS = 3
const BUDGET_S = 5.0

function bench(n)
    times = Float64[]
    deadline = time() + BUDGET_S
    while length(times) < MIN_RUNS || time() < deadline
        t0 = time_ns()
        astar_collatz(n)
        push!(times, time_ns() - t0)
    end
    return median(times), minimum(times)
end

# warm up
for n in NUMBERS
    astar_collatz(n)
end

@printf("%6s  %12s  %12s\n", "n", "median", "min")
println("-" ^ 36)
for n in NUMBERS
    med, mn = bench(n)
    @printf("%6d  %10.1fµs  %10.1fµs\n", n, med / 1e3, mn / 1e3)
end
