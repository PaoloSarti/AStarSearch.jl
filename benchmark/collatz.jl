module BenchmarkAStarSearchCollatz

using BenchmarkTools
using AStarSearch

neighbours(n) = n % 2 == 0 ? [n ÷ 2, 3n + 1] : [3n + 1]

# The best case is if the number is a power of 2, then the moves are just the log2, otherwise this is a very optimistic heuristic (it doesn't overestimate)
heuristic(n, goal) = floor(Int64, log2(n))

astar_collatz(x; kwargs...) = astar(neighbours, x, 1; heuristic, kwargs...)

suite = BenchmarkGroup()

numbers = [1, 34, 523, 1345, 8765]
for number in numbers
  suite[[string(number)]] = @benchmarkable astar_collatz($number)
end

end

BenchmarkAStarSearchCollatz.suite
