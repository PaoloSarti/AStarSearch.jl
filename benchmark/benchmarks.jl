using Pkg
tempdir = mktempdir()
Pkg.activate(tempdir)
Pkg.develop(PackageSpec(path = joinpath(@__DIR__, "..")))
Pkg.add(["BenchmarkTools", "PkgBenchmark"])
Pkg.resolve()

using BenchmarkTools

const SUITE = BenchmarkGroup()

SUITE["collatz"] = include("collatz.jl")
