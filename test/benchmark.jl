using BenchmarkTools
using Random

include("../examples/run.jl")

Random.seed!(7)
a = @benchmark run()
display(a)
