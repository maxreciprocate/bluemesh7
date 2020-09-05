using BenchmarkTools
using Random

Random.seed!(2000)

include("../examples/run.jl")
a = @benchmark run()
display(a)
