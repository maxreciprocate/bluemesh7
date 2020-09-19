include("../src/BlueMesh7.jl")
using .BlueMesh7: generate_graph, initialize_mesh, start, plotgrid
using Random

minutes = 20

Random.seed!(7)
positions = generate_positions()

node_roles = zeros(Int, length(positions))
mesh = initialize_mesh(positions, node_roles)

Random.seed!(7)
produced, received = start(mesh; minutes)
println("None relays had $(round(received / produced; digits=3)) success of delivery")

node_roles = ones(Int, length(positions))
mesh = initialize_mesh(positions, node_roles)

Random.seed!(7)
produced, received = start(mesh; minutes)
println("All relays had $(round(received / produced; digits=3)) success of delivery")

node_roles = rand(0:1, length(positions))
mesh = initialize_mesh(positions, node_roles)

Random.seed!(7)
produced, received = start(mesh; minutes)
println("Half relays had $(round(received / produced; digits=3)) success of delivery")
