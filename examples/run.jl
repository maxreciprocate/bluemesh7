include("../src/BlueMesh7.jl")
using .BlueMesh7: generate_graph, initialize_mesh, start, plotgraph

function run()
    positions, adjacency = generate_graph()

    node_roles = rand(0:1, length(positions))
    mesh = initialize_mesh(positions, node_roles)

    println("running the simulation for 1 minute (60000 steps, 1 step = 1ms)...")
    produced, received = start(mesh, minutes = 20)
    println("packets produced: $produced, received: $received, $(round(100 * received / produced; digits=2))%")

    plotgraph(mesh)
end

run()
