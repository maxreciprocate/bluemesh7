include("../src/BlueMesh7.jl")
using .BlueMesh7: generate_graph, initialize_mesh, start, plotgraph

function run()
    adjacency_matrix, positions = generate_graph()

    node_roles = ones(Int, length(positions))
    mesh = initialize_mesh(positions, node_roles)

    println("running the simulation for 12 minutes (1 step = 1ms)...")
    produced, received = start(mesh, minutes = 12)
    println("packets produced: $produced, received: $received, $(round(100 * received / produced; digits=2))%")

    plotgraph(mesh)
end

run()
