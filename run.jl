include("BlueMesh7.jl")
using .BlueMesh7

function run()
    adjacency_matrix, positions = random_graph()

    node_roles = ones(Int, length(positions))
    model = initialize_model(node_roles, positions)

    println("running the simulation for 12 minutes (1 step = 1ms)...")
    produced, received = start_model(model, 12 * 60_000)
    println("packets produced: $produced, received: $received, $(round(100 * received / produced; digits=2))%")

    plotgraph(model)
end

run()
