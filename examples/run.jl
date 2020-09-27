include("../src/BlueMesh7.jl")
using .BlueMesh7: generate_positions, initialize_mesh, start, plotgrid

function run()
    positions = generate_positions()

    node_roles = rand(0:1, length(positions))
    mesh = initialize_mesh(positions, node_roles)

    println("running the simulation for 1 minute (60000 steps, 1 step = 1ms)...")
    produced, received = start(mesh, minutes = 1)
    println("packets produced: $produced, received: $received, $(round(100 * received / produced; digits=2))%")

    plotgrid(mesh)
end

run()
