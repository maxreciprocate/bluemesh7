include("../src/BlueMesh7.jl")
using .BlueMesh7

function run()
    positions = generate_positions(dims=(20, 20), n=64)
    node_roles = ones(Int, length(positions))
    mesh = initialize_mesh(positions, node_roles)

    println("Running a simulation for 7 minutes (420000 steps, 1 step = 1ms)...")
    stats = start(mesh, minutes = 1)

    stats = round.(collect(stats); digits=2)
    println("Packet delivery rate: $(stats.pdr), Worst delivery rate: $(stats.worstpdr), Average delay: $(stats.delay)ms, Centrality: $(stats.centrality), Average packet loss: $(stats.packetloss)")
end

run()
