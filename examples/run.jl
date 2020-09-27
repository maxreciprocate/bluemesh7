include("../src/BlueMesh7.jl")
using .BlueMesh7

function run()
    positions = generate_positions(dims=(20, 50), n=64)
    node_roles = rand(0:1, length(positions))
    mesh = initialize_mesh(positions, node_roles)

    println("Running a simulation for 7 minutes (420000 steps, 1 step = 1ms)...")
    stats = start(mesh, minutes = 7)

    PDR, worstPDR, delay = round.(collect(stats); digits=2)
    println("Packet delivery rate: $PDR, Worst delivery rate: $worstPDR, Average delay: $(delay)ms")
end

run()
