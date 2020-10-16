include("../src/BlueMesh7.jl")

using Plots
using Statistics: mean
using ProgressMeter: @showprogress
using BSON: @save

ngraphs = 4
graphs = [generate_positions(dims=(30, 30), n=96) for _ = 1:ngraphs]
# ■

emit_rates = [1, collect(5:10:105)...] |> reverse
setup_nrelays = collect(0:16:96) |> reverse

# ■
pdrs = []

@showprogress for emit_rate in emit_rates
    for nrelays in setup_nrelays
        localpdrs = []

        Threads.@threads for graph in graphs
            node_roles = zeros(length(graph))

            for idx = 1:nrelays
                node_roles[idx] = 1
            end

            mesh = initialize_mesh(graph, Random.shuffle(node_roles))
            mesh.packet_emit_rate = emit_rate / 1000

            push!(localpdrs, start(mesh, minutes=1).pdr)
        end

        push!(pdrs, mean(localpdrs))
    end
end

@save "surface.bson" pdrs
# ■

p = plot(reverse(setup_nrelays), reverse(emit_rates), pdrs, st = :surface, camera=(60, 40),
         xticks=([reverse(setup_nrelays);], setup_nrelays), yticks=([reverse(emit_rates);], emit_rates),
         xlabel="relays", ylabel="emit", zlabel="pdr")

savefig("resurfaced-more.png")
