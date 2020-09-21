include("../src/BlueMesh7.jl")
using .BlueMesh7
using Random
using DataFrames
using ProgressMeter: @showprogress
using BSON: @load, @save
# ■

minutes = 5
ngraphs = 5

n_nodes = [32, 64, 128]
dims = [(100, 100), (121, 121), (144, 144)]

setup = [(n, d) for n in n_nodes, d in dims] |> vec
results = DataFrame()

Random.seed!(7)
allgraphs = [generate_graph(; n_nodes = n, dims = d) for _ = 1:ngraphs, (n, d) in setup]

function bench(; init, unroll)
    worst_rates = []

    @showprogress for graphs in eachcol(allgraphs)
        worst_rate = 1.0

        for graph in graphs
            mesh = init(graph)

            Random.seed!(7)
            produced, received = unroll(mesh, minutes)

            rate = received / produced

            if rate < worst_rate
                worst_rate = rate
            end
        end

        push!(worst_rates, worst_rate)
    end

    worst_rates
end

results.None = bench(
    init = (graph) -> initialize_mesh(first(graph), zeros(Int, length(first(graph)))),
    unroll = (mesh, minutes) -> start(mesh; minutes)
)

results.All = bench(
    init = (graph) -> initialize_mesh(first(graph), ones(Int, length(first(graph)))),
    unroll = (mesh, minutes) -> start(mesh; minutes)
)

results.Half = bench(
    init = (graph) -> initialize_mesh(first(graph), rand(0:1, length(first(graph)))),
    unroll = (mesh, minutes) -> start(mesh; minutes)
)

@load "Q-128.bson" Q

function unroll_pomdp(env, minutes)
    na = length(env.positions)
    moves = zeros(Int, na)

    for t = 1:minutes * 60_000
        for a in 1:na
            adj, last = get_state(env, a)

            moves[a] = argmax(Q[:, sum(adj)+1, last+1])
        end

        env(moves)
    end

    env.mesh.produced, env.mesh.received
end

results.TabQ = bench(
    init = (graph) -> BlueMesh7Env(graph...),
    unroll = unroll_pomdp
)

using PyCall
pushfirst!(PyVector(pyimport("sys")."path"), "../test/ble-mesh-algorithms/standard-algorithms/")
greedy = pyimport("greedy_algorithms")
dominator = pyimport("dominator_algorithm")

results.Connect = bench(
    init = (graph) -> initialize_mesh(first(graph), greedy.greedy_connect(last(graph))),
    unroll = (mesh, minutes) -> start(mesh; minutes)
)

results.Dominator = bench(
    init = (graph) -> initialize_mesh(first(graph), dominator.dominator(last(graph))),
    unroll = (mesh, minutes) -> start(mesh; minutes)
)
