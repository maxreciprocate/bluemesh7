include("../src/BlueMesh7.jl")
# using .BlueMesh7

using Random
using DataFrames
using Statistics: mean
using ProgressMeter
using BSON: @load, @save
using Flux
using CUDA
# ■

benchmark = DataFrame()
minutes = 7
ngraphs = 4

nactors = [32, 64, 96]
dims = [(25, 25), (30, 30), (35, 35)]

setups = [(d, n) for d in dims, n in nactors] |> vec
results = DataFrame()

Random.seed!(7)
allgraphs = [generate_positions(; n, dims) for _ = 1:ngraphs, (dims, n) in setups]

Progress(10kk)
function bench(name, benchmark; init, unroll)
    println("Benching $(name)")
    bench_stats = DataFrame()

    @showprogress for (setup, graphs) in zip(setups, eachcol(allgraphs))
        setup_stats = DataFrame()
        spin = Threads.SpinLock()

        Threads.@threads for graph in graphs
            # GIL avoidance for python algorithms
            lock(spin)
            mesh = init(graph)
            unlock(spin)

            Random.seed!(7)
            stats = unroll(mesh, minutes)

            # since when it's not thread safe
            lock(spin)
            push!(setup_stats, stats)
            unlock(spin)
        end

        @assert nrow(setup_stats) == ngraphs
        averages = [setup, colwise(mean, setup_stats)...]
        push!(bench_stats, NamedTuple{(:setup, :pdr, :worstpdr, :delay, :centrality, :packetloss)}(averages))
    end

    bench_stats.algorithm = name

    for row in eachrow(bench_stats)
        push!(benchmark, row)
    end
end

bench("All relays (0)", benchmark,
      init = coordinates -> initialize_mesh(coordinates, ones(Int, length(coordinates))),
      unroll = (mesh, minutes) -> start(mesh; minutes))

bench("Half relays (0)", benchmark,
      init = coordinates -> initialize_mesh(coordinates, rand(0:1, length(coordinates))),
      unroll = (mesh, minutes) -> start(mesh; minutes))

using PyCall
pushfirst!(PyVector(pyimport("sys")."path"), "../test/ble-mesh-algorithms/standard-algorithms/")
greedy = pyimport("greedy_algorithms")
dominator = pyimport("dominator_algorithm")

function init_rssi_to_adjacency(coordinates::Vector{NTuple{2, Int}}, Oracle)
    protomesh = initialize_mesh(coordinates, ones(Int, length(coordinates)))
    mask = protomesh.rssi_map .> protomesh.scanner_sensitivity + 1e-10

    adjacency = zeros(Int, size(protomesh.rssi_map))
    adjacency[mask] .= 1
    adjacency = adjacency[2:end, 2:end]

    roles = Oracle(adjacency)
    initialize_mesh(coordinates, roles)
end

bench("Connect (1)", benchmark,
      init = coordinates -> init_rssi_to_adjacency(coordinates, greedy.greedy_connect),
      unroll = (mesh, minutes) -> start(mesh; minutes))

bench("Dominator (1)", benchmark,
      init = coordinates -> init_rssi_to_adjacency(coordinates, dominator.dominator),
      unroll = (mesh, minutes) -> start(mesh; minutes))

@load "model.bson" model

function unroll_mc(env, minutes)
    moves = zeros(env.model.n)
    S = get_state(env)

    ps = model(S) |> softmax

    moves = rand.(mapslices(Categorical, ps', dims=2)) |> vec

    env(moves)

    for t = 1:minutes * 60_000
        env(moves, true)
    end

    getstats(env.model)
end

bench("Monte Carlo (3)", benchmark,
      init = (positions) -> BlueMesh7Env(positions),
      unroll = unroll_mc)

# ■

using Dates
datenow = Date(now())
@save "benchmark-$(datenow).bson" benchmark
