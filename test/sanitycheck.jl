include("../src/BlueMesh7.jl")
# ■

using Random
using DataFrames
using Statistics: mean
using ProgressMeter: @showprogress
using BSON: @load, @save
# ■

benchmark = DataFrame()
minutes = 2
ngraphs = 7

nactors = [16, 32, 64, 96]
dims = [(20, 20), (30, 30)]

setups = [(d, n) for d in dims, n in nactors] |> vec
results = DataFrame()

Random.seed!(7)
allgraphs = [generate_positions(; n, dims) for _ = 1:ngraphs, (dims, n) in setups]

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
            stats = unroll(mesh, setup, minutes)

            push!(setup_stats, stats)
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

bench("All relays", benchmark,
      init = coordinates -> initialize_mesh(coordinates, ones(Int, length(coordinates))),
      unroll = (mesh, setup, minutes) -> start(mesh; minutes))

bench("Half relays", benchmark,
      init = coordinates -> initialize_mesh(coordinates, rand(0:1, length(coordinates))),
      unroll = (mesh, setup, minutes) -> start(mesh; minutes))
# ■

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

bench("Connect", benchmark,
      init = coordinates -> init_rssi_to_adjacency(coordinates, greedy.greedy_connect),
      unroll = (mesh, setup, minutes) -> start(mesh; minutes))

bench("Dominator", benchmark,
      init = coordinates -> init_rssi_to_adjacency(coordinates, dominator.dominator),
      unroll = (mesh, setup, minutes) -> start(mesh; minutes))
# ■
# @load "../src/10.4-ground-bench.bson" benchmark
@load "../solving/MC-EXP.bson" Q

tilings = [0, 1, 2, 3, [4 * idx for idx = 1:24]...] |> reverse
function tile(nbours::Int)
    for idx in 1:length(tilings)
        if tilings[idx] <= nbours
            return idx
        end
    end
end

function unroll_mc(env, setup, minutes)
    na = length(env.positions)
    moves = zeros(Int, na)
    dim = setup[1][1] ÷ 10

    for a in 1:na
        nbours, _ = get_state(env, a)

        moves[a] = argmax(Q[:, tile(nbours), dim])
    end

    env(moves)

    for t = 1:minutes * 60_000
        env(moves, true)
    end

    getstats(env.mesh)
end

bench("Monte Carlo", benchmark,
      init = (positions) -> BlueMesh7Env(positions),
      unroll = unroll_mc)
# ■

@load "../src/Q-MEGA.bson" Q

function unroll_pomdp(env, setup, minutes)
    na = length(env.positions)
    moves = zeros(Int, na)
    dim = (setup[1][1] - 10) ÷ 10

    for t = 1:minutes * 60_000
        for a in 1:na
            nbours, haspacket = get_state(env, a)

            moves[a] = argmax(Q[:, nbours, haspacket])
        end

        env(moves)
    end

    getstats(env.mesh)
end

bench("TD(0)", benchmark,
      init = (positions) -> BlueMesh7Env(positions),
      unroll = unroll_pomdp)

using Dates
datenow = Date(now())
@save "benchmark-$(datenow).bson" benchmark
