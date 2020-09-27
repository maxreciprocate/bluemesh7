include("../src/BlueMesh7.jl")
using .BlueMesh7

using Random
using DataFrames
using Statistics: mean
using ProgressMeter: @showprogress
using BSON: @load, @save
# ■

benchmark = DataFrame()
minutes = 7
ngraphs = 7

n_nodes = [32, 64, 128]
dims = [(100, 100), (121, 121), (144, 144)]

setups = [(d, n) for d in dims, n in n_nodes] |> vec
results = DataFrame()

Random.seed!(7)
allgraphs = [generate_graph(; n_nodes = n, dims = d) for _ = 1:ngraphs, (d, n) in setups]

function bench(name; init, unroll)
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

            push!(setup_stats, stats)
        end

        @assert nrow(setup_stats) == ngraphs
        averages = [setup, colwise(mean, setup_stats)...]
        push!(bench_stats, NamedTuple{(:setup, :pdr, :worstpdr, :delay)}(averages))
    end

    bench_stats.algorithm = name

    for row in eachrow(bench_stats)
        push!(benchmark, row)
    end
end
# ■

bench("All relays",
      init = (graph) -> initialize_mesh(first(graph), ones(Int, length(first(graph)))),
      unroll = (mesh, minutes) -> start(mesh; minutes))

bench("Half relays",
      init = (graph) -> initialize_mesh(first(graph), rand(0:1, length(first(graph)))),
      unroll = (mesh, minutes) -> start(mesh; minutes))

using PyCall
pushfirst!(PyVector(pyimport("sys")."path"), "../test/ble-mesh-algorithms/standard-algorithms/")
greedy = pyimport("greedy_algorithms")
dominator = pyimport("dominator_algorithm")

bench("Connect",
      init = (graph) -> initialize_mesh(first(graph), greedy.greedy_connect(last(graph))),
      unroll = (mesh, minutes) -> start(mesh; minutes))

bench("Dominator",
      init = (graph) -> initialize_mesh(first(graph), dominator.dominator(last(graph))),
      unroll = (mesh, minutes) -> start(mesh; minutes))

# ■

@load "../src/Q-LAVISH.bson" Q

function unroll_pomdp(env, minutes)
    na = length(env.positions)
    moves = zeros(Int, na)

    for t = 1:minutes * 60_000
        for a in 1:na
            nbours, haspacket = get_state(env, a)

            moves[a] = argmax(Q[:, haspacket, nbours])
        end

        env(moves)
    end

    BlueMesh7.getstats(env.mesh)
end

bench("Tabular Q",
      init = (graph) -> BlueMesh7Env(graph...),
      unroll = unroll_pomdp)

# ■

using Dates
datenow = Dates.now()
@save "benchmark-$(datenow).bson" benchmark

# ■
using Gadfly
using Cairo, Compose

set_default_plot_size(21cm, 16cm)

benchmark.ssetup = repr.(benchmark.setup)
p = plot(benchmark,
     x=:ssetup, y=:worstpdr, color=:algorithm, Geom.point, Geom.line,
     # Guide.yticks(ticks=0.3:0.1:1),
     Guide.title("The worst PDR out of 7 rollouts"),
     Guide.xlabel("Setup (#nodes, meters squared)"),
     Guide.ylabel("worst packet delivery ratio"),
     Theme(
         # minor_label_font="Fira Code",
         # minor_label_font_size=11px,
         # minor_label_color="black",
         major_label_font="Fira Code",
         major_label_font_size=18px,
         # major_label_color="dark grey",
         key_label_font="Fira Code",
         key_label_font_size=13px,
         # major_label_color="dark grey",
         key_title_font="Fira Code",
         key_title_font_size=16px,
         # major_label_color="dark grey",
     ))

draw(SVG("benchmark-$(datenow).svg", 21cm, 16cm), p)
