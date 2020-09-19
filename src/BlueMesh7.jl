module BlueMesh7
using Agents, AgentsPlots
using LightGraphs, GraphPlot
using Random
using StatsBase
include("MeshAgents.jl")
using .MeshAgents: Node, Source, Packet

export generate_positions, initialize_mesh, run, plotgrid

function agent_step!(source::Source, model::AgentBasedModel)
    if rand() < model.packet_emit_rate
        # move off to a random position
        move_agent!(source, model)

        source.packet = Packet(model.tick, source.id, rand(2:nagents(model)), model.ttl)
        source.event_start = model.tick

        model.produced += 1
    end

    source.packet === nothing && return
    model.tick < source.event_start && return

    d, r = divrem(model.tick - source.event_start, source.t_interpdu)
    source.transmitting = r == 0
    source.channel = d + 37

    if source.channel > 39
        if source.n_transmit_left > 0
            source.event_start = model.tick + source.t_og_transmit_delay + rand(source.rt_og_random_delay)
            source.n_transmit_left -= 1
            return
        end

        source.packet = nothing
        source.transmitting = false
        source.n_transmit_left = source.n_og_transmit_count
    end
end

function agent_step!(node::Node, model::AgentBasedModel)
    if node.state == :scanning
        # scanning happens every tick (assuming here scan_window == scan_interval)
        node.channel = div(model.tick - node.event_start, node.t_scan_interval) + 37

        # no packets acquired, repeat
        if node.channel > 39
            node.channel = 37
            node.event_start = model.tick
        end

        # instead of type instability here, zero(Packet) might work
        node.packet === nothing && return

        if node.packet.seq in node.received_packets
            node.packet = nothing

            return
        end

        push!(node.received_packets, node.packet.seq)

        if node.packet.dst == node.id
            model.received += 1
            node.packet = nothing

            return
        end

        if node.packet.ttl == 0
            node.packet = nothing

            return
        end

        node.packet.ttl -= 1

        if node.role == :relay
            # initiating back-off delay before advertising this packet
            node.event_start = model.tick + node.t_back_off_delay
            node.state = :advertising

            return
        end
    end

    node.role != :relay && return

    if node.state == :advertising
        model.tick < node.event_start && return

        # return to scanning if the packet is absent
        if node.packet === nothing
            node.state = :scanning
            node.event_start = model.tick

            return
        end

        d, r = divrem(model.tick - node.event_start, node.t_interpdu)

        # transmitting happens once for each channel
        node.transmitting = iszero(r)

        node.channel = d + 37

        # the end of the advertising
        if node.channel > 39
            # bonus retransmissions for the original/retx packet
            if node.n_transmit_left > 0
                node.event_start = model.tick + node.t_retx_transmit_delay + rand(node.rt_retx_random_delay)

                node.n_transmit_left -= 1

                return
            end

            node.packet = nothing
            node.transmitting = false
            node.state = :scanning
            node.event_start = model.tick + 1
            node.n_transmit_left = node.n_retx_transmit_count
        end
    end
end

distance(a::Tuple{Int, Int}, b::Tuple{Int, Int}) = sqrt((a[1] - b[1]) ^ 2 + (a[2] - b[2]) ^ 2)

function calc_rssi!(agents::Array{Union{Node, Source}, 1})
    n = length(agents)
    rssi_map = Matrix{Float64}(undef, n, n)

    ble_37_channel_wavelength = 0.12491352
    path_loss_multiplier = 4 * π / ble_37_channel_wavelength

    for i in 1:n
        for j in 1:n
            if i == j
                rssi_map[i, i] = -Inf
                continue
            end

            rssi_map[i, j] = min(agents[j].tx_power - (20 * log(
                10, max(path_loss_multiplier * distance(agents[i].pos, agents[j].pos), 1e-9)
            )), 0) # dBm
        end
    end

    rssi_map
end

function recalc_sources_rssi!(agents::Array{Union{Node, Source}, 1}, sources::Array{Int, 1})
    global rssi_map

    ble_37_channel_wavelength = 0.12491352
    path_loss_multiplier = 4 * π / ble_37_channel_wavelength

    for source_idx in sources
       for i in 2:length(agents)
             rssi_map[source_idx, i] = min(agents[source_idx].tx_power - (20 * log(
                10, max(path_loss_multiplier * distance(agents[source_idx].pos, agents[i].pos), 1e-9)
            )), 0) # dBm
        end
    end

    rssi_map
end

# optimization for static mesh
rssi_map = Matrix{Float64}(undef, 0, 0)

function model_step!(model::AgentBasedModel)
    all_agents = collect(allagents(model))
    scanners = filter(i -> all_agents[i].state == :scanning, 1:length(all_agents))
    transmitters = filter(i -> all_agents[i].transmitting, 1:length(all_agents))

    scanner_sensitivity = -95 # dBm
    wifi_rssi           = -70 # dBm
    gaussian_noise      = -90 # dBm

    global rssi_map
    sources = [1]
    rssi_map = length(rssi_map) > 0 ? recalc_sources_rssi!(all_agents, sources) : calc_rssi!(all_agents)

    for scanner in scanners
        # since we support just 1 message within scanning period
        all_agents[scanner].packet == nothing || continue

        neighbours_i = [i for i in transmitters if rssi_map[scanner, i] > scanner_sensitivity]

        length(neighbours_i) > 0 || continue

        neighbours_probs = [
            rssi_map[neighbours_i]...,
            wifi_rssi + rand(-20:20),
            gaussian_noise + rand(-10:10)
        ]

        # add dummy indexes for wifi and noise
        append!(neighbours_i, [0, 0])

        neighbours_probs /= sum(neighbours_probs)

        tx_i = StatsBase.sample(neighbours_i, StatsBase.ProbabilityWeights(neighbours_probs))

        tx_i > 0 || continue

        all_agents[scanner].packet = deepcopy(all_agents[tx_i].packet)
    end

    model.tick += 1
end

function plotgrid(model::AgentBasedModel)
    g = SimpleGraph(nagents(model))

    colors = Dict(:relay => "orange", :sink => "darkblue", :source => "transparent")

    agents = sort(collect(allagents(model)), by=a -> a.id)
    xs = map(a -> a.pos[1], agents)
    ys = map(a -> a.pos[2], agents)
    cs = map(a -> colors[a.role], agents)

    gplot(g, xs, ys; nodefillc = cs)
end

"""
    generate_positions(; dims = (200, 200), n_nodes = 100) → positions::Array{Tuple{Int,Int},1}

Generates random xy-coordinates of the nodes
"""
generate_positions(; dims = (100, 100), n_nodes = 64) = [(rand(1:dims[1]), rand(1:dims[2])) for _ in 1:n_nodes]

"""
    initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int}) → mesh::AgentBasedModel

Create a mesh by specifying roles of each node by `roles`=[0, 1, 0...] where 1 means that the node is a relay, sink otherwise, and including `positions`=[(Int, Int)...] as nodes' positions
"""
function initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int})
    length(roles) == length(positions) || throw(ArgumentError("Both arguments must be of equal length"))

    properties = Dict(
        :tick => 0,
        :packet_error_rate => 0.05,
        :packet_emit_rate => 10 / 1000,
        :ttl => 4,
        :received => 0,
        :produced => 0
    )

    space = GridSpace((maximum(first.(positions)), maximum(last.(positions))), moore = true)
    model = ABM(Union{Source, Node}, space; properties, warn = false)

    add_agent_single!(Source(id = 1, pos = (1, 1)), model)

    for i in 2:length(roles)
        node = Node(id = i; pos = positions[i], role = roles[i] == 1 ? :relay : :sink)

        add_agent_pos!(node, model)
    end

    model
end

"""
    start(model::AgentBasedModel, minutes = 1) → (received, produced)

Start an experiment from `model` for the given number of minutes.
"""
function start(model::AgentBasedModel; minutes = 1)
    steps = minutes * 60 * 1000

    _, dfm = run!(model, agent_step!, model_step!, steps; when_model = [steps], mdata = [:produced, :received])

    dfm[:produced][1], dfm[:received][1]
end
end
