module BlueMesh7
using Agents, AgentsPlots
using LightGraphs, GraphPlot
using Random
using StatsBase
include("MeshAgents.jl")
using .MeshAgents: Node, Source, Packet
using Distributions
using LoopVectorization
using SpecialFunctions

export generate_positions, initialize_mesh, run, plotgrid

const ble_37_channel_wavelength = Float64(0.12491352)
const part_of_path_loss_for_37 = Float64(20 * log(10, 4 * π / ble_37_channel_wavelength))
const Standard_Normal_d = Normal()

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
                node.channel = 37
                return
            end

            node.packet = nothing
            node.transmitting = false
            node.state = :scanning
            node.event_start = model.tick + 1
            node.n_transmit_left = node.n_retx_transmit_count
            node.channel = 37
        end
    end
end

distance(a::Tuple{Int, Int}, b::Tuple{Int, Int}) = sqrt((a[1] - b[1]) ^ 2 + (a[2] - b[2]) ^ 2)

function vmap!(arr::SubArray, func::Function, arr2::SubArray)
    length(arr) > 0 || return Array{typeof(func(arr[1])), 1}(undef, 0)
    @avx for i in 1:length(arr)
        arr2[i] = func(arr[i])
    end
    arr2
end

function vmap!(arr::SubArray, func::Function)
    length(arr) > 0 || return Array{typeof(func(arr[1])), 1}(undef, 0)
    arr2 = Array{typeof(func(arr[1])), 1}(undef, length(arr))
    @avx for i in 1:length(arr)
        arr2[i] = func(arr[i])
    end
    arr2
end

function calc_rssi!(agents::Array{Union{Node, Source}, 1}, P_tx::Array{Float64, 1})
    n = length(agents)
    rssi_map = Matrix{Float64}(undef, n, n)

    L_p = Matrix{Float64}(undef, n, n)

    for i in 1:n
        for j in 1:(i - 1) # more cache-friendly
            L_p[i, j] = L_p[j, i]
        end

        for j in (i + 1):n
            L_p[i, j] = 1 / distance(agents[i].pos, agents[j].pos) ^ 2
        end
    end

    @avx for i in 1:n
        for j in 1:n
            rssi_map[i, j] = P_tx[j] * L_p[i, j] * part_of_path_loss_for_37 # dB
        end

        rssi_map[i, i] = 0
    end

    rssi_map
end

function recalc_sources_rssi!(agents::Array{Union{Node, Source}, 1}, sources::Array{Int, 1},
        rssi_map:: Matrix{Float64}, P_tx::Array{Float64, 1})
    n = length(agents)

    for i in sources
        Lps = vmap!(view(agents, 1:n), x -> 1 / distance(agents[i].pos, x.pos) ^ 2)

        @avx for j in 1:n
            Lps[j] = Lps[j] * part_of_path_loss_for_37
            rssi_map[i, j] = P_tx[j] * Lps[j] # dB
        end

        @avx for j in 1:n
            rssi_map[j, i] = P_tx[i] * Lps[j] # dB
        end

        rssi_map[i, i] = 0

    end

    rssi_map
end


function model_step!(model::AgentBasedModel)
    all_agents = collect(allagents(model))

    transmitters = [
            [a.transmitting && a.channel == 37 for a in all_agents],
            [a.transmitting && a.channel == 38 for a in all_agents],
            [a.transmitting && a.channel == 39 for a in all_agents]
    ]

    recalc_sources_rssi!(all_agents, model.source_nodes, model.rssi_map, model.tx_powers)

    for (agent_i, agent) in enumerate(all_agents)
        # since we support just 1 message within scanning period
        (agent.state == :scanning && agent.packet == nothing) || continue

        neighbours = model.rssi_map[agent_i, transmitters[agent.channel - 36]]
        neighbours_agents = all_agents[transmitters[agent.channel - 36]]
        length(neighbours) > 0 || continue

        # add random noises to calculate RSSI
        neighbours = map(n -> n * rand(model.shadow_d) * rand(model.multipath_d), neighbours)

        visible_mask = [rssi > model.scanner_sensitivity for rssi in neighbours]
        neighbours = neighbours[visible_mask]
        neighbours_agents = neighbours_agents[visible_mask]

        length(neighbours) > 0 || continue

        # add some noises
        total = sum(neighbours) + max(rand(model.wifi_noise_d), 0) + max(rand(model.gaussian_noise), 0)
        #println("RSSI", neighbours, total)

        # calculate SINR and take sqrt
        neighbours = map(n -> n / (total - n), neighbours)

        # calculate BER
        neighbours = map(n -> 0.5 * erfc(sqrt(n / 2)), neighbours)

        # calculate PAR (Package Accept Rate) (PAR = 1 - PER)
        neighbours = map(n -> (1 - n)^model.msg_bit_length, neighbours)

        total_PAR = sum(neighbours)

        rand() <= total_PAR || continue # then we will receive message

        tx_i = StatsBase.sample(1:length(neighbours), Weights(neighbours))
        agent.packet = deepcopy(neighbours_agents[tx_i].packet)
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

function plotgraph(model::AgentBasedModel)
    g = SimpleGraph(nagents(model))

    for (src_i, src) in enumerate(allagents(model)), (dst_i, dst) in enumerate(allagents(model))
        src.role == :relay || continue
        dst.role != :source || continue

        if model.rssi_map[src_i, dst_i] * mean(model.multipath_d) * mean(model.shadow_d) > model.scanner_sensitivity
            add_edge!(g, src.id, dst.id)
        end
    end

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
to_mW(dBm::Int64) = 10^(dBm / 10)
function initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int})
    length(roles) == length(positions) || throw(ArgumentError("Both arguments must be of equal length"))

    properties = Dict(
        :tick                   => 0,
        :packet_error_rate      => 0.05,
        :packet_emit_rate       => 10 / 1000,
        :ttl                    => 4,
        :msg_bit_length         => 312,
        :received               => 0,
        :produced               => 0,
        :rssi_map               => Matrix{Float64}(undef, 0, 0),
        :source_nodes           => Array{Int64, 1}(),
        :tx_powers              => Array{Float64, 1}(undef, length(roles)),
        :scanner_sensitivity    => to_mW(-95),
        :shadow_d               => LogNormal((to_mW(0) - to_mW(-4)) / 2 + to_mW(-4), (to_mW(0) - to_mW(-4)) / 2),
        :multipath_d            => Rayleigh(to_mW(-4)),
        :wifi_noise_d           => Normal((to_mW(-125) - to_mW(-135)) / 2 + to_mW(-135), (to_mW(-125) - to_mW(-135)) / 2),
        :gaussian_noise         => Normal((to_mW(-125) - to_mW(-135)) / 2 + to_mW(-135), (to_mW(-125) - to_mW(-135)) / 2)
    )

    space = GridSpace((maximum(first.(positions)), maximum(last.(positions))), moore = true)
    model = ABM(Union{Source, Node}, space; properties, warn = false)

    model.source_nodes = [1]
    source = Source(id = 1, pos = (1, 1))
    add_agent_single!(source, model)
    model.tx_powers[1] = source.tx_power

    for i in 2:length(roles)
        node = Node(id = i; pos = positions[i], role = roles[i] == 1 ? :relay : :sink)

        add_agent_pos!(node, model)

        model.tx_powers[i] = node.tx_power
    end

    model.rssi_map = calc_rssi!(collect(allagents(model)), model.tx_powers)

    #println("model.rssi_map: ", model.rssi_map)
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
