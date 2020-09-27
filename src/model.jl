export generate_positions, initialize_mesh, start, getstats, plotgraph

const ble_37_channel_wavelength = 0.12491352
const part_of_path_loss_for_37 = 20 * log(10, 4 * π / ble_37_channel_wavelength)

function agent_step!(source::Source, model::AgentBasedModel)
    if rand() < model.packet_emit_rate
        # move off to a random position
        move_agent!(source, model)

        # generate next packet
        source.packet_seq = length(model.packets) + 1
        source.packet_ttl = model.ttl
        packet = Packet(source.packet_seq, source.id, rand(1:nagents(model)-1), model.ttl, model.tick, 0, false)
        push!(model.packets, packet)
        model.packet_xs[packet.seq] = Int[]

        source.event_start = model.tick
    end

    source.packet_seq == 0 && return
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

        # back to resting
        source.packet_seq = 0
        source.transmitting = false
        source.n_transmit_left = source.n_og_transmit_count
    end
end

function agent_step!(node::Node, model::AgentBasedModel)
    if node.state == :scanning
        # scanning happens every tick (assuming here scan_window == scan_interval)

        node.channel = div(abs(model.tick - node.event_start), node.t_scan_interval) + 37

        # no packets acquired, repeat scanning
        if node.channel > 39
            node.channel = 37
            node.event_start = model.tick
        end

        node.packet_seq == 0 && return

        # we've seen this seq recently
        if node.packet_seq in node.received_packets
            node.packet_seq = 0
            return
        end

        push!(node.received_packets, node.packet_seq)

        packet = model.packets[node.packet_seq]

        if node.packet_ttl == 1
            node.packet_seq = 0
            return
        end

        node.packet_ttl -= 1

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
        if node.packet_seq == 0
            node.state = :scanning
            node.event_start = model.tick
            return
        end

        d, r = divrem(abs(model.tick - node.event_start), node.t_interpdu)

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

            node.packet_seq = 0
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

function calc_rssi!(agents::Vector{Union{Node, Source}}, P_tx::Vector{Float64})
    n = length(agents)
    rssi_map = Matrix{Float64}(undef, n, n)

    L_p = Matrix{Float64}(undef, n, n)

    for i in 1:n
        for j in 1:(i - 1)
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

function recalc_sources_rssi!(agents::Vector{Union{Node, Source}}, rssi_map:: Matrix{Float64}, P_tx::Vector{Float64})
    n = length(agents)

    source_i = 1
    Lps = vmap!(view(agents, 1:n), x -> 1 / distance(agents[source_i].pos, x.pos) ^ 2)

    @avx for j in 1:n
        Lps[j] = Lps[j] * part_of_path_loss_for_37
        rssi_map[source_i, j] = P_tx[j] * Lps[j] # dB
    end

    @avx for j in 1:n
        rssi_map[j, source_i] = P_tx[source_i] * Lps[j] # dB
    end

    rssi_map[source_i, source_i] = 0

    rssi_map
end

function model_step!(model::AgentBasedModel)
    all_agents = collect(allagents(model))

    transmitters = [
            [a.transmitting && a.channel == 37 for a in all_agents],
            [a.transmitting && a.channel == 38 for a in all_agents],
            [a.transmitting && a.channel == 39 for a in all_agents]
    ]

    recalc_sources_rssi!(all_agents, model.rssi_map, model.tx_powers)

    for (dst_i, dst) in enumerate(all_agents)
        # since we support just 1 message within scanning period
        (dst.state == :scanning && dst.packet_seq == 0) || continue

        neighbours = model.rssi_map[dst_i, transmitters[dst.channel - 36]]
        neighbours_agents = all_agents[transmitters[dst.channel - 36]]
        length(neighbours) > 0 || continue

        # add random noises to calculate RSSI
        neighbours = map(n -> n / to_mW(rand(model.shadow_d) + rand(model.multipath_d)), neighbours)

        visible_mask = [rssi > model.scanner_sensitivity for rssi in neighbours]
        neighbours = neighbours[visible_mask]
        neighbours_agents = neighbours_agents[visible_mask]

        length(neighbours) > 0 || continue

        # add some noises
        total = sum(neighbours) + max(rand(model.wifi_noise_d), 0) + max(rand(model.gaussian_noise), 0)

        # calculate SINR and take sqrt
        neighbours = map(n -> n / (total - n), neighbours)

        # calculate BER
        neighbours = map(n -> 0.5 * erfc(sqrt(n / 2)), neighbours)

        # calculate PAR (Package Accept Rate) (PAR = 1 - PER)
        neighbours = map(n -> (1 - n)^model.msg_bit_length, neighbours)

        total_PAR = sum(neighbours)

        # do any of this packets pass?
        rand() <= total_PAR || continue

        src_i = sample(1:length(neighbours), Weights(neighbours))
        packet = model.packets[neighbours_agents[src_i].packet_seq]

        packet.done && continue
        # registering the touch upon this yet not delivered packet
        push!(model.packet_xs[packet.seq], dst.id)

        # this is mainly for the pomdp interface
        if packet.dst == dst.id
            # since the packet is delivered, acknowledge everyone participating
            append!(model.reward_plate, model.packet_xs[packet.seq])
        end

        # marking the packet as delivered
        if packet.dst == dst.id
            packet.done = true
            packet.time_done = model.tick

            continue
        end

        # copying packet's reference
        dst.packet_seq = packet.seq
        dst.packet_ttl = neighbours_agents[src_i].packet_ttl
    end

    model.tick += 1
end

function plotgraph(model::AgentBasedModel)
    g = SimpleGraph(nagents(model))

    for (src_i, src) in enumerate(allagents(model)), (dst_i, dst) in enumerate(allagents(model))
        src.role == :relay || continue
        dst.role != :source || continue

        if model.rssi_map[src_i, dst_i] > model.scanner_sensitivity
            add_edge!(g, src.id, dst.id)
        end
    end

    colors = Dict(:relay => "orange", :sink => "darkblue", :source => "transparent")

    agents = sort(collect(allagents(model)), by=a -> a.id)
    filter!(a -> a.role != :source, agents)

    xs = map(a -> a.pos[1], agents)
    ys = map(a -> a.pos[2], agents)
    cs = map(a -> colors[a.role], agents)

    gplot(g, xs, ys; nodefillc = cs)
end

to_mW(dBm::Int64) = 10^(dBm / 10)

"""
    generate_positions(; dims = (200, 200), n = 100) → positions::Array{Tuple{Int,Int},1}

Generates random xy-coordinates for `n` nodes with `dims`
"""
generate_positions(; dims=(100, 100), n=64) = [(rand(1:dims[1]), rand(1:dims[2])) for _ in 1:n]

"""
    initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int}) → mesh::AgentBasedModel

Create a mesh by specifying roles of each node by `roles`=[0, 1, 0...] where 1 means that the node is a relay, sink otherwise, and including `positions`=[(Int, Int)...] as nodes' positions
"""
function initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int})
    length(roles) == length(positions) || throw(ArgumentError("Both arguments must be of equal length"))
    n = length(roles) + 1

    properties = Dict(
        :tick => 0,
        :packet_error_rate => 0.05,
        :packet_emit_rate => 1 / 1000,
        :ttl => 4,
        :rssi_map => Matrix{Float64}(undef, n, n),
        :tx_powers => Vector{Float64}(undef, n),
        :scanner_sensitivity => to_mW(-95),
        :msg_bit_length => 312,
        :shadow_d => LogNormal(0, 4),
        :multipath_d => Rayleigh(4),
        :wifi_noise_d => Normal((to_mW(-125) - to_mW(-135)) / 2 + to_mW(-135), (to_mW(-125) - to_mW(-135)) / 2),
        :gaussian_noise => Normal((to_mW(-125) - to_mW(-135)) / 2 + to_mW(-135), (to_mW(-125) - to_mW(-135)) / 2),
        :packets => Packet[],
        # packet.seq => nodes that have touched that packet
        :packet_xs => Dict{Int, Vector{Int}}(),
        # plate for the actors who contributed to packet's successful delivery
        :reward_plate => Int[]
    )

    space = GridSpace((maximum(first.(positions)), maximum(last.(positions))))
    model = ABM(Union{Source, Node}, space; properties, warn = false)

    source = Source(id = 0, pos = (1, 1))
    model.tx_powers[1] = source.tx_power
    add_agent_single!(source, model)

    for (pos, roles_i) in zip(positions, roles)
        node = Node(id = nextid(model); pos, role = roles_i == 1 ? :relay : :sink)

        add_agent_pos!(node, model)

        model.tx_powers[node.id] = node.tx_power
    end

    model.rssi_map = calc_rssi!(collect(allagents(model)), model.tx_powers)
    model
end

function getstats(model::AgentBasedModel)
    # filter packets which were too recently produced, or produced in an unreachable position
    filter!(packet -> packet.time_start < model.tick - 5000 && !isempty(model.packet_xs[packet.seq]), model.packets)

    # worst case in terms of PDR to a single device
    worstnode = argmax(map(id -> count(p -> p.dst == id && p.done == false, model.packets), 1:nagents(model)-1))
    deprived = filter(p -> p.dst == worstnode, model.packets)
    delivered = filter(p -> p.done == true, model.packets)
    delays = map(p -> p.time_done - p.time_start, delivered)

    return ( PDR = length(delivered) / length(model.packets),
             worstPDR = count(p -> p.done == false, deprived) / length(deprived),
             delay = sum(delays) / length(delays) )
end

"""
    start(model::AgentBasedModel, minutes = 1) → (received, produced)

Start an experiment from `model` for the given number of minutes.
"""
function start(model::AgentBasedModel; minutes = 1)
    steps = minutes * 60 * 1000

    run!(model, agent_step!, model_step!, steps)

    getstats(model)
end
