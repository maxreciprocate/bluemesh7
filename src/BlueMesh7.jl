module BlueMesh7
using Agents, AgentsPlots
using LightGraphs, GraphPlot
using Random
using DataStructures: CircularBuffer

export generate_graph, initialize_mesh, run, plotgraph, plotgrid

mutable struct Packet
    seq::Int
    src::UInt16
    dst::UInt16
    last::UInt16
    ttl::UInt8
end

Base.@kwdef mutable struct Node <: AbstractAgent
    id::Int

    # (x, y) coordinates
    pos :: Tuple{Int, Int}

    # role = :relay | :sink
    role :: Symbol

    # channel = 37 | 38 | 39
    channel :: UInt8 = 37

    # state = :scanning | :advertising | :sleeping
    state :: Symbol = :scanning

    # the time of the event's (current state) begging
    event_start :: Int = rand(0:10)

    # whether the device is currently sending data (this removes the need for redundant allocations)
    transmitting :: Bool = false

    # the power of the transmission
    dBm :: Int = 4

    # the length of the interval between transmissions on different channels (ms)
    t_interpdu :: UInt = 5

    # the length of the interval between scanning on different channels (ms)
    t_scan_interval :: UInt = 20

    # the length of the interval before advertising the received packet
    t_back_off_delay :: UInt = 5

    # the number of extra retransmissions of the received packet
    n_retx_transmit_count :: UInt = 1

    # the length of the delay between bonus transmissions of the received packet (ms)
    t_retx_transmit_delay :: UInt = 20

    # additional random component of delay between retransmissions (min:max ms)
    rt_retx_random_delay :: UnitRange{Int} = 20:50

    # the current number of extra transmissions left
    n_transmit_left :: UInt = 0

    # buffer containing seq of the received packets
    received_packets :: CircularBuffer{Int} = CircularBuffer{Int}(20)

    # the current advertisement
    packet :: Union{Packet, Nothing} = nothing
end

Base.@kwdef mutable struct Source <: AbstractAgent
    id :: Int
    pos :: Tuple{Int, Int}

    role :: Symbol = :source
    state :: Symbol = :advertising

    channel :: UInt8 = 37
    event_start :: UInt = 0
    dBm :: Int = 4
    transmitting :: Bool = false

    t_interpdu :: UInt = 5

    # the number of extra transmissions of the packet originating from the node
    n_og_transmit_count :: UInt = 2

    # the length of the delay between additional transmissions of the original packet (ms)
    t_og_transmit_delay :: UInt = 20

    # the range of the additional random component to delay between additional original transmissions
    rt_og_random_delay :: UnitRange{Int} = 0:20

    # the current number of extra transmissions left
    n_transmit_left :: UInt = 0

    packet :: Union{Packet, Nothing} = nothing
end

function agent_step!(source::Source, model::AgentBasedModel)
    if rand() < model.packet_emit_rate
        # move off to a random position
        move_agent!(source, model)

        source.packet = Packet(model.tick, source.id, rand(1:nagents(model)-1), source.id, model.ttl)
        source.event_start = model.tick

        model.travelling_packets[source.packet.seq] = Int[]
        # @show source.packet.seq, source.packet.dst
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
        node.channel = div(abs(model.tick - node.event_start), node.t_scan_interval) + 37

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

        # already delivered
        if node.packet.dst == node.id
            node.packet = nothing
            return
        end

        if node.packet.ttl == 0
            node.packet = nothing

            return
        end

        # registering the touch upon this yet not delivered packet
        seq = node.packet.seq
        if haskey(model.travelling_packets, seq)
            # println("$(node.id) registering $(seq)")
            push!(model.travelling_packets[seq], node.id)
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

        d, r = divrem(abs(model.tick - node.event_start), node.t_interpdu)

        # transmitting happens once for each channel
        node.transmitting = iszero(r)
        # if node.transmitting
        #     println("$(node.id) broadcasting $(node.packet.seq)")
        # end

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

power_to_distance(dBm::Int) = exp(dBm)
distance(a::Tuple{Int, Int}, b::Tuple{Int, Int}) = sqrt((a[1] - b[1]) ^ 2 + (a[2] - b[2]) ^ 2)

function model_step!(model::AgentBasedModel)
    transmitters = filter(agent -> agent.transmitting, collect(allagents(model)))

    empty!(model.reward_plate)

    for dst in allagents(model)
        dst.state == :scanning || continue

        neighbours = filter(src -> distance(src.pos, dst.pos) < power_to_distance(src.dBm) && src.channel == dst.channel, transmitters)
        length(neighbours) != 1 && continue

        rand() < model.packet_error_rate && continue

        packet = first(neighbours).packet
        # marking the packet as delivered before the actor himself has even noticed
        if dst.id == packet.dst && haskey(model.travelling_packets, packet.seq)
            append!(model.reward_plate, model.travelling_packets[packet.seq])
            delete!(model.travelling_packets, packet.seq)

            model.received += 1
        end

        dst.packet = deepcopy(first(neighbours).packet)
        dst.packet.last = first(neighbours).id
    end

    for seq in keys(model.travelling_packets)
        if model.tick - seq > 300
            delete!(model.travelling_packets, seq)
        end
    end

    model.tick += 1
end


function plotgraph(model::AgentBasedModel)
    g = SimpleGraph(nagents(model) - 1)

    for src in allagents(model), dst in allagents(model)
        src.role == :relay || continue
        dst.role != :source || continue

        if distance(src.pos, dst.pos) <= power_to_distance(src.dBm)
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

"""
    generate_graph(; dims = (200, 200), n_nodes = 100) → adjacency, positions

Generates a random graph and returns the adjacency matrix and xy-coordinates of the nodes
"""
function generate_graph(; dims = (100, 100), n_nodes = 64)
    positions = [(rand(1:dims[1]), rand(1:dims[2])) for _ in 1:n_nodes]
    g = SimpleGraph(n_nodes)

    for src in 1:n_nodes, dst in src+1:n_nodes
        if distance(positions[src], positions[dst]) <= exp(4)
            add_edge!(g, src, dst)
        end
    end

    positions, Matrix(adjacency_matrix(g))
end

"""
    initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int}) → mesh::AgentBasedModel

Create a mesh by specifying roles of each node by `roles`=[0, 1, 0...] where 1 means that the node is a relay, sink otherwise, and including `positions`=[(Int, Int)...] as nodes' positions
"""
function initialize_mesh(positions::Vector{Tuple{Int, Int}}, roles::Vector{Int})
    length(roles) == length(positions) || throw(ArgumentError("Both arguments must be of equal length"))

    properties = Dict(
        :tick => 0,
        :packet_error_rate => 0.05,
        :packet_emit_rate => 5 / 1000,
        :ttl => 4,
        :received => 0,
        :produced => 0,
        # packet.seq => nodes that touched that packet
        :travelling_packets => Dict{Int, Vector{Int}}(),
        # plate for the actors who contributed to packet's successful delivery
        :reward_plate => Int[]
    )

    space = GridSpace((maximum(first.(positions)), maximum(last.(positions))), moore = true)
    model = ABM(Union{Source, Node}, space; properties, warn = false)

    for (pos, roles_i) in zip(positions, roles)
        node = Node(id = nextid(model); pos, role = roles_i == 1 ? :relay : :sink)

        add_agent_pos!(node, model)
    end

    add_agent_single!(Source(id = 0, pos = (1, 1)), model)

    model
end

"""
    start(model::AgentBasedModel, minutes = 1) → (received, produced)

Start an experiment from `model` for the given number of minutes.
"""
function start(model::AgentBasedModel; minutes = 1)
    steps = minutes * 60 * 1000

    _, dfm = run!(model, agent_step!, model_step!, steps)

    model.produced - count(kv -> isempty(last(kv)), mesh.travelling_packets), model.received
end
end
