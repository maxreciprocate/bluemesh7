module BlueMesh7
using Agents, AgentsPlots
using LightGraphs, GraphPlot
using Random

export random_graph, initialize_model, start_model, plotgraph, plotgrid

mutable struct Packet
    seq::Int
    src::UInt16
    dst::UInt16

    ttl::UInt8
end

mutable struct Node <: AbstractAgent
    id::Int

    # (x, y) coordinates
    pos::Tuple{Int, Int}

    # mode = relay | ignore
    mode::Symbol

    # state = scanning | advertising | sleeping (back-off delay)
    state::Symbol

    # channel = 37 | 38 | 39
    channel::UInt8

    # whether the device is currently sending data (this removes the need for redundant allocations)
    transmitting::Bool

    # the time of the event's (current state) begging
    event_start::UInt

    # timings
    t_interpdu::UInt
    t_scan_interval::UInt
    t_back_off_delay::UInt

    # ought to put a length limit on this one
    received_cache::Vector{Int}

    # the current advertisement
    packet::Union{Packet, Nothing}

    Node(id, xy; mode, t_interpdu, t_scan_interval, t_back_off_delay) =
        new(id, xy, mode, :scanning, 0, false, 0, t_interpdu, t_scan_interval, t_back_off_delay, Int[], nothing)
end

function agent_step!(node::Node, model)
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

        if node.packet.seq in node.received_cache
            node.packet = nothing

            return
        end

        push!(node.received_cache, node.packet.seq)

        if node.packet.dst == node.id
            model.received += 1

            return
        end

        if node.packet.ttl == 0
            node.packet = nothing

            return
        end

        node.packet.ttl -= 1

        # initiating back-off delay before advertising this packet
        node.state = :sleeping
        node.event_start = model.tick + rand(0:5)
    end

    node.mode != :relay && return

    if node.state == :sleeping
        if model.tick - node.event_start > node.t_back_off_delay
            node.state = :advertising
            node.event_start = model.tick
        end
    end

    if node.state == :advertising
        # return to scanning if packet is absent
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
            node.channel = 37

            node.packet = nothing

            node.transmitting = false
            node.state = :scanning
            node.event_start = model.tick

            return
        end
    end
end

distance(a::Tuple{Int, Int}, b::Tuple{Int, Int}) = sqrt((a[1] - b[1]) ^ 2 + (a[2] - b[2]) ^ 2)

function model_step!(model)
    if rand() < 0.007 && model[1].state == :scanning
        model[1].packet = Packet(model.tick, 1, rand(2:nagents(model)), 8)
        model.produced += 1
    end

    for src in allagents(model)
        src.transmitting || continue

        for dst in allagents(model)
            if distance(src.pos, dst.pos) < 50 && dst.state == :scanning && dst.channel == src.channel
                if rand() < 0.90
                    dst.packet = deepcopy(src.packet)
                end
            end
        end
    end

    model.tick += 1
end

function plotgrid(model::AgentBasedModel)
    ac(n::Node) = n.mode === :relay ? :orange : :darkblue
    plotabm(model; ac, aspect_ratio=:equal, size = (600, 600), yflip=true, showaxis=false)
end

function plotgraph(model::AgentBasedModel)
    g = SimpleGraph(nagents(model))

    for src in allagents(model), dst in allagents(model)
        src.mode == :relay || continue

        if distance(src.pos, dst.pos) < 50
            add_edge!(g, src.id, dst.id)
        end
    end

    agents = sort(collect(allagents(model)), by=a -> a.id)
    xs = map(a -> a.pos[1], agents)
    ys = map(a -> a.pos[2], agents)
    cs = map(a -> a.mode == :relay ? "orange" : "darkblue", agents)

    gplot(g, xs, ys; nodefillc = cs)
end

"""
Generates a random graph and returns the adjacency matrix and xy-coordinates of nodes
"""
function random_graph(; dims = (128, 128), n_nodes = 64, power_distance = 50)
    xys = [(1, 1); [(rand(1:dims[1]), rand(1:dims[2])) for _ in 1:n_nodes-2]; (dims)]
    g = SimpleGraph(n_nodes)

    for src in eachindex(xys), dst in src+1:n_nodes
        if distance(xys[src], xys[dst]) <= power_distance
            add_edge!(g, src, dst)
        end
    end

    Matrix(adjacency_matrix(g)), xys
end

"""
Accepts [0, 1, 0...] where 1 means that the node is a relay, [(Int, Int)...] as nodes' positions
"""
function initialize_model(roles::Vector{Int}, positions::Vector{Tuple{Int, Int}})
    length(roles) == length(positions) || throw(ArgumentError("Both arguments must be of equal length"))

    properties = Dict(:tick => 0, :received => 0, :produced => 0)
    space = GridSpace((maximum(first.(positions)), maximum(last.(positions))), moore = true)
    model = ABM(Node, space; properties)

    relay_args = (mode = :relay, t_interpdu = 5, t_scan_interval = 20, t_back_off_delay = 1)
    sink_args = (mode = :ignore, t_interpdu = 0, t_scan_interval = 20, t_back_off_delay = 0)

    for idx = eachindex(roles)
        args = roles[idx] == 1 ? relay_args : sink_args
        node = Node(idx, positions[idx]; args...)

        add_agent!(node, positions[idx], model)
    end

    model
end

"""
Conducts an experiment given the model and the number of steps in milliseconds
"""
function start_model(model::AgentBasedModel, steps = 60000)
    _, dfm = run!(model, agent_step!, model_step!, steps; when_model = [steps], mdata = [:produced, :received])

    dfm[:produced][1], dfm[:received][1]
end
end
