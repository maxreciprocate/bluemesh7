using Agents

export BlueMesh7Env, reset!, get_state, get_reward

mutable struct BlueMesh7Env
    positions :: Vector{NTuple{2, Int}}
    adjacency :: Array{Int, 2}
    mesh :: AgentBasedModel
end

function BlueMesh7Env(dims = (100, 100), n_nodes = 64)
    positions, adjacency = generate_graph(; dims, n_nodes)
    mesh = initialize_mesh(positions, zeros(Int, length(positions)))

    BlueMesh7Env(positions, adjacency, mesh)
end

function BlueMesh7Env(positions::Vector{NTuple{2, Int}}, adjacency::Array{Int, 2})
    BlueMesh7Env(positions, adjacency, initialize_mesh(positions, zeros(Int, length(positions))))
end

function reset!(env::BlueMesh7Env)
    env.mesh = initialize_mesh(env.positions, zeros(Int, length(env.positions)))
end

function (env::BlueMesh7Env)(moves::Vector{Int})
    for idx in eachindex(moves)
        env.mesh[idx].role = moves[idx] == 1 ? :relay : :sink
    end

    step!(env.mesh, agent_step!, model_step!)
end

function get_state(env::BlueMesh7Env, id::Int)
    agent = env.mesh[id]

    last = if agent.packet != nothing
        agent.packet.last
    else
        0
    end

    view(env.adjacency, :, id), last
end

function get_reward(env::BlueMesh7Env, id::Int)
    r = 0

    if id in env.mesh.reward_plate
        r += 100
    end

    for touches in values(env.mesh.travelling_packets)
        if id in touches
            r -= 1
        end
    end

    r
end

import Base.display
display(env::BlueMesh7Env) = plotgraph(env.mesh)
