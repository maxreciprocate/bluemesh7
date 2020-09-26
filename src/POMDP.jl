using Agents

export BlueMesh7Env, reset!, get_state, get_reward

mutable struct BlueMesh7Env
    positions :: Vector{NTuple{2, Int}}
    adjacency :: Array{Int, 2}
    rewards :: Vector{Float64}
    mesh :: AgentBasedModel
end

function BlueMesh7Env(dims = (100, 100), n_nodes = 64)
    positions, adjacency = generate_graph(; dims, n_nodes)
    mesh = initialize_mesh(positions, zeros(Int, length(positions)))

    BlueMesh7Env(positions, adjacency, zeros(n_nodes), mesh)
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

    step!(env.mesh, BlueMesh7.agent_step!, BlueMesh7.model_step!)

    # calculate rewards
    fill!(env.rewards, 0.0)

    for id in 1:length(env.positions)
        if id in env.mesh.reward_plate
            env.rewards[id] += 100.0
        end
    end

    for packet in env.mesh.packets
        if packet.done || env.mesh.tick - packet.time_start > 1000
            continue
        end

        for x in env.mesh.packet_xs[packet.seq]
            env.rewards[x] -= 1.0
        end
    end
end

function get_state(env::BlueMesh7Env, id::Int)
    agent = env.mesh[id]

    view(env.adjacency, :, id)
end

function get_reward(env::BlueMesh7Env, id::Int)
    env.rewards[id]
end

import Base.display
display(env::BlueMesh7Env) = plotgraph(env.mesh)
