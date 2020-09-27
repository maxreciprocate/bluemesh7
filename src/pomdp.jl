export BlueMesh7Env, reset!, get_state, get_reward

mutable struct BlueMesh7Env
    positions :: Vector{NTuple{2, Int}}
    rewards :: Vector{Float64}
    mesh :: AgentBasedModel
end

function BlueMesh7Env(dims = (100, 100), n = 64)
    positions = generate_positions(; dims, n)
    mesh = initialize_mesh(positions, zeros(Int, n))

    BlueMesh7Env(positions, zeros(n), mesh)
end

function BlueMesh7Env(positions::Vector{NTuple{2, Int}})
    n = length(positions)

    BlueMesh7Env(positions, zeros(Float64, n), initialize_mesh(positions, zeros(Int, n)))
end

function reset!(env::BlueMesh7Env)
    env.mesh = initialize_mesh(env.positions, zeros(Int, length(env.positions)))
    fill!(env.rewards, 0.0)
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

    view(env.mesh.rssi_map, :, id), Int(agent.packet_seq > 0)
end

function get_reward(env::BlueMesh7Env, id::Int)
    env.rewards[id]
end

import Base.display
display(env::BlueMesh7Env) = plotgraph(env.mesh)
