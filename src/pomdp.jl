export BlueMesh7Env, reset!, get_state, get_reward

mutable struct BlueMesh7Env
    positions :: Vector{NTuple{2, Int}}
    rewards :: Vector{Float64}
    nbours :: Vector{Int}
    mesh :: AgentBasedModel
end

function BlueMesh7Env(dims = (100, 100), n = 64)
    positions = generate_positions(; dims, n)
    mesh = initialize_mesh(positions, zeros(Int, n))

    nbours = [count(mw -> mw > mesh.scanner_sensitivity, @view mesh.rssi_map[id + 1, 2:end]) + 1 for id = 1:n]

    BlueMesh7Env(positions, zeros(n), nbours, mesh)
end

function BlueMesh7Env(positions::Vector{NTuple{2, Int}})
    n = length(positions)

    mesh = initialize_mesh(positions, zeros(Int, n))
    nbours = [count(mw -> mw > mesh.scanner_sensitivity, @view mesh.rssi_map[id + 1, 2:end]) for id = 1:n]

    BlueMesh7Env(positions, zeros(Float64, n), nbours, mesh)
end

function (env::BlueMesh7Env)(moves::Vector{Int}, eval=false)
    if !eval
        for idx in eachindex(moves)
            env.mesh[idx].role = moves[idx] == 1 ? :relay : :sink
        end
    end

    step!(env.mesh, agent_step!, model_step!)

    # no need for rewards if no one is going to get them
    eval && return

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
    env.nbours[id], Int(env.mesh[id].packet_seq > 0) + 1
end

function get_reward(env::BlueMesh7Env, id::Int)
    env.rewards[id]
end

import Base.display
display(env::BlueMesh7Env) = plotgraph(env.mesh)
