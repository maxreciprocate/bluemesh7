export BlueMesh7Env, reset!, get_state, get_reward

mutable struct BlueMesh7Env
    positions :: Vector{NTuple{2, Int}}
    model :: AgentBasedModel
    rewards :: Vector{Float64}
end

function BlueMesh7Env(dims = (100, 100), n = 64)
    positions = generate_positions(; dims, n)
    model = initialize_mesh(positions, zeros(Int, n))

    BlueMesh7Env(positions, model, zeros(n))
end

function BlueMesh7Env(positions::Vector{NTuple{2, Int}})
    n = length(positions)
    model = initialize_mesh(positions, zeros(Int, n))

    BlueMesh7Env(positions, model, zeros(Float64, n))
end

function (env::BlueMesh7Env)(moves::Vector{Int}, eval=false)
    if !eval
        for idx in eachindex(moves)
            env.model[idx].role = moves[idx] == 1 ? :relay : :sink
        end
    end

    step!(env.model, agent_step!, model_step!)

    # no need for rewards if no one is going to get them
    eval && return

    fill!(env.rewards, 0.0)

    for id in env.model.reward_plate
        env.rewards[id] += 100.0
    end

    for packet in env.model.packets
        if packet.done || env.model.tick - packet.time_start > 1000
            continue
        end

        for x in env.model.packet_xs[packet.seq]
            env.rewards[x] -= 1.0
        end
    end
end

function get_state(env::BlueMesh7Env)
    # count of neighbours which pass our sensitivity threshold
    nbours = count.(env.model.rssi_map[:, id] .> mesh.scanner_sensitivity for id = 1:env.model.n)

    # some information about the neighbourhood
    # about recent packet history
    # about the current packet
    nbours
end

function get_reward(env::BlueMesh7Env)
    env.rewards
end

import Base.display
display(env::BlueMesh7Env) = plotgraph(env.model)
