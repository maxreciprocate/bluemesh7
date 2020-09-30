mutable struct Packet
    seq :: Int
    src :: UInt16
    dst :: UInt16
    ttl :: UInt8

    time_start :: Int
    time_done :: Int
    done :: Bool
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
    event_start :: Int = rand(0:50)

    # whether the device is currently sending data (this removes the need for redundant allocations)
    transmitting :: Bool = false

    # the length of the interval between transmissions on different channels (ms)
    t_interpdu :: UInt = 1

    # the length of the interval between scanning on different channels (ms)
    t_scan_interval :: UInt = 20

    # the length of the interval before advertising the received packet
    t_back_off_delay :: UInt = 10

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

    # transmitting power
    tx_power :: Float64 = 1e-3

    # the reference to the current holding packet
    packet_seq :: Int = 0

    # own ttl for the current packet
    packet_ttl :: UInt8 = 0
end

Base.@kwdef mutable struct Source <: AbstractAgent
    id :: Int
    pos :: Tuple{Int, Int}

    role :: Symbol = :source
    state :: Symbol = :advertising

    channel :: UInt8 = 37
    event_start :: UInt = 0
    transmitting :: Bool = false

    t_interpdu :: UInt = 1

    # the number of extra transmissions for a packet originating from the source
    n_og_transmit_count :: UInt = 2

    # the length of the delay between additional transmissions of the original packet (ms)
    t_og_transmit_delay :: UInt = 10

    # the range of the additional random component to delay between additional original transmissions
    rt_og_random_delay :: UnitRange{Int} = 0:20

    # the current number of extra transmissions left
    n_transmit_left :: UInt = 0

    # transmitting power
    tx_power :: Float64 = 1e-3

    packet_seq :: Int = 0
    packet_ttl :: UInt8 = 0
end
