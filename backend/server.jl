using HTTP, JSON3, LinearAlgebra

const G = 6.67430e-11

struct Body
    pos::Vector{Float64}
    vel::Vector{Float64}
    mass::Float64
    radius::Float64
end

# Compute accelerations on all bodies
function accelerations(bodies)
    n = length(bodies)
    accs = [zeros(3) for _ in 1:n]
    for i in 1:n
        for j in i+1:n
            r = bodies[j].pos - bodies[i].pos
            dist2 = dot(r, r) + 1e-9   # avoid /0
            f = G * bodies[i].mass * bodies[j].mass / dist2
            a = f / sqrt(dist2) * (r / sqrt(dist2))
            accs[i] += a / bodies[i].mass
            accs[j] -= a / bodies[j].mass
        end
    end
    return accs
end

# Velocity Verlet simulation
function simulate(bodies, t_max, dt)
    nsteps = Int(floor(t_max/dt))
    snapshots = Vector{Any}(undef, 0)

    accs = accelerations(bodies)
    for step in 0:nsteps
        # record snapshot every 100 steps
        if step % 100 == 0
            push!(snapshots, Dict(
                "time" => step*dt,
                "bodies" => [Dict(
                    "x"=>b.pos[1], "y"=>b.pos[2], "z"=>b.pos[3]
                ) for b in bodies]
            ))
        end

        # update positions
        for (i,b) in enumerate(bodies)
            b.pos .= b.pos .+ b.vel*dt .+ 0.5*accs[i]*dt^2
        end

        # update accelerations
        new_accs = accelerations(bodies)

        # update velocities
        for (i,b) in enumerate(bodies)
            b.vel .= b.vel .+ 0.5*(accs[i] .+ new_accs[i])*dt
        end

        accs = new_accs
    end

    return Dict("snapshots" => snapshots)
end

# HTTP server
HTTP.serve("127.0.0.1", 8080) do req::HTTP.Request
    if req.method == "POST" && req.target == "/simulate"
        data = JSON3.read(String(req.body))
        bodies = [Body(
            [d["position"]["x"], d["position"]["y"], d["position"]["z"]],
            [d["velocity"]["x"], d["velocity"]["y"], d["velocity"]["z"]],
            d["mass"], d["radius"]
        ) for d in data["bodies"]]

        sim = data["simulation"]
        result = simulate(bodies, sim["t_max"], sim["dt"])
        return HTTP.Response(200, JSON3.write(result))
    else
        return HTTP.Response(404, "Not found")
    end
end
