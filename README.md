# RigidBodyDynamicsTutorial

WIP - below are just my notes on top of what the Julia Robotics team has done.

Maybe should do a stream alongside a blog post?

https://github.com/JuliaRobotics/RigidBodyDynamics.jl/tree/master/examples

## Double pendulum

Specify moment of intertia (angular mass) about joint axis, center of mass w.rt. to joint axis and mass

The world is a rigid body

```julia
world = RigidBody{Float64}("world")
doublependulum = Mechanism(world; gravity = SVector(0, 0, g))
```


Can do coordinate transforms relative to each joint or relative to world

Can also load in urdf as opposed to creating our own data structures


```julia
urdf = joinpath(dirname(pathof(RigidBodyDynamics)), "..", "test", "urdf", "Acrobot.urdf")
parse_urdf(urdf)
```

Can act on configuration by using set configuration and set velocity

```julia
set_configuration!(state, shoulder, 0.3)
set_configuration!(state, elbow, 0.4)
set_velocity!(state, shoulder, 1.)
set_velocity!(state, elbow, 2.);
```

Can do kinematics -> make translations
Or dynamics -> Put foreces

WHen we simulate a mechanism we get back 3 vectors
* Time
* Joint configurations
* Velocities


## Closed loop simulation

Controllers are pretty trivial to create on the joints, heres one that just applies a sign wave

``` julia
shoulder, elbow = joints(mechanism)
function simple_control!(torques::AbstractVector, t, state::MechanismState)
    torques[velocity_range(state, shoulder)] .= -1 .* velocity(state, shoulder)
    torques[velocity_range(state, elbow)] .= 10 * sin(t)
end

```

Visualizations can be done using MeshCatMechanisms

```julia
#NBSKIP
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf))
open(mvis);
MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);
```

## Four bar linkage forward kinematics

Visualization and simulation are trivial

The construction is a bit convoluted here though. Theres a coordinate transform that happens before each bone is attached

```julia
# link2 and joint2
joint2 = Joint("joint2", Revolute(axis))
inertia2 = SpatialInertia(CartesianFrame3D("inertia2_centroidal"), moment=I_2*axis*axis', com=zero(SVector{3, T}), mass=m_2)
link2 = RigidBody(inertia2)
before_joint2_to_after_joint1 = Transform3D(frame_before(joint2), frame_after(joint1), SVector(l_1, 0., 0.))
c2_to_joint = Transform3D(inertia2.frame, frame_after(joint2), SVector(c_2, 0, 0))
attach!(fourbar, link1, link2, joint2, joint_pose = before_joint2_to_after_joint1, successor_pose = c2_to_joint)
```


## Jacobian IK 
Sam Buss wrote the tutorial for this  - should reach out and definitely read his tutorial
Has code for inverse Jacobian update

API is as simple as 

```julia
jacobian_transpose_ik!(state, body, point, desired_tip_location)
```

Can even have more complicated goals, instead of just being a point - can have a trajectory like a circular trajectory

Need to reread this

## TODO: Derivatives and gradients using ForwardDiff