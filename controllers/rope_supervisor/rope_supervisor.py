# rope_supervisor.py
# Supervisor that creates a visible flat rope between ROSBOT and TURTLE,
# and applies a spring-damper force to the TURTLE so that ROSBOT "pulls" it.

from controller import Supervisor
import math

SUP = Supervisor()
timestep = int(SUP.getBasicTimeStep())

# Configuration (tweak these)
ROPE_REST_LENGTH = 0.6   # desired slack length (m); if robots further, spring pulls
K = 80.0                 # spring constant
C = 10.0                 # damping coefficient
MAX_FORCE = 100.0        # clamp the applied force magnitude

# look up nodes by DEF (you added these)
ros_node = SUP.getFromDef('ROSBOT')
turtle_node = SUP.getFromDef('TURTLE')
rope_node = SUP.getFromDef('ROPE_VIS')

if ros_node is None:
    print("ERROR: ROSBOT not found. Make sure you added: DEF ROSBOT Rosbot {...}")
if turtle_node is None:
    print("ERROR: TURTLE not found. Make sure you added: DEF TURTLE TurtleBot3Burger {...}")
if rope_node is None:
    print("ERROR: ROPE_VIS not found. Make sure you pasted the Transform snippet for ROPE_VIS.")

# Helper math
def vec_sub(a, b): return [a[i]-b[i] for i in range(3)]
def vec_add(a, b): return [a[i]+b[i] for i in range(3)]
def vec_dot(a, b): return sum(a[i]*b[i] for i in range(3))
def vec_scale(a, s): return [a[i]*s for i in range(3)]
def vec_len(a): return math.sqrt(vec_dot(a,a))
def vec_norm(a):
    l = vec_len(a)
    return [0,0,0] if l == 0 else [a[i]/l for i in range(3)]

# Fields we'll update for the rope Transform
rope_translation_field = rope_node.getField('translation')
rope_rotation_field = rope_node.getField('rotation')
rope_scale_field = rope_node.getField('scale')

print("Rope supervisor started. Press Play ▶ in Webots.")

while SUP.step(timestep) != -1:
    # get absolute positions (center) of the two robots
    ros_pos = ros_node.getPosition()     # [x,y,z]
    turtle_pos = turtle_node.getPosition()

    ros_anchor_offset = [0.0, 0.0, 0.05]    
    turtle_anchor_offset = [0.0, 0.0, 0.12]  

    # --- Correct anchor retrieval ---#
    ros_anchor_node = SUP.getFromDef('ROS_ANCHOR')
    turtle_anchor_node = SUP.getFromDef('TURTLE_ANCHOR')

    if ros_anchor_node:
        ros_anchor = ros_anchor_node.getPosition()
    else:
        ros_anchor = vec_add(ros_pos, [0.0, 0.0, 0.05])  # fallback

    if turtle_anchor_node:
        turtle_anchor = turtle_anchor_node.getPosition()
    else:
        turtle_anchor = vec_add(turtle_pos, [0.0, 0.0, 0.12])  # fallback

    # vector from turtle -> ros
    v = vec_sub(ros_anchor, turtle_anchor)
    dist = vec_len(v)
    if dist == 0:
        dir = [0,0,0]
    else:
        dir = vec_scale(v, 1.0/dist)

    # get linear velocities (6-vector) and extract linear part
    # Node.getVelocity() returns [vx,vy,vz,wx,wy,wz]
    ros_vel6 = ros_node.getVelocity()
    turtle_vel6 = turtle_node.getVelocity()
    ros_lin_vel = ros_vel6[0:3] if ros_vel6 is not None else [0,0,0]
    turtle_lin_vel = turtle_vel6[0:3] if turtle_vel6 is not None else [0,0,0]
    rel_vel = vec_sub(ros_lin_vel, turtle_lin_vel)
    rel_speed_along_dir = vec_dot(rel_vel, dir)

    # Spring-damper law (only pull when stretched beyond rest length)
    stretch = dist - ROPE_REST_LENGTH
    spring_force = 0.0
    if stretch > 0.0:
        spring_force = K * stretch
    damping_force = C * rel_speed_along_dir
    force_mag = spring_force + damping_force

    # clamp
    if force_mag > MAX_FORCE:
        force_mag = MAX_FORCE

    # force applied to the Turtle toward the ROSbot anchor
    force_vec = vec_scale(dir, force_mag)

    # apply force to the Turtle (in world coords)
    # negative sign: apply force pointing to ROSbot (turtle should be pulled)
    if turtle_node is not None:
        # apply at turtle center (relative=False)
        turtle_node.addForce(force_vec, False)

    # (optional) apply opposite small force to rosbot for realism
    # comment-out if you don't want the rosbot reaction:
    # if ros_node is not None:
    #     ros_node.addForce(vec_scale(force_vec, -1.0), False)

    # --- Update visible rope Transform ---
    # midpoint
    mid = [(ros_anchor[i] + turtle_anchor[i]) * 0.5 for i in range(3)]
    # set translation (y is up in Webots; keep same height as anchors)
    rope_translation_field.setSFVec3f(mid)

    # set scale.x to the length (rope box base length is 1)
    if rope_scale_field:
        # avoid zero-length scaling
        sx = dist if dist > 0.001 else 0.001
        rope_scale_field.setSFVec3f([sx, 1.0, 1.0])

    # compute rotation to align the rope's local X axis with direction vector
    # default rope points along +X in its local coordinates
    # find rotation axis = cross([1,0,0], dir)
    default_axis = [1.0, 0.0, 0.0]
    cross = [ default_axis[1]*dir[2] - default_axis[2]*dir[1],
              default_axis[2]*dir[0] - default_axis[0]*dir[2],
              default_axis[0]*dir[1] - default_axis[1]*dir[0] ]
    dot = vec_dot(default_axis, dir)
    # clamp numerical
    dot = max(-1.0, min(1.0, dot))
    angle = math.acos(dot)
    # if dir is (near) opposite, pick an arbitrary perpendicular axis
    if vec_len(cross) < 1e-6:
        # dir is parallel or anti-parallel
        # if anti-parallel (dot ~ -1), rotate 180 deg around any perpendicular axis
        if dot < 0:
            axis = [0,0,1]
            angle = math.pi
        else:
            axis = [0,0,1]
            angle = 0.0
    else:
        axis = vec_norm(cross)
    # set rotation
    rope_rotation_field.setSFRotation([axis[0], axis[1], axis[2], angle])

# end while
