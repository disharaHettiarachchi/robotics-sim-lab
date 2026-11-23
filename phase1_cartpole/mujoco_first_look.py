"""
mujoco_first_look.py

Method step 4, part 1: load cartpole.xml and check it matches the hand model
BEFORE simulating anything. Two checks:
  1. masses: does MuJoCo see M and m where you expect?
  2. angle convention: for a given (x, theta), is the bob where r_p(q) says?

Run:  python phase1_cartpole/mujoco_first_look.py
"""
import os
import numpy as np
import mujoco

# Build the path to cartpole.xml from this file's own folder, so the script
# works no matter which folder you run it from.
HERE = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "cartpole.xml"))

# MuJoCo splits things in two:
#   model (MjModel): everything constant (masses, lengths, joint axes, timestep)
#   data  (MjData) : everything that changes (qpos, qvel, ctrl, time, positions)
data = mujoco.MjData(model)

l = 0.5   # must match the length you put in the XML

# ---------------------------------------------------------------
# Check 1: sizes and masses
# ---------------------------------------------------------------
# nq = number of position coordinates, nv = number of velocities,
# nu = number of actuators. For the cart-pole expect 2, 2, 1.
print(f"nq = {model.nq}, nv = {model.nv}, nu = {model.nu}")
for i in range(model.nbody):           # body 0 is always 'world' (mass 0)
    print(f"body '{model.body(i).name}': mass = {model.body_mass[i]}")

# ---------------------------------------------------------------
# Check 2: angle convention
# ---------------------------------------------------------------
# Set the joint positions by hand, then call mj_forward. mj_forward computes
# all world positions from qpos WITHOUT advancing time (no simulation step).
print("\n theta  | MuJoCo bob [x, y, z]        | expected from your r_p")
for th in [0.0, 0.3, -0.3, np.pi / 2]:
    mujoco.mj_resetData(model, data)          # back to qpos = 0, qvel = 0
    x = 0.2
    data.joint('slider').qpos[0] = x          # named access: the slider's qpos is your x
    data.joint('hinge').qpos[0] = th          # the hinge's qpos is your theta
    mujoco.mj_forward(model, data)

    bob = data.geom('bob').xpos               # bob centre in world coords [x, y, z]

    # TODO: your pole-mass position from day 1, as [horizontal, 0, vertical].
    # (MuJoCo's vertical axis is z; y is the out-of-plane direction, always 0 here.)
    expected = np.array([x+l*np.sin(th), 0.0, l*np.cos(th)])

    print(f"{th:6.3f} | {np.round(bob, 4)} | {np.round(expected, 4)}")
