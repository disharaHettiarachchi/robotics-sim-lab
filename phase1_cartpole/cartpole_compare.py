"""
cartpole_compare.py

Method step 5: run the SAME experiment in two independent simulators and
compare them.
  - "model":  your hand-derived equations, integrated with solve_ivp (cartpole_sim.py)
  - "mujoco": cartpole.xml, which MuJoCo turns into equations by itself
Any difference bigger than numerical error is a bug to find and explain.

You fill in the TODO (the MuJoCo stepping loop). Run:
    python phase1_cartpole/cartpole_compare.py
"""
import os
import numpy as np
import matplotlib.pyplot as plt
import mujoco
from scipy.integrate import solve_ivp

import cartpole_sim                      # your step 3 file (same folder)
from cartpole_sim import dynamics, P     # reuse your equations and parameters

HERE = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "cartpole.xml"))
data = mujoco.MjData(model)
dt = model.opt.timestep                  # 0.001 s, set in the XML


def run_mujoco(s0, F_func, t_end):
    """Simulate cartpole.xml from state s0 = [x, theta, xdot, thetadot].

    Returns t (shape (N+1,)) and S (shape (N+1, 4)), one row per time step,
    columns in the same order as s0.
    """
    n_steps = int(round(t_end / dt))
    t = np.arange(n_steps + 1) * dt
    S = np.zeros((n_steps + 1, 4))

    # TODO 1: reset data, then set the initial state.
    mujoco.mj_resetData(model, data)          # back to qpos = 0, qvel = 0
    x = s0[0]
    th = s0[1]
    xd = s0[2]
    thd = s0[3]
    data.joint('slider').qpos[0] = x          
    data.joint('slider').qvel[0] = xd        
    data.joint('hinge').qpos[0] = th  
    data.joint('hinge').qvel[0] = thd 
    #   data.qpos holds [x, theta]  and  data.qvel holds [xdot, thetadot]
    #   (same order as the joints appear in the XML: slider first, then hinge).
    #   Hint: mujoco.mj_resetData(model, data), then assign slices, e.g. data.qpos[:] = ...

    S[0] = s0

    for k in range(n_steps):
        # TODO 2: set the cart force for this step. data.ctrl[0] is F (gear = 1).
        #   Use F_func(data.time). data.time is the current simulation time.
        data.ctrl[0] = F_func(t[k] + dt/2)  # set the force for this time step
        # TODO 3: advance one time step with mujoco.mj_step(model, data)
        mujoco.mj_step(model, data)
        # TODO 4: record the new state into row k + 1 of S
        #   (positions from data.qpos, velocities from data.qvel)
        S[k + 1] = [data.joint('slider').qpos[0], data.joint('hinge').qpos[0], data.joint('slider').qvel[0], data.joint('hinge').qvel[0]]

    return t, S


def run_model(s0, F_func, t_eval):
    """Your hand-derived model with solve_ivp, saved at the same times as MuJoCo."""
    cartpole_sim.force = lambda t, s: F_func(t)   # replace the force used inside dynamics()
    sol = solve_ivp(dynamics, (t_eval[0], t_eval[-1]), s0, args=(P,),
                    t_eval=t_eval, rtol=1e-10, atol=1e-10)
    return sol.y.T                                  # shape (N+1, 4), same layout as MuJoCo


if __name__ == "__main__":
    t_end = 10.0
    cases = {
        "upright, theta0 = 0.1, F = 0": ([0.0, 0.1, 0.0, 0.0], lambda t: 0.0),
        "hanging, theta0 = pi + 0.1, F = 0": ([0.0, np.pi + 0.1, 0.0, 0.0], lambda t: 0.0),
        "hanging at rest, F = 2 sin(3t)": ([0.0, np.pi, 0.0, 0.0], lambda t: 2.0 * np.sin(3.0 * t)),
    }

    for name, (s0, F_func) in cases.items():
        s0 = np.array(s0)
        t, S_mj = run_mujoco(s0, F_func, t_end)
        S_md = run_model(s0, F_func, t)
        err = S_md - S_mj

        print(f"{name}:  max |x error| = {np.max(np.abs(err[:, 0])):.2e} m,"
              f"  max |theta error| = {np.max(np.abs(err[:, 1])):.2e} rad")

        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(7, 8))
        fig.suptitle(name)
        ax[0].plot(t, S_md[:, 0], label="my model");  ax[0].plot(t, S_mj[:, 0], '--', label="MuJoCo")
        ax[0].set_ylabel("x [m]");  ax[0].legend()
        ax[1].plot(t, S_md[:, 1], label="my model");  ax[1].plot(t, S_mj[:, 1], '--', label="MuJoCo")
        ax[1].set_ylabel("theta [rad]")
        ax[2].semilogy(t, np.abs(err[:, 0]) + 1e-16, label="|x error|")   # +1e-16 avoids log(0)
        ax[2].semilogy(t, np.abs(err[:, 1]) + 1e-16, label="|theta error|")
        ax[2].set_ylabel("error");  ax[2].set_xlabel("t [s]");  ax[2].legend()
        fig.tight_layout()

    plt.show()
