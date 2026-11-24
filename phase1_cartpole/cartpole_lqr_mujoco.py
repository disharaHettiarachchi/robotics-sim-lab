"""
cartpole_lqr_mujoco.py

Method step 6b, part 2: run the SAME LQR gain K on
  (1) your own nonlinear model (solve_ivp), and
  (2) MuJoCo (cartpole.xml),
for two experiments:
  A. recovery from an initial tilt, theta0 = 0.1 rad
  B. push recovery: pole balanced at rest, then a sideways push on the bob

Code-light phase: the plumbing is written for you. Your parts are marked
TODO (3 short pieces). Read every line before running.
Run:  python phase1_cartpole/cartpole_lqr_mujoco.py
"""
import os
import numpy as np
import matplotlib.pyplot as plt
import control as ct
import mujoco
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------
# Parameters (must match cartpole.xml and cartpole_sim.py)
# ---------------------------------------------------------------
M, m, l, g = 1.0, 0.2, 0.5, 9.81

# ---------------------------------------------------------------
# LQR design (same as cartpole_lqr.py baseline)
# ---------------------------------------------------------------
A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, -m * g / M, 0, 0],
              [0, (M + m) * g / (l * M), 0, 0]])
B = np.array([[0], [0], [1 / M], [-1 / (l * M)]])
Q = np.diag([1.0, 10.0, 1.0, 1.0])
R = np.array([[0.1]])
K, _, E = ct.lqr(A, B, Q, R)          # K has shape (1, 4); "_" discards the Riccati matrix
print("K =", K.round(3))


def controller(s):
    """Cart force F for state s = [x, theta, xdot, thetadot]."""
    # TODO 1 (1 line): the LQR law. Return a plain float.
    return float((-K @ s)[0])


# ---------------------------------------------------------------
# The push: a horizontal force on the bob for a short time
# ---------------------------------------------------------------
PUSH_FORCE = 5.0      # [N], positive = toward +x
PUSH_START = 1.0      # [s]
PUSH_DURATION = 0.1   # [s]  -> impulse = 5 * 0.1 = 0.5 N s


def push_force(t, push_on):
    """Horizontal push on the bob at time t (0 outside the push window)."""
    if push_on and PUSH_START <= t < PUSH_START + PUSH_DURATION:
        return PUSH_FORCE
    return 0.0


# ---------------------------------------------------------------
# 1. Your own model, written as D(q) qddot + C qdot + G = B F + Q_push
# ---------------------------------------------------------------
def generalized_push(s, Fp):
    """Generalized force [Q_x, Q_theta] produced by a horizontal force Fp on the bob.

    TODO 2 (physics, 2 lines): use Q = J_p^T f, where f = [Fp, 0] is the force on
    the bob (horizontal, vertical) and J_p is the pole-mass Jacobian from day 2.
    Think first: which way does a +x push on the bob turn the pole?
    """
    x, th, xd, thd = s
    Qx = Fp
    Qth = Fp*l*np.cos(th)
    return np.array([Qx, Qth])


def dynamics(t, s, push_on):
    """sdot for your model. Solves D qddot = B F + Q_push - C qdot - G numerically,
    so the push enters exactly as a generalized force (no re-derivation needed)."""
    x, th, xd, thd = s
    F = controller(s)
    D = np.array([[M + m, m * l * np.cos(th)],
                  [m * l * np.cos(th), m * l ** 2]])
    Cqd = np.array([-m * l * thd ** 2 * np.sin(th), 0.0])   # C(q, qdot) qdot
    G = np.array([0.0, -m * g * l * np.sin(th)])
    rhs = np.array([F, 0.0]) + generalized_push(s, push_force(t, push_on)) - Cqd - G
    xdd, thdd = np.linalg.solve(D, rhs)                     # solve, don't invert
    return [xd, thd, xdd, thdd]


def run_model(s0, t, push_on):
    # max_step keeps the solver from stepping over the 0.1 s push window
    sol = solve_ivp(dynamics, (t[0], t[-1]), s0, t_eval=t, args=(push_on,),
                    max_step=0.005, rtol=1e-8, atol=1e-8)
    return sol.y.T                       # shape (N, 4)


# ---------------------------------------------------------------
# 2. MuJoCo
# ---------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "cartpole.xml"))
data = mujoco.MjData(model)
dt = model.opt.timestep
POLE_ID = model.body('pole').id          # index of the pole body, for the push


def mujoco_state():
    """Current MuJoCo state in YOUR order [x, theta, xdot, thetadot]."""
    # TODO 3 (1 line): build the 4-vector from data.qpos and data.qvel.
    #   qpos = [x, theta], qvel = [xdot, thetadot]. Return a new NumPy array.
    x = data.joint('slider').qpos 
    th = data.joint('hinge').qpos
    xdot = data.joint('slider').qvel
    thetadot = data.joint('hinge').qvel
    return np.hstack([x, th, xdot, thetadot])


def run_mujoco(s0, t, push_on):
    mujoco.mj_resetData(model, data)
    data.qpos[:] = s0[:2]
    data.qvel[:] = s0[2:]
    S = np.zeros((len(t), 4))
    S[0] = s0
    for k in range(len(t) - 1):
        s = mujoco_state()
        data.ctrl[0] = controller(s)     # state feedback, held for the whole step (ZOH)
        # External force on the pole body, in world coordinates [fx, fy, fz, tx, ty, tz],
        # applied at the body's centre of mass (the bob, set by <inertial> in the XML).
        data.xfrc_applied[POLE_ID, :] = 0.0
        data.xfrc_applied[POLE_ID, 0] = push_force(data.time + dt / 2, push_on)  # midpoint sample
        mujoco.mj_step(model, data)
        S[k + 1] = mujoco_state()
    return S


# ---------------------------------------------------------------
# 3. Run both experiments and compare
# ---------------------------------------------------------------
if __name__ == "__main__":
    t = np.arange(0, 6.0 + dt / 2, dt)    # same time grid for both simulators
    experiments = {
        "A: recovery from theta0 = 0.1 rad": (np.array([0.0, 0.1, 0.0, 0.0]), False),
        "B: push on the bob (5 N for 0.1 s at t = 1 s)": (np.zeros(4), True),
    }
    for name, (s0, push_on) in experiments.items():
        S_md = run_model(s0, t, push_on)
        S_mj = run_mujoco(s0, t, push_on)
        F_md = np.array([controller(s) for s in S_md])   # same function the sims used
        F_mj = np.array([controller(s) for s in S_mj])
        err = np.abs(S_md - S_mj)
        print(f"{name}\n  max |x err| = {err[:, 0].max():.2e} m, "
              f"max |theta err| = {err[:, 1].max():.2e} rad, "
              f"peak |F| = {np.abs(F_mj).max():.2f} N, "
              f"max |theta| = {np.abs(S_mj[:, 1]).max():.3f} rad, "
              f"max |x| = {np.abs(S_mj[:, 0]).max():.3f} m")

        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(7, 9))
        fig.suptitle(name)
        for i, lab in enumerate(["x [m]", "theta [rad]"]):
            ax[i].plot(t, S_md[:, i], label="my model")
            ax[i].plot(t, S_mj[:, i], "--", label="MuJoCo")
            ax[i].set_ylabel(lab)
        ax[0].legend()
        ax[2].plot(t, F_md, label="my model"); ax[2].plot(t, F_mj, "--", label="MuJoCo")
        ax[2].set_ylabel("F [N]")
        ax[3].semilogy(t, err[:, 0] + 1e-16, label="|x err|")
        ax[3].semilogy(t, err[:, 1] + 1e-16, label="|theta err|")
        ax[3].set_ylabel("error"); ax[3].set_xlabel("t [s]"); ax[3].legend()
        if push_on:
            for a in ax:
                a.axvspan(PUSH_START, PUSH_START + PUSH_DURATION, color="red", alpha=0.15)
        fig.tight_layout()
    plt.show()
