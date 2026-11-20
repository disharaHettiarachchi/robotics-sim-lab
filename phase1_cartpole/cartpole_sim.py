"""
cartpole_sim.py

Method step 3: simulate YOUR hand-derived cart-pole model with SciPy solve_ivp.

State vector (positions first, then velocities):
    s = [x, theta, xdot, thetadot]
This ordering makes the linearized A matrix have the block form
[[0, I], [*, *]] later in step 6, which is the standard convention.

You fill in the TODOs. Run:  python cartpole_sim.py
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ---------------------------------------------------------------
# Parameters (SI units). A dict keeps them in one place.
# ---------------------------------------------------------------
P = dict(M=1.0,    # cart mass [kg]
         m=0.2,    # pole tip mass [kg]
         l=0.5,    # rod length [m]
         g=9.81)   # gravity [m/s^2]


def force(t, s):
    """Horizontal force F on the cart at time t and state s.
    Zero for now (free motion). The LQR controller will go here in step 6."""
    return 0.0


def dynamics(t, s, P):
    """Right-hand side for solve_ivp: returns sdot = f(t, s).

    solve_ivp calls this many times per step. It must take (t, s, ...)
    in that order and return the 4 derivatives in the same order as s.
    """
    x, th, xd, thd = s                       # unpack the state
    M, m, l, g = P['M'], P['m'], P['l'], P['g']
    F = force(t, s)

    # TODO: your simplified xddot and thetaddot formulas.
    # Use np.sin / np.cos (NumPy), not sp.sin (SymPy).
    xdd = (F + m*l*thd**2*np.sin(th)-m*g*np.sin(th)*np.cos(th))/(M + m*np.sin(th)**2)  # TODO: horizontal acceleration of the cart
    thdd = ((M+m)*g*np.sin(th) - F*np.cos(th) - m*l*np.sin(th)*np.cos(th)*thd**2)/(l*(M + m*np.sin(th)**2))  # TODO: angular acceleration of the pole

    return [xd, thd, xdd, thdd]              # [xdot, thetadot, xddot, thetaddot]


def energy(s, P):
    """Total energy E = T + V.

    Works for one state (shape (4,)) or a whole trajectory (shape (4, N)),
    because unpacking a (4, N) array gives four length-N rows and NumPy
    applies sin, cos, *, + element by element.
    """
    x, th, xd, thd = s
    M, m, l, g = P['M'], P['m'], P['l'], P['g']
    T = 0.5 * (M + m) * xd**2 + 0.5 * m * (l * thd)**2 + m*l*xd*thd*np.cos(th)  # TODO kinetic energy (same T as your notes)
    V = m*g*l*np.cos(th)              # TODO: potential energy
    return T + V


def momentum_x(s, P):
    """Horizontal momentum p_x = dL/dxdot. Conserved when F = 0."""
    x, th, xd, thd = s
    M, m, l, g = P['M'], P['m'], P['l'], P['g']
    return (M + m) * xd + m * l * thd * np.cos(th)  # TODO


def simulate(s0, t_end=10.0):
    """Integrate from initial state s0 for t_end seconds."""
    t_eval = np.linspace(0.0, t_end, 2001)   # times where we want output saved
    sol = solve_ivp(
        dynamics, (0.0, t_end), s0,
        args=(P,),          # extra arguments passed on to dynamics(t, s, P)
        t_eval=t_eval,
        method='RK45',      # same family as MATLAB's ode45
        rtol=1e-10, atol=1e-10,  # tight tolerances: we are testing conservation,
                                 # so numerical error must be tiny
    )
    if not sol.success:
        raise RuntimeError(sol.message)
    return sol


if __name__ == "__main__":
    cases = {
        "upright, theta0 = 0.1": [0.0, 0.1, 0.0, 0.0],
        "hanging, theta0 = pi + 0.1": [0.0, np.pi + 0.1, 0.0, 0.0],
    }

    for name, s0 in cases.items():
        sol = simulate(np.array(s0))
        t, S = sol.t, sol.y          # S has shape (4, N): one row per state variable
        E = energy(S, P)
        p = momentum_x(S, P)

        print(f"{name}:  max |E - E0| = {np.max(np.abs(E - E[0])):.2e} J,"
              f"  max |p - p0| = {np.max(np.abs(p - p[0])):.2e} kg m/s")

        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(7, 9))
        fig.suptitle(name)
        ax[0].plot(t, S[0]);        ax[0].set_ylabel("x [m]")
        ax[1].plot(t, S[1]);        ax[1].set_ylabel("theta [rad]")
        ax[2].plot(t, E - E[0]);    ax[2].set_ylabel("E - E0 [J]")
        ax[3].plot(t, p - p[0]);    ax[3].set_ylabel("p_x - p_x0")
        ax[3].set_xlabel("t [s]")
        fig.tight_layout()

    plt.show()
