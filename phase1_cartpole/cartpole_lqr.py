"""
cartpole_lqr.py

Method step 6b, part 1: design an LQR controller from your linear model
(A, B from step 6a) and test it on your NONLINEAR model (cartpole_sim.py).

More of the code is yours this time. Each TODO says what to write and which
function to use. Run:  python phase1_cartpole/cartpole_lqr.py
Needs: pip install control   (python-control, already in your .venv)
"""
import numpy as np
import matplotlib.pyplot as plt
import control as ct                     # python-control: ct.ctrb, ct.lqr
from scipy.integrate import solve_ivp

import cartpole_sim
from cartpole_sim import dynamics, P

M, m, l, g = P['M'], P['m'], P['l'], P['g']

# ---------------------------------------------------------------
# 1. Linear model from step 6a
# ---------------------------------------------------------------
# TODO 1: write A (4x4) and B (4x1) as NumPy arrays using M, m, l, g.
#   State order s = [x, theta, xdot, thetadot].
#   B must be a COLUMN (shape (4, 1)), so write it as [[0], [0], [...], [...]].
#   python-control expects 2D arrays: a flat [0, 0, a, b] would be the wrong shape.
A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, -1*m*g/M, 0, 0], [0, g*(M+m)/(l*M), 0, 0]])
B = np.array([[0], [0], [1/M], [-1/(l*M)]])

# ---------------------------------------------------------------
# 2. Checks against your hand results
# ---------------------------------------------------------------
# TODO 2a: open-loop eigenvalues of A (np.linalg.eigvals). Expect 0, 0, +/-4.85.
L = np.linalg.eigvals(A)
print("Open-loop eigenvalues L:", L)

# TODO 2b: controllability matrix with ct.ctrb(A, B). Print its rank
#   (np.linalg.matrix_rank) and its determinant (np.linalg.det), and compare the
#   determinant with your hand formula -g**2 / (l**4 * M**4).
C = ct.ctrb(A, B)
print("Controllability matrix rank:", np.linalg.matrix_rank(C))
print("Controllability matrix determinant:", np.linalg.det(C))
print("Hand formula -g**2 / (l**4 * M**4):", -g**2 / (l**4 * M**4))
# ---------------------------------------------------------------
# 3. LQR design
# ---------------------------------------------------------------
# Q weights the state error, R weights the force. Larger Q entry = "care more
# about keeping that state small". Larger R = "use less force".
# Starting values: theta matters most, then x. Tune these in the experiments.
Q = np.diag([0.001, 10.0, 1.0, 1.0])     # weights on [x, theta, xdot, thetadot]
R = np.array([[0.1]])                  # weight on F (1x1 matrix)

# TODO 3a: call ct.lqr(A, B, Q, R). It returns three things: K (1x4 gain),
#   S (Riccati solution, ignore for now), E (closed-loop eigenvalues).
#   Print K.
K, P_ric, E = ct.lqr(A, B, Q, R)
print("LQR gain K:", K)
print("Closed-loop eigenvalues E:", E)
# TODO 3b: compute the eigenvalues of the closed-loop matrix (A - B K) yourself
#   with np.linalg.eigvals and check they match E. Are all real parts negative?
print("Closed-loop eigenvalues of A-BK:", np.linalg.eigvals(A - B @ K))
# ---------------------------------------------------------------
# 4. The controller
# ---------------------------------------------------------------
def controller(t, s):
    """Return the cart force F (a plain float) for state s.

    TODO 4: F = -K s. K has shape (1, 4) and s has shape (4,), so K @ s has
    shape (1,). Return it as a float (for example float((K @ s)[0]))."""
    return float((-K @ s)[0])


# ---------------------------------------------------------------
# 5. Closed-loop simulation on the NONLINEAR model
# ---------------------------------------------------------------
# TODO 5a: make dynamics() use your controller. dynamics() calls the
#   module-level function cartpole_sim.force(t, s), so replace that function
#   (same trick as in cartpole_compare.py).
cartpole_sim.force = controller

# TODO 5b: simulate with solve_ivp from s0 = [0, 0.1, 0, 0] for 6 s.
#   Use t_eval = np.linspace(0, 6, 1201) and args=(P,), rtol and atol 1e-8.
#   Store the time array in t and the states as S with shape (4, N) (that is sol.y).
s0 = [0, 0.1, 0, 0]
t = np.linspace(0, 6, 1201)
S = solve_ivp(dynamics, (t[0], t[-1]), s0, args=(P,), t_eval=t, rtol=1e-8, atol=1e-8).y

# TODO 5c: force history F_hist (length N): apply -K to every column of S at once.
#   Hint: (K @ S) has shape (1, N); .ravel() flattens it.
F_hist = (-K @ S).ravel()

# ---------------------------------------------------------------
# 6. Plot (given)
# ---------------------------------------------------------------
fig, ax = plt.subplots(3, 1, sharex=True, figsize=(7, 8))
fig.suptitle("LQR on the nonlinear cart-pole, theta0 = 0.1 rad")
ax[0].plot(t, S[0]);   ax[0].set_ylabel("x [m]")
ax[1].plot(t, S[1]);   ax[1].set_ylabel("theta [rad]")
ax[2].plot(t, F_hist); ax[2].set_ylabel("F [N]")
ax[2].set_xlabel("t [s]")
for a in ax:
    a.axhline(0, color='gray', lw=0.5)
fig.tight_layout()
plt.show()
