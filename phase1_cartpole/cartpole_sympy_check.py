"""
cartpole_sympy_check.py

Method step 2: verify the hand-derived cart-pole equations with SymPy.
You fill in the TODO parts (kinematics, energies, Euler-Lagrange).
Everything else is given and commented.

Run:   python cartpole_sympy_check.py
Needs: pip install sympy
"""
import sympy as sp

# ---------------------------------------------------------------
# 1. Symbols
# ---------------------------------------------------------------
# Physical parameters. positive=True tells SymPy these are > 0,
# so simplify() can cancel things like sqrt(l**2) -> l.
M, m, l, g = sp.symbols('M m l g', positive=True)
F = sp.symbols('F', real=True)          # horizontal force on the cart

# Generalized coordinates and their derivatives as plain symbols.
# We treat q, qdot, qddot as independent symbols and do d/dt by hand
# with the chain rule (ddt below). Same algebra as on paper.
x, th = sp.symbols('x theta', real=True)
xd, thd = sp.symbols('xdot thetadot', real=True)
xdd, thdd = sp.symbols('xddot thetaddot', real=True)

q = sp.Matrix([x, th])
qd = sp.Matrix([xd, thd])
qdd = sp.Matrix([xdd, thdd])


def ddt(expr):
    """Total time derivative of an expression f(q, qdot).

    Chain rule (no explicit time dependence in our problems):
        df/dt = sum_i  df/dq_i * qdot_i  +  df/dqdot_i * qddot_i
    This is exactly the step where 'd/dt, not partial d/dt' matters.
    """
    return sum(sp.diff(expr, q[i]) * qd[i] + sp.diff(expr, qd[i]) * qdd[i]
               for i in range(len(q)))


# ---------------------------------------------------------------
# 2. Kinematics  (TODO)
# ---------------------------------------------------------------
# Inertial positions as 2x1 column vectors [horizontal, vertical].
# theta is measured from the UPWARD vertical. Vertical zero at the pivot.
r_c = sp.Matrix([x, 0])   # TODO: cart position
r_p = sp.Matrix([x+l*sp.sin(th), l*sp.cos(th)])   # TODO: pole mass position

# Velocities from Jacobians: rdot = J(q) qdot  (same J as on paper)
J_c = r_c.jacobian(q)    # 2x2
J_p = r_p.jacobian(q)    # 2x2
sp.pprint("Cart Jacobian J_c ="); sp.pprint(J_c)
sp.pprint("Pole Jacobian J_p ="); sp.pprint(J_p)
v_c = J_c * qd
v_p = J_p * qd
sp.pprint("Cart velocity v_c ="); sp.pprint(v_c)
sp.pprint("Pole velocity v_p ="); sp.pprint(v_p)

# ---------------------------------------------------------------
# 3. Energies  (TODO)
# ---------------------------------------------------------------
# Hint: |v|^2 is v.dot(v)
T = sp.Rational(1,2) * M * v_c.dot(v_c) + sp.Rational(1,2) * m * v_p.dot(v_p)  # TODO: kinetic energy of cart + pole mass
V = m*g*l*sp.cos(th)              # TODO: potential energy
L = T - V

# ---------------------------------------------------------------
# 4. Euler-Lagrange  (TODO)
# ---------------------------------------------------------------
Q = sp.Matrix([F, 0])    # generalized forces [Q_x, Q_theta]

# Build a 2x1 Matrix whose i-th entry is
#     d/dt(dL/dqdot_i) - dL/dq_i - Q_i      (each entry = 0)
# Use ddt(...) and sp.diff(...). A list comprehension over i works.
EL = sp.Matrix([ddt(sp.diff(L, qd[i])) - sp.diff(L, q[i]) - Q[i] for i in range(len(q))])
EL = sp.simplify(EL)
print("Euler-Lagrange equations (each = 0):")
sp.pprint(EL)

# ---------------------------------------------------------------
# 5. Compare with your hand derivation (session 1, eqs 1 and 2)
# ---------------------------------------------------------------
# Written as (left side - right side), so each should be 0.
hand_eq1 = (M + m)*xdd + m*l*thdd*sp.cos(th) - m*l*thd**2*sp.sin(th) - F
hand_eq2 = m*l**2*thdd + m*l*xdd*sp.cos(th) - m*g*l*sp.sin(th)

# If SymPy and your hand derivation agree, both print 0.
print("eq1 difference:", sp.simplify(EL[0] - hand_eq1))
print("eq2 difference:", sp.simplify(EL[1] - hand_eq2))

# ---------------------------------------------------------------
# 6. Mass matrix and gravity vector
# ---------------------------------------------------------------
# EL is linear in qddot, so D(q) is its Jacobian with respect to qddot.
D = sp.simplify(EL.jacobian(qdd))
# Gravity vector G(q) = dV/dq (as a column)
G = sp.Matrix([V]).jacobian(q).T
print("D(q) =");   sp.pprint(D)
print("det D =");  sp.pprint(sp.trigsimp(sp.factor(D.det())))
print("G(q) =");   sp.pprint(G)

# ---------------------------------------------------------------
# 7. Solve for the accelerations
# ---------------------------------------------------------------
# Compare these with your simplified xddot and thetaddot from today.
sol = sp.solve(list(EL), [xdd, thdd], dict=True)[0]
print("xddot =");     sp.pprint(sp.simplify(sol[xdd]))
print("thetaddot ="); sp.pprint(sp.simplify(sol[thdd]))
