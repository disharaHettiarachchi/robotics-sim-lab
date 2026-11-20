# Progress Log

## Session 1 (2026-11-19)

**Covered**
- Generalized coordinates: DOF = Cartesian coords minus constraints; q must have exactly DOF independent entries.
- Cart-pole kinetic energy via position, then velocity, then T.
- Mass matrix: rdot = J(q) qdot, D(q) = sum of m_k J_k^T J_k. Cart-pole:
  D = [[M+m, m l cos(th)], [m l cos(th), m l^2]]
  T = 1/2 (M+m) xdot^2 + m l xdot thdot cos(th) + 1/2 m l^2 thdot^2
- Derived Euler-Lagrange from Hamilton's principle (steps A to E):
  d/dt(dL/dqdot) - dL/dq = Q   (n coordinates give n equations)
- Cart-pole EOM derived and checked (clamped cart gives thddot = (g/l) sin th, upright unstable):
  (1) (M+m) xddot + m l thddot cos th - m l thdot^2 sin th = F
  (2) m l^2 thddot + m l xddot cos th - m g l sin th = 0

**Got right**
- T must use inertial velocity. DOF counts. J_c, J_p, D(q), locked-rod check.
- Chain rule for dL/deps, integration by parts, test function trick (eta = f b), adding Q.
- Both cart-pole equations, including product rule and the cancelling xdot thdot sin th terms.

**Mistakes to watch**
- Wrote partial d/dt instead of total d/dt (fixed by end of session).
- Dropped terms: cart KE, +x in pole position.
- J shape: must be (position components) x (number of q).
- Factored a time-dependent term out of the boundary bracket.
- Integral = 0 does not imply integrand = 0 (only with fixed sign or a "for all" test).
- Signs: V = m g h, minus sign from L = T - V.
- Step D: said 2n equations; correct is n (eta_i dot is not independent of eta_i).

**Next**
- Matrix form D(q) qddot + C(q, qdot) qdot + G(q) = B F. Check D matches the Jacobian mass matrix.
- Compute det D, show it is never zero.
- Solve explicitly for xddot and thddot (input to ode45).

**Open questions**
- Why stationary action is true (optional: Feynman path integral).
