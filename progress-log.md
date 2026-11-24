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

## Session 2 (2026-11-20, part 1)

**Covered**
- Quiz: EL with Q (correct), shapes of J and D (correct), meaning of the m l cos(th) coupling (forgot; re-explained), hanging pendulum linearization (math right, comparison with upright incomplete).
- Matrix form: D = [[M+m, m l cos th], [m l cos th, m l^2]], C = [[0, -m l thdot sin th], [0, 0]], G = [0, -m g l sin th], B = [1, 0].
- det D = m l^2 (M + m sin^2 th) > 0. det D = 0 means a zero-inertia direction (singularity).
- Explicit accelerations:
  xddot  = (F + m l thdot^2 sin th - m g sin th cos th) / (M + m sin^2 th)
  thddot = ((M+m) g sin th - F cos th - m l thdot^2 sin th cos th) / (l (M + m sin^2 th))
- Switched tooling from MATLAB to Python: SymPy, SciPy solve_ivp, MuJoCo, python-control, PyTorch. VS Code + .venv.
- Step 2 done: cartpole_sympy_check.py, SymPy matches hand derivation (differences print 0; D, det D, G, qddot all match).
- Step 3 done: cartpole_sim.py with state s = [x, th, xdot, thdot]. F = 0 drifts: E ~1e-9, p_x ~1e-9 (upright), 1e-10 / 1e-12 (hanging).
  Conserved p_x = (M+m) xdot + m l thdot cos th (x is cyclic, shift symmetry, Noether).
  Checks matched: x(t) = x_cm - (m l/(M+m)) sin th; hanging period ~1.30 s vs predicted 2 pi / sqrt(g (M+m)/(M l)) = 1.29 s.

**Got right**
- Matrix form, det D, inverse, simplification, equilibrium, M -> infinity limit, push-direction check, G = dV/dq.
- State-space form, both simulations, conservation checks.

**Mistakes to watch**
- Lost the square on sin^2 th when copying (carried through to the end).
- Units: radians are dimensionless; thdot^2 has units 1/s^2.
- "det D = 0 iff M = 0" wrong (needs sin th = 0 too).
- Used V = m g l sin th in code (should be cos th: theta from upward vertical).
- Tried to build T from hand-typed D in SymPy check (circular verification). Checker must not be given the answer.
- Energy conservation reason: not "no external forces" (gravity is external); it is "all remaining forces are conservative or do no work".
- Physical meaning of terms is the weak spot; ask "what does this term do?"

**Next**
- git init + first commit (.gitignore for .venv), push to GitHub.
- Step 4: build cart-pole in MuJoCo (cartpole.xml): hinge axis to match theta convention, point mass on massless rod, contacts/damping off.
- Step 5: overlay MuJoCo and solve_ivp trajectories.

**Open questions**
- None.

## Session 2 (2026-11-20, part 2)

**Covered**
- MuJoCo basics: MJCF XML, body tree (world > cart > pole), slide and hinge joints (qpos = q), explicit <inertial> for point mass on massless rod, model (constants) vs data (state), mj_forward vs mj_step, motor actuator (force = gear * ctrl).
- Step 4a/4b done: cartpole.xml built (M = 1, m = 0.2, l = 0.5, RK4, dt = 1e-3, contacts off). mujoco_first_look.py: nq = 2, nv = 2, nu = 1, masses correct, bob position matches r_p for 4 angles.
- Hinge axis = +y. Frames are right-handed (x cross y = z), so with x right and z up, +y points INTO the screen.
- Step 4c: viewer explored (control slider, perturbations).

**Got right**
- Right-hand rule reasoning, once the frame was correct. All 4b checks passed.

**Mistakes to watch**
- Assumed +y points out of the page. Always check x cross y = z before using a frame.
- 4c questions not answered: does the pole fall on its own at exactly theta = 0 (no: exact equilibrium, torque m g l sin 0 = 0; unstable, perturbations grow like e^(4.85 t)); push direction check.

**Next**
- Step 5: fill the MuJoCo loop in cartpole_compare.py (TODOs 1 to 4), run 3 cases. Expect errors ~1e-5 or smaller for F = 0.
- Explain why the forced case (F = 2 sin 3t) has a larger error, then fix it with a one-line change (hint: when is F evaluated; zero-order hold).
- git commit if not done.

**Working agreements (from Dishara, 2026-11-20)**
- Start each day by agreeing the scope and a clear end goal for that day (he is fitting this around other work).
- Coding: give more fill-in-the-blank work (well-commented skeletons with real code gaps: loops, function bodies), increasing over time, so he learns the libraries, not just the physics.

## Session 3 (2026-11-21)

**Covered**
- Quiz: p_x conservation via Noether (correct); energy conservation with gravity (wrong: said gravity does no work; correct reason is gravity is conservative so its work is -dV and is already inside E = T + V, constraint forces do no work, F is non-conservative); hinge axis -y (half right: MuJoCo physics still valid, but the convention mismatch breaks comparison and would make an LQR push the wrong way).
- Step 5 done: cartpole_compare.py. MuJoCo vs own model, max errors:
  upright F = 0: x 7.4e-7 m, theta 8.8e-6 rad
  hanging F = 0: x 4.7e-8 m, theta 5.7e-7 rad
  forced F = 2 sin 3t: 5.5e-4 m, 8.2e-4 rad before fix; fixed by sampling F at the step midpoint, data.ctrl[0] = F_func(t[k] + dt/2).
- Zero-order hold: MuJoCo holds ctrl constant over a step, so a start-of-step sample lags the true signal by about dt/2 (first-order error in dt). Midpoint sampling removes it.
- Forced case drift (cart ends ~5.7 m away although F averages to 0) is real physics: sine force started from rest gives mean velocity ~ (2/3)/(M+m).

**Got right**
- MuJoCo stepping loop (reset, set state, ctrl, mj_step, record) with named joint access.
- Diagnosed two bugs from curve shapes: unsaved file (MuJoCo curve exactly 0), hard-coded initial state copied from another script (all cases started at x = 0.2, th = 0.1; flat 0.2 m x error = translation symmetry).
- Worked out the zero-order-hold fix from hints with a numeric sketch.

**Mistakes to watch**
- Ran code without saving (turn on Auto Save).
- Copied hard-coded values between scripts; a function ignored its input argument.
- First "fix" F_func(t[k]) was identical to F_func(data.time); check that a change actually changes the output.
- Said the staircase is "ahead" of the curve; it lags (is delayed by ~dt/2). Conclusion F(t + dt/2) was right.
- Skipped answering hint questions; answer each one in writing.
- Gravity does work on the bob. Conservative force != no work.

**Next**
- Report the new forced-case error line (expect ~1e-6).
- git commit step 5.
- Step 6a: linearize about upright by hand (A, B), eigenvalues; verify in code (fill-in).
- Animation track: first matplotlib animation of the cart-pole from own simulation data (first portfolio post).

**Open questions**
- With feedback control F = -K s, the future state is unknown, so midpoint sampling is not possible: how much does the dt/2 delay matter? (Comes up in step 6.)

## Session 4 (2026-11-22, night + morning)

**Covered**
- Step 5 closed: forced case after midpoint fix, max errors x 2.03e-6 m, theta 8.64e-7 rad (was 5.53e-4 m, 8.22e-4 rad; about 270x and 950x smaller). All 3 cases agree at the 1e-6 level.
- Decided Phase 1 order: 6a, 6b, post 1, step 7 mini AI layer, post 2. Updated Lab and Writing Desk project descriptions (exports/ convention, three projects).
- LQR big picture: F = -K s, K chosen to minimize J = integral of (s^T Q s + R F^2) dt; needs a linear model, hence 6a.
- 6a done by hand:
  Equilibrium: theta = thetadot = xdot = 0, F = 0, any x (translation symmetry).
  Linearized: xddot ~ -(m g / M) theta + F / M ;  thetaddot ~ ((M+m) g / (l M)) theta - F / (l M)
  s = [x, th, xdot, thdot]:
  A = [[0,0,1,0],[0,0,0,1],[0,-m g/M,0,0],[0,(M+m) g/(l M),0,0]],  B = [0, 0, 1/M, -1/(l M)]
  det(lambda I - A) = lambda^2 (lambda^2 - (M+m) g/(l M))
  Eigenvalues: 0 (twice), +/- sqrt((M+m) g/(l M)) = +/- 4.85 1/s for M = 1, m = 0.2, l = 0.5.
  Unstable time constant 1/4.85 = 0.21 s (tilt grows by e every 0.21 s).
  Same magnitude as the hanging-case frequency sqrt(g (M+m)/(M l)): upright gives real +/-, hanging gives imaginary +/- i.

**Got right**
- Equilibrium, linearization, A and B, units of A and B rows, characteristic polynomial.
- Sign meaning of -F/(l M) (push the base under the falling pole).

**Mistakes to watch**
- Wrote "second-order terms = 0"; they are approx 0 (dropped), not zero. They are why LQR fails at large angles.
- Reported lambda = +/- 23.544 s^-2: that is lambda^2. lambda = sqrt(23.544) = 4.85 s^-1. Units check (an eigenvalue is a rate, 1/s) would have caught it.
- Meaning of -(m g/M) theta: explanation muddled (not about overcoming F). Correct: with F = 0 the falling bob pushes the cart back through the rod (momentum conservation), so pole tilts right, cart accelerates left.
- Why (M+m)g/(lM) > g/l: gave the math only. Physical reason: the cart recoils away from under the bob, so the pole loses support and falls faster than on a fixed pivot.
- Double zero eigenvalue: said "theta crossing pi pulls the cart back". Wrong: nothing pulls the cart back; the cart can sit anywhere (one zero) and drift at constant velocity x = x0 + v t (second zero). So LQR must penalize x too.

**Next**
- 6a stretch: controllability, rank [B, AB, A^2 B, A^3 B] = 4 by hand.
- 6b: LQR with python-control (fill-in code), test in own model and MuJoCo, push recovery.
- Then post 1 (balancing animation). Create phase1_cartpole/exports/ with README.md.

**Open questions**
- With F = -K s computed from the current state (no midpoint sampling possible), how much does the dt/2 hold delay matter?

## Session 5 (2026-11-22, afternoon and evening)

**Covered**
- Controllability (new theory). Kalman rank test: C = [B, AB, A^2 B, A^3 B], controllable iff rank 4.
  Why: discrete intuition (s_{k+1} = A_d s_k + B_d u_k, reachable set spanned by B, AB, ...); rigorous continuous version via variation of constants s(t) = integral of e^{A(t-tau)} B u(tau) dtau and the power series of e^{At}; Cayley-Hamilton stops at A^3 B. Dishara derived the variation-of-constants formula himself with an integrating factor.
- Cart-pole, with a = -m g/M, b = (M+m) g/(l M), c = 1/M, d = -1/(l M):
  AB = [c, d, 0, 0], A^2 B = [0, 0, a d, b d], A^3 B = [a d, b d, 0, 0]
  det C = -d^2 (b c - a d)^2, b c - a d = g/(l M), so det C = -g^2/(l^4 M^4) = -1539.78 (code: -1539.7776, matches).
  Controllable iff g != 0. Physical reason: the rod can only push along itself (vertical near upright), so the cart cannot move the bob sideways directly; gravity on a tilted pole does. With g = 0, x + l theta (bob position) is unreachable, like two blocks pushed by the same force.
- 6b part 1 done: cartpole_lqr.py (most code written by Dishara).
  Q = diag(1, 10, 1, 1), R = 0.1: K = [-3.162, -47.577, -5.752, -10.524].
  Closed-loop eigenvalues: -9.00, -3.49, -1.40 +/- 0.07i (eig(A - B K) matches python-control E).
  Nonlinear model, s0 = [0, 0.1, 0, 0]: F(0) = 4.76 N, cart peak x = 0.165 m, theta undershoot about -0.03 rad; balanced pole and cart home within about 6 s (plot).
  Settling (2% band, computed by Claude, not yet rerun by Dishara): theta 3.7 s, x 5.1 s. Rule of thumb 4/|Re lambda_slowest| = 2.9 s; slower in practice because the slowest pair is nearly repeated.
- K_x < 0: non-minimum phase. Cart right of home is first pushed further right so the pole leans left, then the system walks home. Visible in the plot (theta negative near t = 0.7 s while x peaks). Claude's check for s0 = [0.5, 0, 0, 0]: F(0) = +1.58 N, x out to 0.512 m, theta min -0.036 rad (Dishara to confirm).
- LQR regulates the whole state to 0; Q sets priorities (balance first, homing second). Reference tracking later with F = -K (s - s_ref).

**Got right**
- A and B in code; open-loop eigenvalues, rank, det check, closed-loop eigenvalues.
- A p shortcut for computing AB, A^2 B, A^3 B; matrix C.
- Sign of K_theta (push under the falling pole). Broom intuition for the wrong-way push.

**Mistakes to watch**
- det C by cofactor: kept the wrong column in the first minor and dropped a sign in the second. Conclusion survived, but the claimed reason (m != 0) was wrong; det C does not depend on m.
- Answered "what if det C = 0" with the definition, not the physics.
- Code: controller returned +K s (positive feedback, would throw the pole over); t undefined (NameError at plot); logged F_hist computed separately from the controller (single source of truth); variable S reused for Riccati matrix and states.
- Could not map the K_x sign to the broom intuition at first.
- Skipped the experiments.

**Next**
- Experiments table (one change at a time vs baseline): Q_thth = 100; R = 1; s0 = [0.5, 0, 0, 0]; Q_xx = 0.001; theta0 = 0.3, 0.6, 1.0 and larger until failure. Record peak |F|, max cart excursion, theta settling, notes. Refactor into run(Q, R, s0, t_end) and loop.
- Answer: what Q_thth and R trade away; why LQR works at 1 rad; what makes it fail.
- 6b part 2: same K on MuJoCo, push recovery (kick to the bob mid-balance). Data for post 1.
- Create phase1_cartpole/exports/ with README.md.

**Open questions**
- With F = -K s (no midpoint sampling), how much does the dt/2 hold delay matter in MuJoCo? (6b part 2.)

**Animation / post status**
- None yet. Post 1 after 6b part 2.

## Session 6 (2026-11-23, part 1)

**Covered**
- Moved project folder to D:\Vault\04 Projects\Physics Sims. Rebuilt .venv (venvs store absolute paths). Found the old venv only had PyOpenGL: packages had been installed into global Python. Reinstalled numpy, scipy, sympy, matplotlib, control, mujoco in .venv; requirements.txt to be regenerated in UTF-8 (Out-File -Encoding utf8). Git history and remote moved fine; steps 4 to 6a not yet committed.
- Quiz: Q2 (wrong-way push sequence) and Q3 (slowest pair -1.40 +/- 0.07i, ~4/1.4 = 2.9 s) correct. Q1 (why gravity gives controllability) wrong twice, then correct as a chain: rod pushes only along itself (near vertical) -> cart cannot push bob sideways -> on a tilted pole the rod carries the weight ~ m g, whose sideways part ~ m g theta is the only sideways force on the bob -> with g = 0 the bob position x + l theta is uncontrollable.
- LQR experiments (nonlinear model, one change at a time, verified against Claude's runs). Settling = 2% of peak |theta|, last exit from band:
  baseline Q = diag(1,10,1,1), R = 0.1, s0 = [0,0.1,0,0]: peak F 4.76 N, max x 0.16 m, theta settles 3.73 s
  Q_thth = 100: 6.12 N, 0.15 m, 3.67 s
  R = 1: 3.24 N, 0.25 m, 5.26 s
  s0 = [0.5,0,0,0]: 1.58 N, 0.51 m, 5.20 s (wrong-way push confirmed)
  Q_xx = 0.001: 4.01 N, 0.26 m, 2.39 s (slowest closed-loop eigenvalue about -0.03, cart still ~0.24 m from home at 6 s)
  theta0 = 0.3 / 0.6 / 1.0: 14.27 / 28.55 / 47.58 N (= 47.58 * theta0), 0.51 / 1.19 / 2.87 m, 3.77 / 3.90 / 3.90 s

**Mistakes to watch**
- Ran "R = 1" with R = 0.01 (got 9.99 N). Sanity question missed: more force penalty cannot give more force.
- Settling metric: first took the first entry into a fixed 0.02 rad band (gave ~0.3 s, the first zero crossing; and 0.00 s when theta starts at 0), then the last sample inside the band. Correct: last sample OUTSIDE a 2%-of-peak band, plus one, with guards for "never left" and "never settled".

**Next (after gym)**
- Answer: what Q_thth buys and costs; R trade-off; why Q_xx = 0.001 balances faster; why LQR works at 1 rad and what breaks it (sweep theta0 > 1.0, note the force needed).
- 6b part 2: LQR on MuJoCo + push recovery. Commit steps 4 to 6b. requirements.txt in UTF-8.

## Session 6 (2026-11-23 evening to 2026-11-24 night, part 2)

**Decisions**
- Started a parallel Python Learn project (python_learn\ subfolder, own log python_learn\python-log.md) because Dishara can read code but not write it fluently from a blank file (recognition vs recall; autocomplete trained recognition). VS Code Copilot and inline suggestions off.
- Lab coding policy: Phases 1 to 3 code-light (Claude writes commented plumbing; Dishara does physics, math, interpretation and short parts up to ~10 lines). Phase 4 onward code-intensive.
- Lab project description updated (new folder path, coding policy, status).

**Next**
- Experiment answers (what Q_thth buys and costs, R trade-off, why Q_xx = 0.001 settles theta faster, why LQR works at 1 rad and the failure sweep theta0 > 1.0).
- 6b part 2: phase1_cartpole/cartpole_lqr_mujoco.py ready (3 TODOs: LQR law, generalized force of a push on the bob Q = J_p^T f, MuJoCo state vector). Experiments A (theta0 = 0.1) and B (5 N push on the bob for 0.1 s at t = 1 s), own model vs MuJoCo.

## Session 7 (2026-11-24 night)

**Covered**
- Quiz: Q1 settling time (half right: must stay in the band for good; first entry = first zero crossing). Q2 wrong and backwards: said larger R gives larger force. Correct: R penalizes F^2 in J, so larger R = less force, gentler, slower (own data: R = 1 gave 3.24 N vs 4.76 N). Q3: pole tilts +x, controller pushes cart +x to get under it (correct); at the instant of a horizontal push on the bob at theta = 0 the cart feels nothing (xddot = 0, thetaddot = Fp/(m l)), visible in B plot (x flat during the push).
- 6b part 2 done: cartpole_lqr_mujoco.py. Same K on own model (solve_ivp, push as generalized force Q = J_p^T f: Q_x = Fp, Q_th = Fp l cos th) and MuJoCo (push via xfrc_applied on the pole body).
  A (theta0 = 0.1): curves overlap, max error ~4e-4 m in x, ~2e-4 rad in theta (Claude's run: 4.23e-4, 1.91e-4).
  B (5 N on bob for 0.1 s at t = 1 s): pole survives; theta +0.07 then -0.42 rad; peak F 31.22 N; max cart excursion 1.746 m; errors ~1e-3.
- Why error ~1e-4 here vs ~1e-6 in step 5: feedback F = -K s is computed from the state at the start of each MuJoCo step and held for dt = 1 ms (zero-order hold); solve_ivp updates F continuously. Midpoint sampling is impossible because the future state is unknown. Effective delay ~dt/2 = 0.5 ms vs instability time 1/4.85 = 0.21 s and fastest closed-loop mode 1/9 = 0.11 s, so negligible here. Delay matters when comparable to the fastest time scale (real sensors/motors: tens of ms; Phase 3 hidden effect).

**Mistakes to watch**
- TODO 3 copy-paste bug: xdot read from qpos instead of qvel -> controller lost cart damping; A oscillated, B diverged (765 N, 71 m). Diagnosed by: own-model curves fine, MuJoCo wrong from t = 0 without push -> MuJoCo-side state bug, growing oscillation -> velocity/damping. Fix: np.concatenate([data.qpos, data.qvel]).
- Q1 of the 6b questions answered with the wrong cause (sine vs impulse); the cause is the held feedback, present even in A with no push.
- Did not understand "does 1 ms matter": compare the delay with the system's time constants.

**Next**
- Commit steps 4 to 6b (+ requirements.txt in UTF-8).
- Experiment answers still owed: what Q_thth buys/costs, R trade-off sentence, why Q_xx = 0.001 settles theta faster, why LQR works at 1 rad + failure sweep theta0 > 1.0.
- Create phase1_cartpole/exports/ with README.md (verified numbers from 6b) and export experiment B trajectory (CSV) for post 1.
- Post 1: matplotlib animation of balance + push recovery (Claude writes rendering code; data from cartpole_lqr_mujoco.py).

**Animation / post status**
- Data for post 1 ready (experiment B). Animation not started.
