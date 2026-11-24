# S01 · P1 NumPy: shapes and broadcasting

## Warm-up (blank file warmup_s01.py, no docs, 5 min)
Predict each output in a comment BEFORE running.
1. `a` = integers 0..9. Print its shape and dtype.
2. `Z` = 3x4 float zeros. Set row 1 to ones. Then set the last column to 7. Print Z.
3. From `a`: the last 3 elements; every second element starting at 0; `a` reversed.
4. Turn `a` into shape (2, 5), then back to (10,).

## Exercises (write s01.py from a blank file)
Run: `python test_s01.py` (from inside p1_numpy).
Do not open test_s01.py until you pass or are stuck. Just run it.

Rules for every function:
- First line inside it: a comment with input and output shapes.
- E2, E5, E6: a comment with the shape of every intermediate, e.g. `# (n,1) * (1,m) -> (n,m)`.
- E1, E2, E5, E6: no `for` or `while` (the test checks).

E1. `as_column(x)`
  In: `x` of shape (n,), (n,1) or (1,n). Out: shape (n,1), same values in order.

E2. `outer_by_broadcast(a, b)`
  In: a (n,), b (m,). Out: (n,m) with out[i,j] = a[i]*b[j].
  Not allowed: np.outer, np.multiply.outer, np.einsum.

E3. `split_states(X)`  (cart-pole)
  In: X (N,4), columns [x, x_dot, theta, theta_dot].
  Out: tuple (x, x_dot, theta, theta_dot), each shape (N,).

E4. `peak_abs_theta(t, X)`  (cart-pole)
  In: t (N,), X (N,4). Out: (t_peak, theta_peak) as Python floats.
  theta_peak is the SIGNED theta where |theta| is largest. First one if tied.

E5. `pairwise_dist(P)`
  In: P (N,2) points. Out: D (N,N), D[i,j] = Euclidean distance between point i and j.

E6 (stretch). `normalize_columns(X)`
  In: X (N,d). Out: (N,d), every column has mean 0 and std 1 (population std, ddof=0).
  If any column has std 0, raise ValueError.
