"""Checks for S01. Run: python test_s01.py  (or: pytest test_s01.py)"""
import io
import inspect
import tokenize

import numpy as np

try:
    import s01
except Exception as e:  # noqa: BLE001
    s01 = None
    IMPORT_ERR = e


def _fn(name):
    if s01 is None:
        raise AssertionError(f"s01.py failed to import: {IMPORT_ERR!r}")
    f = getattr(s01, name, None)
    if f is None:
        raise AssertionError(f"{name} not written yet")
    return f


def _no_loops(f):
    src = inspect.getsource(f)
    toks = tokenize.generate_tokens(io.StringIO(src).readline)
    bad = [t.string for t in toks if t.type == tokenize.NAME and t.string in ("for", "while")]
    assert not bad, f"{f.__name__} uses a loop ({bad[0]}). Vectorize it."


def test_e1_as_column():
    f = _fn("as_column")
    _no_loops(f)
    exp = np.array([[1], [2], [3]])
    for x in (np.array([1, 2, 3]), np.array([[1, 2, 3]]), np.array([[1], [2], [3]])):
        out = f(x)
        assert out.shape == (3, 1), f"input shape {x.shape}: got {out.shape}, want (3, 1)"
        assert np.array_equal(out, exp), f"input shape {x.shape}: wrong values {out.ravel()}"


def test_e2_outer():
    f = _fn("outer_by_broadcast")
    _no_loops(f)
    src = inspect.getsource(f)
    for banned in ("outer", "einsum"):
        assert banned not in src.replace("def outer_by_broadcast", ""), f"{banned} is not allowed"
    out = f(np.array([1, 2]), np.array([10, 20, 30]))
    assert out.shape == (2, 3), f"got shape {out.shape}, want (2, 3)"
    assert np.array_equal(out, [[10, 20, 30], [20, 40, 60]]), f"wrong values:\n{out}"
    rng = np.random.default_rng(0)
    a, b = rng.normal(size=4), rng.normal(size=7)
    assert f(a, b).shape == (4, 7)


def test_e3_split_states():
    f = _fn("split_states")
    X = np.arange(20).reshape(5, 4)
    parts = f(X)
    assert len(parts) == 4, f"want 4 outputs, got {len(parts)}"
    want = [[0, 4, 8, 12, 16], [1, 5, 9, 13, 17], [2, 6, 10, 14, 18], [3, 7, 11, 15, 19]]
    names = ["x", "x_dot", "theta", "theta_dot"]
    for p, w, n in zip(parts, want, names):
        assert p.shape == (5,), f"{n}: shape {p.shape}, want (5,)"
        assert np.array_equal(p, w), f"{n}: got {p}, want {w}"


def test_e4_peak_abs_theta():
    f = _fn("peak_abs_theta")
    t = np.linspace(0.0, 1.0, 6)
    X = np.zeros((6, 4))
    X[:, 2] = [0.0, 0.1, 0.3, -0.5, 0.2, 0.0]
    X[:, 0] = 9.0  # decoy: x column is large
    tp, thp = f(t, X)
    assert isinstance(tp, float) and isinstance(thp, float), "return Python floats"
    assert np.isclose(tp, 0.6) and np.isclose(thp, -0.5), f"got ({tp}, {thp}), want (0.6, -0.5)"
    X[:, 2] = [0.0, 0.4, -0.4, 0.0, 0.0, 0.0]
    tp, thp = f(t, X)
    assert np.isclose(tp, 0.2) and np.isclose(thp, 0.4), f"tie: got ({tp}, {thp}), want (0.2, 0.4)"


def test_e5_pairwise_dist():
    f = _fn("pairwise_dist")
    _no_loops(f)
    P = np.array([[0.0, 0.0], [3.0, 4.0], [0.0, 4.0]])
    D = f(P)
    assert D.shape == (3, 3), f"got shape {D.shape}, want (3, 3)"
    assert np.allclose(D, [[0, 5, 4], [5, 0, 3], [4, 3, 0]]), f"wrong values:\n{D}"
    Q = np.random.default_rng(1).normal(size=(50, 2))
    DQ = f(Q)
    assert DQ.shape == (50, 50)
    assert np.allclose(DQ, DQ.T) and np.allclose(np.diag(DQ), 0)


def test_e6_normalize_columns():
    f = _fn("normalize_columns")
    _no_loops(f)
    X = np.array([[1.0, 10.0], [2.0, 20.0], [3.0, 30.0]])
    Y = f(X)
    assert Y.shape == (3, 2), f"got shape {Y.shape}, want (3, 2)"
    c = 1.224744871391589
    assert np.allclose(Y, [[-c, -c], [0, 0], [c, c]]), f"wrong values:\n{Y}"
    try:
        f(np.array([[1.0, 5.0], [2.0, 5.0]]))
    except ValueError:
        pass
    else:
        raise AssertionError("constant column must raise ValueError")


if __name__ == "__main__":
    tests = [(n, g) for n, g in sorted(globals().items()) if n.startswith("test_")]
    passed = 0
    for name, test in tests:
        try:
            test()
            print(f"PASS  {name}")
            passed += 1
        except Exception as e:  # noqa: BLE001
            print(f"FAIL  {name}: {e}")
    print(f"\n{passed}/{len(tests)} passed")
