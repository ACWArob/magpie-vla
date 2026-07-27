import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.weight_estimate import weight_from_fz, assess, reconcile_force


def test_weight_sign_convention():
    # baseline 0, holding -1.96N (downward) -> ~200g
    w = weight_from_fz(0.0, -1.96)
    assert abs(w - 1.96) < 1e-6
    assert weight_from_fz(0.0, 0.5) == 0.0     # noise never yields negative weight


def test_mass_readout():
    e = assess(fz_baseline=0.0, fz_holding=-0.98)   # ~100g
    assert 95 <= e.mass_g <= 105, e.mass_g


def test_heavier_needs_more_force():
    light = assess(0.0, -0.5)
    heavy = assess(0.0, -5.0)
    assert heavy.grip_force_min_n > light.grip_force_min_n


def test_heavy_delicate_conflict_flagged():
    # heavy object (~1kg, e.g. full water bottle) with a low delicate cap -> conflict
    e = assess(0.0, -9.81, delicate_force_cap_n=8.0)
    assert e.conflict and 'flag' in e.note


def test_reconcile_clamps_and_reports():
    e = assess(0.0, -9.81, delicate_force_cap_n=8.0)  # ~1kg, needs > 8N
    force, ok = reconcile_force(seed_force_n=6.0, weight=e, delicate_force_cap_n=8.0)
    assert force == 8.0 and ok is False               # clamped, flagged unsafe

    e2 = assess(0.0, -0.5, delicate_force_cap_n=8.0)  # light, fine
    force2, ok2 = reconcile_force(4.0, e2, 8.0)
    assert ok2 and force2 >= 4.0


def test_not_lifted_reads_zero():
    e = assess(0.1, 0.1)
    assert e.weight_n == 0.0 and 'no measurable' in e.note


if __name__ == '__main__':
    fails = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            try: f(); print(f'  PASS {n}')
            except AssertionError as e: fails += 1; print(f'  FAIL {n}: {e}')
    raise SystemExit(fails)
