import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.policy_memory import pack, MEMORY_DIM, MEMORY_NAMES, _EMPTY


def test_empty_neighbors_is_zero_token():
    t = pack([])
    assert t.shape == (MEMORY_DIM,) and t.dtype == np.float32
    assert np.allclose(t, 0.0)


def test_coverage_and_confidence_zero_when_no_memory():
    t = pack([])
    assert t[MEMORY_NAMES.index('mem_coverage')] == 0.0
    assert t[MEMORY_NAMES.index('mem_top_sim')] == 0.0


def test_force_is_similarity_weighted():
    # near neighbor says 2N, far neighbor says 10N -> weighted mean near 2
    nb = [dict(sim=0.95, force=2.0, held=True),
          dict(sim=0.20, force=10.0, held=True)]
    t = pack(nb)
    fm = t[MEMORY_NAMES.index('mem_force_mean')]
    assert 2.0 <= fm < 4.0, fm


def test_success_rate():
    nb = [dict(sim=0.9, force=3, held=True),
          dict(sim=0.8, force=3, held=False),
          dict(sim=0.7, force=3, held=True)]
    assert abs(t := pack(nb)[MEMORY_NAMES.index('mem_success_rate')] - 2/3) < 1e-5


def test_angle_circular_mean_wraps():
    # grasp angle mod 180: 5 deg and 175 deg are ~10 deg apart, mean near 0/180
    nb = [dict(sim=0.9, force=3, angle=5.0, held=True),
          dict(sim=0.9, force=3, angle=175.0, held=True)]
    t = pack(nb)
    s = t[MEMORY_NAMES.index('mem_angle_sin')]; c = t[MEMORY_NAMES.index('mem_angle_cos')]
    ang = (np.degrees(np.arctan2(s, c)) / 2.0) % 180.0
    assert ang < 15 or ang > 165, ang


def test_sim_floor_drops_different_object():
    nb = [dict(sim=0.3, force=9, held=True)]   # below floor 0.5 -> nothing
    assert np.allclose(pack(nb, sim_floor=0.5), 0.0)


def test_missing_fields_dont_crash():
    nb = [dict(sim=0.9), dict(sim=0.8, force=4)]   # sparse
    t = pack(nb)
    assert t.shape == (MEMORY_DIM,)


if __name__ == '__main__':
    fails = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            try: f(); print(f'  PASS {n}')
            except AssertionError as e: fails += 1; print(f'  FAIL {n}: {e}')
    raise SystemExit(fails)
