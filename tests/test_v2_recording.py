"""Tests for V2 state_builder (wrist_fz fix) + deformation_check."""
import sys, os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.state_builder import (
    build_state, wrist_fz_from_wrench, fz_is_live, STATE_DIM, STATE_NAMES)
from magpie_control.v2.deformation_check import assess


# ---- state_builder ----

def test_state_shape_and_fz_slot():
    s = build_state([0.1, -0.5, 0.3, 3.1, 0.0, 0.0], grip_mm=27.0, grip_force=8.0, wrist_fz=-2.5)
    assert s.shape == (STATE_DIM,) and s.dtype == np.float32
    assert s[STATE_NAMES.index('wrist_fz')] == np.float32(-2.5)
    assert s[STATE_NAMES.index('grip_mm')] == np.float32(27.0)


def test_wrist_fz_sign_preserved():
    # downward load must stay NEGATIVE — the place-guard depends on the sign
    class W:
        class wrench:
            class force: z = -4.2
    assert wrist_fz_from_wrench(W) == -4.2
    assert wrist_fz_from_wrench(None) == 0.0        # graceful pre-subscription


def test_fz_live_detector_catches_v1_bug():
    # V1 bug: wrist_fz constant 0 across the episode -> must read NOT live
    dead = np.zeros((20, STATE_DIM), np.float32)
    assert fz_is_live(dead) is False
    live = dead.copy()
    live[:, STATE_NAMES.index('wrist_fz')] = np.linspace(0, -5, 20)
    assert fz_is_live(live) is True


def test_bad_tcp_length_rejected():
    try:
        build_state([0, 0, 0], 10, 1, 0); assert False
    except ValueError:
        pass


# ---- deformation_check ----

def test_rigid_block_is_gentle():
    # block 50mm, seated at 49mm, force on target -> pristine
    r = assess(expected_width_mm=50, seated_aperture_mm=49,
               applied_force_n=16, target_force_n=16)
    assert r.ok and r.gentleness > 0.9, r


def test_crushed_strawberry_flagged_geometric():
    # expected 40mm, fingers closed to 22mm -> 45% over-close -> squish
    r = assess(expected_width_mm=40, seated_aperture_mm=22)
    assert not r.ok and 'over-closed' in r.reason, r
    assert r.deform_ratio > 0.4


def test_over_forced_flagged():
    # geometry fine, but 2x the target force -> over-forced
    r = assess(expected_width_mm=40, seated_aperture_mm=39,
               applied_force_n=30, target_force_n=15)
    assert not r.ok and 'over-forced' in r.reason, r


def test_missing_signals_do_not_fabricate_squish():
    # no width, no force -> cannot judge -> must NOT falsely fail
    r = assess(expected_width_mm=0, seated_aperture_mm=20)
    assert r.ok, r


def test_worst_of_two_signals():
    # geometry pristine but force terrible -> overall must fail on force
    r = assess(expected_width_mm=40, seated_aperture_mm=40,
               applied_force_n=40, target_force_n=10)
    assert not r.ok and r.gentleness < 0.6


if __name__ == '__main__':
    fails = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            try: f(); print(f'  PASS {n}')
            except AssertionError as e: fails += 1; print(f'  FAIL {n}: {e}')
    raise SystemExit(fails)
