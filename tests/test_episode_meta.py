import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from magpie_control.v2.episode_meta import EpisodeMeta, PHASES


def test_phase_flow():
    m = EpisodeMeta(task='pick up the strawberry', object_name='strawberry', instance_id='strawberry_1')
    assert m.frame() == {'phase': 'idle', 'phase_idx': 0}
    for p in ['approach', 'center', 'descend', 'squeeze', 'lift', 'transport', 'place', 'release']:
        m.set_phase(p)
    f = m.frame()
    assert f['phase'] == 'release' and f['phase_idx'] == PHASES.index('release')
    s = m.finish(success=True)
    assert s['phases_visited'][0] == 'idle' and s['phases_visited'][-1] == 'release'
    assert s['task'].startswith('pick up') and s['instance_id'] == 'strawberry_1'


def test_unknown_phase_rejected():
    m = EpisodeMeta('t', 'o')
    try:
        m.set_phase('yeet')
        assert False, 'should have raised'
    except ValueError:
        pass


def test_repeated_phase_not_duplicated():
    m = EpisodeMeta('t', 'o')
    m.set_phase('approach'); m.set_phase('approach')
    assert m.finish(True)['phases_visited'] == ['idle', 'approach']


if __name__ == '__main__':
    fails = 0
    for n, f in sorted(globals().items()):
        if n.startswith('test_'):
            try: f(); print(f'  PASS {n}')
            except AssertionError as e: fails += 1; print(f'  FAIL {n}: {e}')
    raise SystemExit(fails)
