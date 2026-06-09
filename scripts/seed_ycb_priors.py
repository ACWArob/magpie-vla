"""
Seed GraspMemory with known object physical properties from the YCB dataset
and common household objects.

Sources:
  - YCB object masses: Calli et al. 2015 (Yale-CMU-Berkeley Object Set)
  - Friction coefficients: rubber-coated gripper vs various materials (empirical)
  - Stiffness: DeliGrasp k parameter estimates

Run once to pre-populate force_priors.json. These are priors — real grasps
will override them via Kalman updates after n ≥ 2 observations.

F_min = mass*g / (2*mu)  — minimum force to support weight with two contact points.
Initial x = F_min * 1.5  — 50% safety margin over minimum.
"""

import sys, os, json, pathlib

sys.path.insert(0, os.path.dirname(__file__))
from grasp_memory import GraspMemory, _P0

# ── YCB + common objects ──────────────────────────────────────────────────────
# (name_lower, mass_g, mu_gripper, notes)
OBJECTS = [
    # YCB set — masses from Calli et al. 2015
    ('master chef can',   414., 0.60, 'metal can'),
    ('cracker box',        411., 0.55, 'cardboard box'),
    ('sugar box',          514., 0.55, 'cardboard box'),
    ('tomato soup can',    349., 0.60, 'metal can'),
    ('mustard bottle',     603., 0.55, 'plastic bottle'),
    ('tuna fish can',      171., 0.60, 'metal can'),
    ('pudding box',        187., 0.50, 'cardboard box'),
    ('gelatin box',        117., 0.50, 'cardboard box'),
    ('potted meat can',    370., 0.60, 'metal can'),
    ('banana',             118., 0.45, 'soft fruit'),
    ('pitcher base',       66., 0.45, 'plastic'),
    ('bleach cleanser',    987., 0.50, 'plastic bottle'),
    ('bowl',               147., 0.40, 'ceramic/plastic'),
    ('mug',                118., 0.50, 'ceramic mug'),
    ('power drill',        895., 0.55, 'plastic/metal'),
    ('wood block',         729., 0.65, 'wood block'),
    ('scissors',           082., 0.40, 'metal/plastic'),
    ('large marker',       019., 0.45, 'plastic marker'),
    ('large clamp',        125., 0.50, 'metal clamp'),
    ('extra large clamp',  202., 0.50, 'metal clamp'),
    ('foam brick',         013., 0.60, 'foam — very soft'),
    ('golf ball',          046., 0.35, 'hard rubber'),
    ('baseball',           145., 0.55, 'leather/cork'),
    ('softball',           177., 0.55, 'leather'),
    ('tennis ball',        058., 0.50, 'rubber — soft'),
    ('racquetball',        042., 0.50, 'rubber — soft'),
    ('toy airplane',       097., 0.40, 'plastic'),
    ('lego duplo',         020., 0.55, 'ABS plastic'),
    ('dice',               012., 0.50, 'hard plastic'),
    ('padlock',            182., 0.50, 'metal'),
    # Common lab/demo objects
    ('red cube',           120., 0.60, 'wood/plastic cube'),
    ('green cube',         120., 0.60, 'wood/plastic cube'),
    ('blue cube',          120., 0.60, 'wood/plastic cube'),
    ('cube',               120., 0.60, 'generic cube'),
    ('apple',              182., 0.40, 'fruit — soft'),
    ('orange',             198., 0.40, 'fruit — medium'),
    ('lemon',              130., 0.40, 'fruit — medium'),
    ('pear',               160., 0.38, 'fruit — soft'),
    ('kiwi',               090., 0.40, 'fruit — small'),
    ('pen',                012., 0.45, 'plastic pen'),
    ('marker',             022., 0.45, 'plastic marker'),
    ('bottle',             200., 0.50, 'generic plastic bottle'),
    ('water bottle',       500., 0.50, 'filled plastic bottle'),
    ('phone',              175., 0.45, 'glass/aluminium'),
    ('book',               300., 0.55, 'paper/cardboard'),
    ('small box',          100., 0.55, 'cardboard box'),
    ('tape',               150., 0.55, 'plastic roll'),
    ('stapler',            280., 0.50, 'metal/plastic'),
    ('eraser',             030., 0.65, 'rubber'),
]

G = 9.81  # m/s²


def compute_initial_force(mass_g: float, mu: float) -> float:
    """F_min*1.5 safety margin, clipped to valid gripper range."""
    f_min = (mass_g / 1000. * G) / (2. * max(mu, 0.01))
    return float(min(max(f_min * 1.5, 0.5), 16.0))


def seed(data_dir: str = '~/magpie_control/data/grasp_log',
         dry_run: bool = False):
    gm = GraspMemory(data_dir)
    priors_path = pathlib.Path(data_dir).expanduser() / 'force_priors.json'

    seeded = 0
    skipped = 0
    for name, mass_g, mu, note in OBJECTS:
        key = name.lower().strip()
        x0  = compute_initial_force(mass_g, mu)

        if key in gm._priors:
            n = gm._priors[key]['n']
            if n >= 2:
                # Real grasps exist — don't overwrite
                print(f'  SKIP  {key:<30} (n={n} real grasps)')
                skipped += 1
                continue
        if not dry_run:
            # Seed with n=1 synthetic observation so real grasps quickly override
            gm._priors[key] = {'x': x0, 'P': _P0 * 0.75, 'n': 1}
        print(f'  SEED  {key:<30}  force={x0:.2f}N  mass={mass_g:.0f}g  mu={mu}  [{note}]')
        seeded += 1

    if not dry_run:
        gm._save_priors()
    print(f'\nSeeded {seeded} objects, skipped {skipped} (already have real grasps).')
    print(f'Priors saved to {priors_path}')


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default='~/magpie_control/data/grasp_log')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    seed(args.data_dir, args.dry_run)
