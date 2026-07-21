"""Kill abandoned Jupyter kernels that are still holding CUDA memory.

Why this exists: VSCode's "Restart Kernel" starts a NEW kernel and leaves the
old one alive, still holding its CUDA context (~320-480MB each). On 2026-07-21
five of them accumulated over an afternoon of restarts and starved the policy:
SAM3 needs 4.25GB of a 7.59GB card, so ~2GB of dead kernels is the difference
between the eval running and OOMing on a 20MB allocation.

Safety: only processes whose cmdline contains 'ipykernel_launcher' are ever
killed. SAM3, GraspGenX, the gripper driver and the browser are never touched,
because they are not ipykernels. The calling process and its own children are
also skipped -- SAM3/GraspGenX run as children of the live kernel.

Caveat: a second notebook you deliberately left running WILL be reaped if it
holds GPU memory. That is the intent (free the VRAM), but pass dry_run=True
first if you are unsure.
"""
import os
import signal
import subprocess


def _ppid_of(pid):
    """Parent pid from /proc/<pid>/stat, parsed after the comm field so that
    process names containing ')' cannot shift the columns."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            return int(f.read().rsplit(')', 1)[1].split()[1])
    except Exception:
        return -1


def _cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return f.read().decode('utf-8', 'ignore')
    except Exception:
        return ''


def gpu_procs():
    """[(pid, mib)] currently holding CUDA memory."""
    try:
        out = subprocess.run(
            ['nvidia-smi', '--query-compute-apps=pid,used_memory',
             '--format=csv,noheader,nounits'],
            capture_output=True, text=True, timeout=15).stdout
    except Exception as e:
        print(f'  [reap] nvidia-smi failed: {e}')
        return []
    procs = []
    for line in out.strip().splitlines():
        try:
            pid, mib = (x.strip() for x in line.split(','))
            procs.append((int(pid), int(mib)))
        except Exception:
            continue
    return procs


def reap_zombie_kernels(dry_run=False, verbose=True, keep=()):
    """Kill every ipykernel holding GPU memory except us, our children, and
    anything in `keep`. Returns MiB freed.

    Call this FROM INSIDE the live kernel: self-detection is what protects the
    running notebook. Run standalone and it cannot tell which kernel is live,
    which is why __main__ refuses to kill without an explicit --keep.
    """
    me = os.getpid()
    keep = set(keep) | {me}
    freed = 0
    for pid, mib in gpu_procs():
        if pid in keep or 'ipykernel_launcher' not in _cmdline(pid):
            continue
        if _ppid_of(pid) == me:              # our own child
            continue
        if dry_run:
            print(f'  [reap] WOULD kill kernel {pid} ({mib}MB)')
            freed += mib
            continue
        try:
            os.kill(pid, signal.SIGKILL)
            freed += mib
            if verbose:
                print(f'  [reap] killed abandoned kernel {pid} (+{mib}MB)')
        except ProcessLookupError:
            pass
        except PermissionError:
            print(f'  [reap] no permission to kill {pid} — not ours?')
    if verbose:
        if freed:
            print(f'  [reap] freed ~{freed}MB ({freed/1024:.1f}GB)')
        else:
            print('  [reap] no abandoned kernels')
    return freed


if __name__ == '__main__':
    import sys
    argv = sys.argv[1:]
    keep = [int(argv[argv.index('--keep') + 1])] if '--keep' in argv else []
    # Standalone we cannot identify the live kernel, so killing is opt-in.
    # In-notebook the self/child checks make it safe automatically.
    do_kill = '--yes' in argv and keep
    if not do_kill:
        print('DRY RUN — standalone cannot tell which kernel is live.\n'
              'To actually kill:  python3 scripts/gpu_reap.py --keep <LIVE_PID> --yes\n'
              '(or just call reap_zombie_kernels() from inside the notebook)\n')
    reap_zombie_kernels(dry_run=not do_kill, keep=keep)
    print('\ncurrently on GPU:')
    for pid, mib in gpu_procs():
        name = (_cmdline(pid).split('\x00')[0].split('/')[-1] or '?')[:28]
        kind = ('ipykernel' if 'ipykernel_launcher' in _cmdline(pid)
                else 'other (protected)')
        print(f'  {pid:>8}  {mib:>6}MB  {name:<28} {kind}')
