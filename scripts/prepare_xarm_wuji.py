"""Ready xArm and prepare Wuji with one operator confirmation, using one session."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.xarm import arming  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default=os.environ.get('CUBE_SESSION'),
                        help='Session directory or operator-input.json (default: $CUBE_SESSION)')
    parser.add_argument('--minutes', type=float, default=10,
                        help='Arming duration in minutes (default: 10)')
    parser.add_argument('--yes', action='store_true',
                        help='Operator already confirmed the complete sequence (used by the dashboard)')
    args = parser.parse_args(argv)
    if not args.session:
        parser.error('set CUBE_SESSION or supply --session; no session is selected automatically')
    if not math.isfinite(args.minutes) or args.minutes <= 0:
        parser.error('--minutes must be finite and positive')
    root = Path(args.session).expanduser().resolve()
    if root.is_file():
        root = root.parent
    try:
        options = json.loads((root / 'operator-input.json').read_text())['backend_options']
        if options.get('end_effector', {}).get('type') != 'wuji_hand2' or not options.get('xarm_ip'):
            raise ValueError('session must configure xArm and wuji_hand2')
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        parser.error(f'invalid session: {exc}')

    env = {**os.environ, 'RAX_PYTHON': sys.executable}

    def run(*command, capture=False):
        return subprocess.run(command, cwd=PROJECT, env=env, check=True,
                              text=True, capture_output=capture)

    # This only queries the running owner. Never start a second hand SDK client.
    state = json.loads(run('bash', 'scripts/wuji', 'status', '--session', str(root), capture=True).stdout)
    if state.get('mode') in ('executing', 'preparing', 'stopping'):
        parser.error(f"hand is {state['mode']}; wait for it to stop before preparing")

    print(f'Session: {root}\nxArm: {options["xarm_ip"]}\n'
          f'Preparation covers arm readiness, {args.minutes:g} minutes of arming, '
          'and enabling the hand to hold its measured posture.\n'
          'Check the arm workspace and whole-hand sweep; keep a person at the emergency stop.',
          flush=True)

    # Pin xarm ready to this session's options, regardless of newer sessions or
    # a stale RAX_BACKEND_OPTIONS in the caller's shell. The file is mode 0600.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', prefix='rax-ready-') as saved:
        json.dump(options, saved)
        saved.flush()
        env['RAX_BACKEND_OPTIONS'] = saved.name
        try:
            run('bash', 'scripts/xarm', 'ready', *(['--yes'] if args.yes else []))
            # READY already confirmed the complete sequence above.
            run(sys.executable, 'scripts/arm_robot.py', str(root), 'arm',
                '--minutes', str(args.minutes), '--yes')
            run('bash', 'scripts/wuji', 'prepare', '--session', str(root))
            if not arming.read(root)['armed']:
                raise RuntimeError('session disarmed during preparation')
        except BaseException:
            print('Preparation did not complete; revoking session arming and requesting hand stop.',
                  file=sys.stderr, flush=True)
            try:
                arming.write(root, False, reason='combined preparation failed or was cancelled')
            finally:
                try:
                    run('bash', 'scripts/wuji', 'stop', '--session', str(root))
                except Exception as exc:
                    print(f'Hand stop failed: {exc}; check the physical stop and hand status.', file=sys.stderr)
            raise
    print(f'xArm ready; session armed for {args.minutes:g} minutes; Wuji prepared.\n'
          'Answer the pending operator request separately when its prerequisites are satisfied.')


if __name__ == '__main__':
    main()
