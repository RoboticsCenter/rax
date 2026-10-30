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
from services.wuji.preparation import PreparationBlocked, preparation_action  # noqa: E402


def hand_ready(state):
    """Only reuse a fresh, fault-free hold controlled by this session's owner."""
    return (state.get('mode') == 'holding' and state.get('fresh') is True
            and state.get('enabled') is True and state.get('owns_control') is True
            and state.get('target_positions') is not None
            and not state.get('fault') and not state.get('faults'))


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
                              text=True, stdout=subprocess.PIPE if capture else None, stderr=subprocess.PIPE)

    # This only queries the running owner. Never start a second hand SDK client.
    def hand_status():
        return json.loads(run('bash', 'scripts/wuji', 'status', '--session', str(root), capture=True).stdout)

    state = hand_status()
    if state.get('mode') in ('executing', 'preparing', 'enabling', 'stopping'):
        parser.error(f"hand is {state['mode']}; wait for it to stop before preparing")

    print(f'Session: {root}\nxArm: {options["xarm_ip"]}\n'
          f'Preparation covers arm readiness, {args.minutes:g} minutes of arming, '
          'and preserving a healthy hand hold or preparing a disabled hand at its measured posture.\n'
          'Check the arm workspace and whole-hand sweep; keep a person at the emergency stop.',
          flush=True)

    # Pin xarm ready to this session's options, regardless of newer sessions or
    # a stale RAX_BACKEND_OPTIONS in the caller's shell. The file is mode 0600.
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', prefix='rax-ready-') as saved:
        json.dump(options, saved)
        saved.flush()
        env['RAX_BACKEND_OPTIONS'] = saved.name
        try:
            action = 'prepare' if hand_ready(state) else preparation_action(state, options['end_effector'])
            if action in ('reconnect', 'reconnect-stall'):
                kind = 'stall' if action == 'reconnect-stall' else 'thermal'
                arming.write(root, False, reason=f'operator-requested {kind} recovery before preparation')
                detail = 'Device stall has cleared' if kind == 'stall' else 'Thermal fault has cooled'
                print(f'{detail}: reconnecting the disabled hand once before preparation.', flush=True)
                run('bash', 'scripts/wuji', f'recover-{kind}', '--session', str(root), capture=True)
                state = hand_status()
                if state.get('fault') or state.get('faults') or not state.get('fresh') or state.get('enabled'):
                    raise PreparationBlocked('Hand recovery did not establish a healthy disabled state.')
            run('bash', 'scripts/xarm', 'ready', *(['--yes'] if args.yes else []))
            # READY already confirmed the complete sequence above.
            run(sys.executable, 'scripts/arm_robot.py', str(root), 'arm',
                '--minutes', str(args.minutes), '--yes')
            # Re-read after confirmation and arming: the owner may have disabled
            # the hand while the operator was answering the prompt. Conversely,
            # an already healthy hold must not be prepared a second time.
            state = hand_status()
            if hand_ready(state):
                print('Wuji already prepared; preserving its current hold.', flush=True)
            else:
                if state.get('enabled') or state.get('mode') in ('executing', 'preparing', 'enabling', 'stopping'):
                    raise RuntimeError('hand is enabled or busy without a healthy hold; cannot prepare')
                run('bash', 'scripts/wuji', 'prepare', '--session', str(root))
                if not hand_ready(hand_status()):
                    raise RuntimeError('hand preparation did not establish a fresh, fault-free hold')
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


def cli():
    try:
        main()
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or f'Preparation step failed: {" ".join(map(str, exc.cmd))}').strip()
        print(f'Preparation blocked: {detail}', file=sys.stderr)
        return 1
    except (RuntimeError, OSError, ValueError) as exc:
        print(f'Preparation blocked: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(cli())
