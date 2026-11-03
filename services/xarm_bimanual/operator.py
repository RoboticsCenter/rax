"""Operator preparation and direct stop for both assemblies; never MCP tools."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
import os
import tempfile
import threading
import time

from services.xarm import arming
from services.xarm.backend import DEFAULTS
from services.xarm.sdk import XArmDriver
from services.wuji.client import operation, request, socket_path, recorded_status
from services.wuji.preparation import preparation_action, recover_cleared_stall, recover_cooled_thermal
from services.wuji.partial import control_enabled
from .config import SIDES, validate_options, arm_options

PROJECT = Path(__file__).resolve().parents[2]


def ready_hand(state):
    return (state.get('mode') == 'holding' and state.get('fresh') is True
            and control_enabled(state, state) and state.get('owns_control') is True
            and state.get('target_positions') is not None
            and not state.get('fault') and not state.get('faults'))


def stop(root, options, *, driver_factory=XArmDriver):
    """Every transport gets its own stop attempt, even if another transport blocks."""
    root = Path(root) if root is not None else None
    disarm_error = None
    if root is not None:
        try:
            arming.write(root, False, reason='operator stopped bimanual assembly')
        except Exception as exc:
            disarm_error = str(exc)
    def arm_stop(side):
        arm = None
        try:
            arm = driver_factory({**DEFAULTS, **arm_options(options, side)})
            arm.hold()
            return {'requested': True, 'sdk_returned': True, 'confirmed': False}
        finally:
            if arm is not None:
                arm.close()
    def hand_stop(side):
        if root is None:
            raise RuntimeError('no session hand owner is selected')
        return operation(root, 'emergency', side=side)
    actions = {f'{side}_arm': (arm_stop, side) for side in SIDES}
    actions.update({f'{side}_hand': (hand_stop, side) for side in SIDES})
    reports, lock = {}, threading.Lock()
    def attempt(name, action):
        try:
            result = action[0](action[1])
        except Exception as exc:
            result = {'requested': True, 'confirmed': False, 'error': str(exc)}
        with lock:
            reports[name] = result
    threads = [threading.Thread(target=attempt, args=item, daemon=True) for item in actions.items()]
    for thread in threads:
        thread.start()
    deadline = time.monotonic()+10
    for thread in threads:
        thread.join(max(0., deadline-time.monotonic()))
    with lock:
        return {name: {**reports.get(name, {'requested': True, 'confirmed': False, 'pending': True}),
                       **({'disarm_error': disarm_error, 'confirmed': False} if disarm_error else {})}
                for name in actions}


def status(root, options, *, driver_factory=XArmDriver):
    """Read each arm and the matching session's two hand snapshots independently."""
    def read(side):
        hand = recorded_status(root, side=side) if root else {
            'fresh': False, 'mode': 'unavailable', 'fault': 'No matching bimanual session snapshot'}
        report = {'hand': {**hand, 'ok': bool(hand.get('fresh')) and not hand.get('fault') and not hand.get('faults')}}
        arm = None
        try:
            arm = driver_factory({**DEFAULTS, **arm_options(options, side)})
            state = arm.state()
            report['arm'] = {**{k: state.get(k) for k in ('state', 'mode', 'error_code', 'warn_code')},
                **{k: [round(float(v), 5) for v in state[k]] for k in ('tcp_position', 'tcp_quaternion', 'joints')},
                'ok': True, 'xarm_ip': options['arms'][side]['xarm_ip']}
        except Exception as exc:
            report['arm'] = {'ok': False, 'error': str(exc), 'xarm_ip': options['arms'][side]['xarm_ip']}
        finally:
            if arm is not None:
                try:
                    arm.close()
                except Exception as exc:
                    report['arm']['close_error'] = str(exc)
        return report
    with ThreadPoolExecutor(max_workers=2) as pool:
        return {'arms': dict(zip(SIDES, pool.map(read, SIDES))), 'bimanual': True}


def prepare(root, options, *, minutes, confirmed=False):
    """Pin readiness to both configured arms; prepare each existing hand owner once."""
    root = Path(root)
    options = validate_options(options)
    states = {s: request(socket_path(root, s), 'status') for s in SIDES}
    actions = {}
    for side in SIDES:
        state = states[side]
        if state.get('mode') in ('executing', 'preparing', 'enabling', 'stopping'):
            raise RuntimeError(f'{side} hand is busy; wait before preparation')
        actions[side] = 'preserve' if ready_hand(state) else preparation_action(
            state, options['arms'][side]['end_effector'])
    print('Prepare both xArms and both Wuji hands at their measured postures. '
          'Verify both arm workspaces, whole-hand sweeps and a person at the emergency stop.', flush=True)
    if not confirmed:
        with open('/dev/tty', 'r+') as tty:
            tty.write('Type READY to prepare both assemblies: ')
            tty.flush()
            if tty.readline().strip() != 'READY':
                raise RuntimeError('preparation cancelled')
    try:
        for side, action in actions.items():
            if action in ('reconnect', 'reconnect-stall'):
                arming.write(root, False, reason=f'operator recovery of {side} hand')
                recover = recover_cleared_stall if action == 'reconnect-stall' else recover_cooled_thermal
                recover(root, side=side)
        for side in SIDES:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.json', prefix=f'rax-{side}-ready-') as saved:
                json.dump(arm_options(options, side), saved)
                saved.flush()
                subprocess.run(['bash', str(PROJECT/'scripts/xarm'), 'ready', '--yes'], cwd=PROJECT,
                    env={**os.environ, 'RAX_PYTHON': sys.executable, 'RAX_BACKEND_OPTIONS': saved.name}, check=True)
        arming.write(root, True, minutes=minutes, by='bimanual-operator')
        for side in SIDES:
            state = request(socket_path(root, side), 'status')
            if ready_hand(state):
                continue
            if state.get('enabled') or state.get('mode') in ('executing', 'preparing', 'enabling', 'stopping'):
                raise RuntimeError(f'{side} hand is not healthy and disabled')
            preparation_action(state, options['arms'][side]['end_effector'])
            operation(root, 'prepare', side=side)
            if not ready_hand(request(socket_path(root, side), 'status')):
                raise RuntimeError(f'{side} hand preparation did not establish a healthy hold')
        if not arming.read(root)['armed']:
            raise RuntimeError('session disarmed during preparation')
    except BaseException:
        stop(root, options)
        raise
    print(f'Both assemblies prepared; session armed for {minutes:g} minutes.', flush=True)
