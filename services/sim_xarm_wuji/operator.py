"""Operator actions on the simulated rig: the steps a person does on the real one.

    python -m services.sim_xarm_wuji.operator prepare --session SESSION [--minutes M]
    python -m services.sim_xarm_wuji.operator stop    --session SESSION
    python -m services.sim_xarm_wuji.operator status  --session SESSION
    python -m services.sim_xarm_wuji.operator reset   --session SESSION [--seed N] [--scene NAME]

`prepare` = scripts/xarm-wuji-ready on the rig: ready both simulated controllers (clear
errors, position mode, enable motion), arm the session, prepare each hand through its
running hand service. services/xarm_bimanual/operator.py routes here for sessions whose
options carry `simulation`, so the dashboard and scripts/xarm-wuji-ready work too.
Nothing here can reach real hardware: it talks only to the session's sim server.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from services.xarm import arming

from .rpc import Client, socket_path

SIDES = ('left', 'right')


def driver_factory(root):
    """An XArmDriver factory (services/xarm_bimanual/operator.py signature) on the sim server."""
    from .xarm_api import SimXArmDriver
    path = socket_path(root)
    return lambda options: SimXArmDriver(options, path)


def ready_arm(root, options, side):
    """scripts/xarm ready, on the simulated controller box."""
    from .xarm_api import SimXArmAPI
    arm = SimXArmAPI(options['arms'][side]['xarm_ip'], is_radian=True, sim_socket=socket_path(root))
    try:
        for call, args in ((arm.clean_error, ()), (arm.clean_warn, ()), (arm.set_mode, (0,)),
                           (arm.set_state, (0,)), (arm.motion_enable, (True,)), (arm.set_state, (0,))):
            code = call(*args)
            if code != 0:
                raise RuntimeError(f'{side} simulated xArm: {call.__name__} returned code {code}')
    finally:
        arm.disconnect()


def ready_hand(state):
    from services.wuji.partial import control_enabled
    return (state.get('mode') == 'holding' and state.get('fresh') is True
            and control_enabled(state, state) and state.get('owns_control') is True
            and state.get('target_positions') is not None
            and not state.get('fault') and not state.get('faults'))


def prepare(root, options, *, minutes=None, timeout=20.0):
    """Ready both arms, arm the session, prepare both hands; returns the hands' states."""
    from services.wuji.client import operation, request, socket_path as hand_socket
    from services.wuji.preparation import preparation_action
    root = Path(root)
    for side in SIDES:
        ready_arm(root, options, side)
    arming.write(root, True, minutes=minutes, by='simulation-operator', reason='simulated rig prepared')
    states = {}
    for side in SIDES:
        state = request(hand_socket(root, side), 'status')
        if not ready_hand(state):
            if state.get('enabled') or state.get('mode') in ('executing', 'preparing', 'enabling', 'stopping'):
                raise RuntimeError(f'{side} simulated hand is busy or enabled without a healthy hold')
            action = preparation_action(state, options['arms'][side]['end_effector'])
            if action != 'prepare':
                operation(root, 'connect', side=side, timeout=timeout)
            operation(root, 'prepare', side=side, timeout=timeout)
            state = request(hand_socket(root, side), 'status')
            if not ready_hand(state):
                raise RuntimeError(f'{side} simulated hand preparation did not establish a healthy hold')
        states[side] = state
    return states


def stop(root, options):
    from services.xarm_bimanual.operator import stop as bimanual_stop
    return bimanual_stop(root, options, driver_factory=driver_factory(root))


def status(root, options):
    from services.xarm_bimanual.operator import status as bimanual_status
    return bimanual_status(root, options, driver_factory=driver_factory(root))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('prepare', 'stop', 'status', 'reset', 'stats'))
    parser.add_argument('--session', required=True)
    parser.add_argument('--minutes', type=float)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--scene')
    args = parser.parse_args(argv)
    root = Path(args.session).expanduser().resolve()
    if root.is_file():
        root = root.parent
    options = json.loads((root/'operator-input.json').read_text())['backend_options']
    from .config import complete
    options = complete(options)
    if args.action == 'prepare':
        value = {side: state.get('mode') for side, state in prepare(root, options, minutes=args.minutes).items()}
    elif args.action == 'stop':
        value = stop(root, options)
    elif args.action == 'status':
        value = status(root, options)
    elif args.action == 'reset':
        client = Client(socket_path(root))
        value = (client.call('sim.load_scene', scene=args.scene, seed=args.seed) if args.scene
                 else client.call('sim.reset', seed=args.seed))
    else:
        value = Client(socket_path(root)).call('sim.stats')
    print(json.dumps(value, indent=1, default=str))


if __name__ == '__main__':
    main()
