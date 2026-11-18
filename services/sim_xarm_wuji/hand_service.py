"""The single-writer Wuji hand controller (services/wuji/controller.py, unchanged) on a
simulated hand: the same `serve()` the real hand service runs, with RAX's WujiDriver
given the SDK mimic (wuji_sdk.py) instead of the vendor module.

    python -m services.sim_xarm_wuji.hand_service --config CFG --socket PATH \
        --parent-pid PID --side left|right --sim-socket SIM_SOCKET

Started by the sim backend's hand client exactly where the real backend starts
`python -m services.wuji.controller`.
"""
import argparse
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--side', choices=('left', 'right'))
    parser.add_argument('--sim-socket', type=Path, required=True)
    args = parser.parse_args()
    os.environ['WUJI_TELEMETRY_DISABLE'] = '1'
    from services.controller.config import Configuration
    from services.wuji.config import validate_config
    from services.wuji.controller import serve
    from services.wuji.sdk import DeviceLease, WujiDriver
    from . import wuji_sdk
    from .shm import token_for
    wuji_sdk.configure(args.sim_socket)
    session = Configuration.load(args.config)
    config = validate_config(session.backend_options['end_effector'])
    # One owner per simulated hand, as per real hand, but per session: simulated rigs of
    # different sessions are different devices that share serial names.
    token = token_for(session.root)
    driver = WujiDriver(config, sdk=wuji_sdk,
                        lease_factory=lambda serial, paths: DeviceLease(f'{serial}-sim-{token}', paths))
    serve(config, session.root, args.socket, driver=driver, parent_pid=args.parent_pid, side=args.side)


if __name__ == '__main__':
    main()
