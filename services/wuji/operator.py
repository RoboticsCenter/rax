"""Operator-only lifecycle commands for an existing RAX Wuji owner."""
import argparse
import json
from pathlib import Path
import sys

from .client import operation, request, socket_path
from .preparation import recover_cleared_stall, recover_cooled_thermal


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('status', 'prepare', 'disable', 'stop', 'reconnect',
                                           'recover-thermal', 'recover-stall'))
    parser.add_argument('--session', type=Path, required=True, help='Session directory printed by configure_session')
    parser.add_argument('--hand', choices=('left', 'right'), help='Required for a bimanual session')
    args = parser.parse_args(argv)
    from services.controller.config import Configuration
    config = Configuration.load(args.session/'operator-input.json')
    if (config.robot == 'xarm_bimanual') != (args.hand is not None):
        parser.error('bimanual sessions require --hand left|right; single-hand sessions omit it')
    if args.action == 'status':
        result = request(socket_path(args.session, args.hand), 'status')
    elif args.action == 'recover-thermal':
        result = recover_cooled_thermal(args.session, side=args.hand)
    elif args.action == 'recover-stall':
        result = recover_cleared_stall(args.session, side=args.hand)
    else:
        op = {'stop': 'emergency', 'reconnect': 'connect'}.get(args.action, args.action)
        result = operation(args.session, op, side=args.hand)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError) as exc:
        print(f'Wuji operation failed: {exc}', file=sys.stderr)
        sys.exit(1)
