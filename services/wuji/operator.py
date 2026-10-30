"""Operator-only lifecycle commands for an existing RAX Wuji owner."""
import argparse
import json
from pathlib import Path
import sys

from .client import operation, request, socket_path
from .preparation import recover_cooled_thermal


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('status', 'prepare', 'disable', 'stop', 'reconnect', 'recover-thermal'))
    parser.add_argument('--session', type=Path, required=True, help='Session directory printed by configure_session')
    args = parser.parse_args(argv)
    if args.action == 'status':
        result = request(socket_path(args.session), 'status')
    elif args.action == 'recover-thermal':
        result = recover_cooled_thermal(args.session)
    else:
        op = {'stop': 'emergency', 'reconnect': 'connect'}.get(args.action, args.action)
        result = operation(args.session, op)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError) as exc:
        print(f'Wuji operation failed: {exc}', file=sys.stderr)
        sys.exit(1)
