"""Operator-only lifecycle commands for an existing RAX Wuji owner."""
import argparse
import json
from pathlib import Path

from .client import operation, request, socket_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('status', 'prepare', 'disable', 'stop', 'reconnect'))
    parser.add_argument('--session', type=Path, required=True, help='Session directory printed by configure_session')
    args = parser.parse_args()
    if args.action == 'status':
        result = request(socket_path(args.session), 'status')
    else:
        op = {'stop': 'emergency', 'reconnect': 'connect'}.get(args.action, args.action)
        result = operation(args.session, op)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
