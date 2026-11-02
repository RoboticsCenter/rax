#!/usr/bin/env python3
"""Read-only identity/state check. Never construct a motion backend or send motor writes."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def usb_inventory():
    result = []
    for directory in sorted(Path('/sys/bus/usb/devices').glob('*')):
        values = {key: (directory/key).read_text().strip()
                  for key in ('idVendor', 'idProduct', 'manufacturer', 'product', 'serial')
                  if (directory/key).is_file()}
        if values.get('idVendor') == '34b7':
            result.append({'usb_path': directory.name, **values})
    return result


def check_arm(options):
    from services.xarm.sdk import XArmDriver
    from services.xarm.backend import DEFAULTS
    from services.json_io import jsonable
    arm = None
    try:
        arm = XArmDriver({**DEFAULTS, **options})
        return {'ok': True, 'xarm_ip': options['xarm_ip'], 'state': jsonable(arm.state())}
    except Exception as exc:
        return {'ok': False, 'xarm_ip': options['xarm_ip'], 'error': str(exc)}
    finally:
        if arm is not None:
            arm.close()  # Disconnect only; no hold, enable or mode writes.


def check_hand(options, root):
    from services.revo2.client import RevoHand
    hand = None
    try:
        hand = RevoHand(SimpleNamespace(root=root, backend_options=options))
        return {'ok': True, 'state': hand.status()}
    except Exception as exc:
        return {'ok': False, 'error': str(exc)}
    finally:
        if hand is not None:
            hand.close()  # A read-only worker has no active motor command to stop.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--options', type=Path, help='bimanual backend-options JSON (not a session file)')
    parser.add_argument('--inventory-only', action='store_true', help='read USB metadata without opening any hardware')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = {'usb_adapters': usb_inventory(), 'motion_commands_sent': False}
    if not args.inventory_only:
        if args.options is None:
            parser.error('--options is required unless --inventory-only is selected')
        from services.xarm.bimanual_config import validate_config
        settings = validate_config(json.loads(args.options.read_text()))
        # Open hands sequentially: discovery probes candidate adapters and retains
        # the exact-serial match; each read-only check releases it before the next.
        with tempfile.TemporaryDirectory(prefix='rax-revo-readonly-') as folder:
            root = Path(folder)
            report['arms'] = {side: check_arm(cfg) for side, cfg in settings.items()}
            report['hands'] = {side: check_hand(cfg, root) for side, cfg in settings.items()}
        report['ok'] = all(v['ok'] for group in ('arms', 'hands') for v in report[group].values())
    text = json.dumps(report, indent=2)
    if args.output:
        args.output.write_text(text+'\n')
    print(text)
    return 0 if report.get('ok', True) else 1


if __name__ == '__main__':
    raise SystemExit(main())
