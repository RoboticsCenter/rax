"""Read-only right-pair preflight and Piper session configuration; never enable or move.

    python scripts/piper.py check [--output /tmp/piper-check.json]
    python scripts/piper.py configure --task TASK --task-file task.md

Session arming remains scripts/arm_robot.py; it does not enable a disabled arm.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('check', 'configure'))
    parser.add_argument('--backend-options', type=Path, default=PROJECT / 'examples/piper-brainco.json')
    parser.add_argument('--output', type=Path)
    args, extra = parser.parse_known_args()
    if args.action == 'configure':
        if args.output:
            parser.error('--output is only for check')
        if any(item == '--robot' or item.startswith('--robot=') for item in extra):
            parser.error('This command configures only the Piper backend')
        return subprocess.call([sys.executable, str(PROJECT / 'scripts/configure_session.py'),
                                *extra, '--robot', 'piper', '--backend-options', str(args.backend_options)])
    if extra:
        parser.error('Unexpected check arguments: ' + ' '.join(extra))
    from services.piper.config import options
    from services.piper.transport import PairLease
    from services.piper.sdk import PiperArm
    from services.piper.hand import RevoHand
    from services.piper.camera import Camera
    report = {'time': datetime.now(timezone.utc).isoformat(), 'robot': 'piper', 'side': 'right',
              'motion_tested': False, 'physical_success': None, 'ok': False}
    arm = hand = camera = lease = None
    try:
        settings = options(json.loads(args.backend_options.read_text()))
        lease = PairLease(settings['arm_usb_serial'])
        arm = PiperArm(settings)
        report['arm'] = arm.state()
        with tempfile.TemporaryDirectory(prefix='rax-piper-check-') as folder:
            hand = RevoHand(settings, folder)
            report['hand'] = hand.state()
            camera = Camera({**settings, 'stream_enabled': False}, folder, lambda: {})
            timestamp, data = camera.grab()
            report['camera'] = {'serial': settings['cameras'][0]['serial'], 'received_at': timestamp,
                                'jpeg_bytes': len(data), 'camera_geometry': False}
            camera.close()
            camera = None
        report['ok'] = True
        report['motion_ready'] = False
        report['motion_blockers'] = ['Supervised motion qualification not performed by read-only preflight',
                                     'Operator session arming required']
        if report['arm']['enabled'] != [True] * 6:
            report['motion_blockers'].append('Arm motors are disabled; no enable attempted')
        if not report['arm']['firmware_verified']:
            report['motion_blockers'].append('Piper firmware profile is not verified')
        if not settings['motion_commissioned']:
            report['motion_blockers'].append('motion_commissioned is false')
        if not settings['tool_calibrated']:
            report['motion_blockers'].append('Hand TCP is uncalibrated; Cartesian motion unavailable')
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        for device in (camera, hand, arm, lease):
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    report.update(ok=False, close_error=str(exc))
    text = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
