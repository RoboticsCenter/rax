"""Operator-cleared, hand-only live qualification of the two current-control MCP tools.

Default is a hardware-free plan. --execute requires an empty, clear hand and a
person at the stop. Exit RAX first. Starts the production frontend, never an agent.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from auto_research_agent.api.runtime import Context
from services.controller.config import Configuration
from services.wuji.client import operation, request, socket_path
from services.xarm import arming

PLAN = {
    'arm_motion': False,
    'joint': 'middle S3 (zero-based index 10)',
    'effort_a': [0.0, 0.05, 0.0, -0.05, 0.0],
    'ramp_s': 0.5, 'hold_s': 1.0,
    'impedance': {'kp': 7.2, 'kd': 0.14, 'current_limit_a': 1.0},
    'baseline': {'kp': 8.0, 'kd': 0.15, 'current_limit_a': 1.5},
    'max_joint_excursion_rad': 0.03,
    'finish': 'Restore baseline, disarm, verify disabled and zero commanded effort.',
    'failure': 'Disarm and stop; no retry or rearming. Preserve evidence.',
}


def save(root, name, value):
    (root / name).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def healthy(state):
    return state['fresh'] and not state['fault'] and not state['faults']


def qualify(root, sock):
    report = {'plan': PLAN, 'started_unix': time.time(), 'calls': [], 'ok': False}
    stopped, monitoring = threading.Event(), threading.Event()
    reference = []
    guard_errors = []

    def state():
        return request(socket_path(root), 'status')

    def record():
        try:
            with (root / 'hand-trace.jsonl').open('x') as stream:
                while not stopped.is_set():
                    sample = state()
                    stream.write(json.dumps({'at': time.monotonic(), 'state': sample}) + '\n')
                    if monitoring.is_set():
                        require(healthy(sample) and sample['enabled'], 'hand became unhealthy/disabled')
                        excursion = max(abs(q - q0) for q, q0 in zip(sample['positions'], reference))
                        require(excursion <= PLAN['max_joint_excursion_rad'],
                                f'joint excursion {excursion:.6f} rad exceeds test bound')
                    stopped.wait(.01)
        except BaseException as exc:
            guard_errors.append(repr(exc))
            arming.write(root, False, reason='effort qualification monitor stopped')
            try:
                operation(root, 'emergency', timeout=5)
            except Exception as stop_error:
                guard_errors.append(repr(stop_error))

    recorder = threading.Thread(target=record, daemon=True)
    try:
        with Context(str(sock)) as ctx:
            arm, hand = ctx.value('xarm_status'), ctx.value('wuji_status')
            report['before'] = {'arm': arm, 'hand': hand}
            require(arm['controller_state'] in (4, 5) and arm['error_code'] == 0,
                    'arm must already be stopped and error-free')
            require(not arm['motion']['armed'], 'qualification must start disarmed')
            require(healthy(hand) and not hand['enabled'], 'fresh healthy disabled hand required')
            require(hand['diagnostics']['states'] == [1] * 20, 'all joints must be Ready')
            require(hand.get('command_effort_a') == [0.0] * 20, 'new effort owner not ready')
            require(hand['profile_revision'] == 'right-hand2-scurve-20261028-v13', 'expected v13')
            baseline = {key: [value] * 20 for key, value in PLAN['baseline'].items()}
            require(hand['profile_control_parameters'] == baseline, 'expected v13 gains/current limits')
            recorder.start()
            arming.write(root, True, minutes=2, by='operator-cleared-effort-qualification')
            report['prepare'] = operation(root, 'prepare', timeout=15)
            prepared = state()
            require(healthy(prepared) and prepared['enabled'], 'preparation did not establish control')
            reference[:] = prepared['positions']
            report['prepared'] = prepared
            monitoring.set()
            time.sleep(1)

            def call(name, **args):
                require(not guard_errors, str(guard_errors))
                item = {'tool': name, 'arguments': args, 'started': time.monotonic()}
                report['calls'].append(item)
                print('CALL', name, json.dumps(args), flush=True)
                try:
                    item['result'] = ctx.value(name, **args)
                    require(item['result'].get('ok', True), str(item['result']))
                    item['state'] = state()
                    require(healthy(item['state']), 'unhealthy hand after MCP call')
                    require(not guard_errors, str(guard_errors))
                    return item['result']
                finally:
                    item['finished'] = time.monotonic()
                    save(root, 'qualification-report.json', report)

            for value in PLAN['effort_a']:
                currents = [0.0] * 20
                currents[10] = value
                call('wuji_set_effort', effort_a=currents, duration_s=PLAN['ramp_s'])
                require(state()['command_effort_a'] == currents, 'commanded effort mismatch')
                time.sleep(PLAN['hold_s'])
            softened = deepcopy(baseline)
            for key, value in PLAN['impedance'].items():
                softened[key][10] = value
            result = call('wuji_set_impedance', **softened)
            require(result['readback_verified'], 'impedance readback not verified')
            require(state()['control_parameters'] == softened, 'active impedance mismatch')
            time.sleep(1)
            result = call('wuji_set_impedance', **baseline)
            require(result['readback_verified'], 'baseline readback not verified')
            require(state()['control_parameters'] == baseline, 'baseline restoration mismatch')
            time.sleep(1)
            require(not guard_errors, str(guard_errors))
            monitoring.clear()
            report['disarm_at'] = time.monotonic()
            arming.write(root, False, reason='effort qualification disarm check')
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                sample = state()
                if (sample['fresh'] and not sample['enabled'] and
                        sample['diagnostics']['states'] == [1] * 20 and
                        sample['command_effort_a'] == [0.0] * 20 and
                        max(map(abs, sample['velocity_rad_s'])) <= .05):
                    report['stop_confirmed_after_s'] = time.monotonic() - report['disarm_at']
                    report['stopped'] = sample
                    break
                time.sleep(.01)
            require('stopped' in report, 'disabled/zero-current state was not confirmed')
            report['arm_after'] = ctx.value('xarm_status')
            require(report['arm_after']['tcp_position'] == arm['tcp_position'], 'arm changed position')
            report['ok'] = True
    except BaseException as exc:
        report['error'] = repr(exc)
    finally:
        monitoring.clear()
        arming.write(root, False, reason='effort qualification complete')
        try:
            report['cleanup_stop'] = operation(root, 'emergency', timeout=5)
            report['final_hand'] = state()
            require(report['final_hand']['fresh'] and not report['final_hand']['enabled'] and
                    report['final_hand']['diagnostics']['states'] == [1] * 20 and
                    report['final_hand']['command_effort_a'] == [0.0] * 20,
                    'final disable/zero effort not confirmed')
        except Exception as exc:
            report['cleanup_error'] = repr(exc)
            report['ok'] = False
        stopped.set()
        if recorder.ident:
            recorder.join(timeout=2)
        report['guard_errors'] = guard_errors
        if guard_errors:
            report['ok'] = False
        report['finished_unix'] = time.time()
        save(root, 'qualification-report.json', report)
        print(json.dumps({k: report[k] for k in ('ok', 'error', 'cleanup_error', 'guard_errors',
                                                'stop_confirmed_after_s') if k in report}), flush=True)
    return report['ok']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-session', type=Path, required=True)
    parser.add_argument('--execute', action='store_true', help='Operator has confirmed current physical clearance')
    args = parser.parse_args()
    source = args.source_session.resolve()
    config = Configuration.load(source / 'operator-input.json')
    if not args.execute:
        print(json.dumps(PLAN, indent=2))
        return
    with (source / 'launch.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Exit the running RAX session before testing') from None
        # Device locks also enforce exclusivity; this check prevents unnecessary startups.
        require(not socket_path(source).exists(), 'source hand service still owns its socket')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        root = source.parent / f'{stamp}_wuji_effort_qualification_{uuid.uuid4().hex[:6]}'
        root.mkdir(mode=0o700)
        value = config.to_dict()
        value.update(root=str(root), task='wuji_effort_qualification', session_seconds=600,
                     task_description=__doc__, subagents=0)
        config = Configuration.from_dict(value)
        save(root, 'operator-input.json', config.to_dict())
        save(root, 'qualification-plan.json', PLAN)
        sock = Path(f'/tmp/rax-effort-qualification-{os.getpid()}.sock')
        print('EVIDENCE', root, flush=True)
        with (root / 'frontend.log').open('w') as log:
            proc = subprocess.Popen([sys.executable, '-m', 'services.controller.frontend',
                                     '--config', str(root / 'operator-input.json'), '--socket', str(sock)],
                                    cwd=PROJECT, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    require(proc.poll() is None, 'frontend exited; inspect evidence/frontend.log')
                    if sock.exists():
                        sample = request(socket_path(root), 'status')
                        if sample['fresh'] and sample['mode'] == 'disabled':
                            break
                    time.sleep(.1)
                else:
                    raise RuntimeError('fresh disabled owner not ready within 30s')
                ok = qualify(root, sock)
            finally:
                arming.write(root, False, reason='qualification frontend shutdown')
                proc.terminate()
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    raise RuntimeError('frontend did not exit; inspect hardware before reconnecting') from None
        raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
