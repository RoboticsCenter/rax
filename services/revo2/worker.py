"""Isolated native BrainCo SDK owner. No SDK globals are shared between hands."""
import asyncio
import json
import math
import os
import sys
import time

from .config import MOTORS, validate_config, validate_tool
from .damiao import DamiaoBus, native_context, register_request
from services.xarm import arming


class HandHardware:
    def __init__(self, settings, root, *, sdk=None, client=None, context=None):
        self.cfg = validate_config(settings)
        self.root = root
        if sdk is None:
            from importlib.metadata import version
            if version('bc-stark-sdk') != '2.0.9':
                raise RuntimeError('Install the qualified API version bc-stark-sdk==2.0.9')
            from bc_stark_sdk import main_mod as sdk
        self.sdk = sdk
        self.loop = asyncio.new_event_loop()
        self.bus = None
        self.client = client
        self.info = {}
        self.active = False
        self.closed = False
        self.fault = None
        try:
            if client is not None:
                self.inspect()
            elif self.cfg['transport'] == 'modbus':
                self.client = self.run(sdk.modbus_open, self.cfg['port'],
                                       sdk.Baudrate.from_int(self.cfg['baudrate']))
                self.inspect()
            else:
                context = context or native_context(self.cfg['damiao_library'])
                errors = []
                for index in range(context.count):
                    try:
                        self.bus = DamiaoBus(context, index, self.cfg)
                        sdk.set_can_tx_callback(self.bus.send)
                        sdk.set_can_rx_callback(self.bus.receive)
                        self.client = sdk.init_device_handler(sdk.StarkProtocolType.CanFd, 1)
                        self.inspect()
                        break
                    except Exception as exc:
                        errors.append(f'adapter {index}: {exc}')
                        if self.bus is not None:
                            self.bus.close()
                            self.bus = None
                        self.client = None
                if self.client is None:
                    raise RuntimeError('No matching Revo hand: ' + '; '.join(errors))
        except BaseException:
            self.close()
            raise

    def run(self, method, *args, write=False):
        async def invoke():
            return await asyncio.wait_for(method(*args), timeout=2.)
        value = self.loop.run_until_complete(invoke())
        if self.bus and (self.bus.error or self.bus.transaction_error):
            raise RuntimeError(self.bus.error or self.bus.transaction_error)
        if value is None and not write:
            raise RuntimeError('BrainCo SDK returned no feedback')
        if write and value is False:
            raise RuntimeError('BrainCo SDK rejected the request')
        return value

    def read(self, method):
        return self.run(getattr(self.client, method), self.cfg['hand_id'])

    def inspect(self):
        info = self.read('get_device_info')
        if info.serial_number != self.cfg['serial'] or info.firmware_version != self.cfg['firmware']:
            raise RuntimeError('BrainCo serial or firmware mismatch')
        if not info.uses_revo2_motor_api():
            raise RuntimeError('Device does not expose the Revo 2 motor API')
        if self.cfg['touch'] and not info.uses_revo2_touch_api():
            raise RuntimeError('Configured Touch API is unavailable on this device')
        mode = self.read('get_finger_unit_mode')
        currents = list(self.read('get_finger_protected_currents'))
        turbo = self.read('get_turbo_mode_enabled')
        if (int(mode) != int(self.sdk.FingerUnitMode.Normalized)
                or currents != self.cfg['protected_currents_ma'] or bool(turbo)):
            raise RuntimeError('Revo units, protection currents or turbo differs from configuration; no settings changed')
        self.info = {'serial': info.serial_number, 'firmware': info.firmware_version,
                     'handedness': self.cfg['handedness'], 'hand_id': self.cfg['hand_id'],
                     'transport': self.cfg['transport'], 'units': 'normalized_0_1000',
                     'motor_order': list(MOTORS), 'protected_currents_ma': currents,
                     'turbo': False, 'driver': 'bc-stark-sdk',
                     'adapter_index': self.bus.index if self.bus else None}
        self.state()

    def state(self):
        status = self.read('get_motor_status')
        value = {'positions': list(status.positions), 'speeds': list(status.speeds),
                 'currents': list(status.currents), 'motor_states': [int(v) for v in status.states]}
        if (any(len(v) != 6 for v in value.values())
                or any(not isinstance(x, (int, float)) or not math.isfinite(x)
                       for values in value.values() for x in values)
                or any(not 0 <= p <= 1000 for p in value['positions'])
                or any(abs(x) > 1000 for k in ('speeds', 'currents') for x in value[k])
                or any(x not in (0, 1, 2) for x in value['motor_states'])):
            raise RuntimeError('Invalid or faulted Revo feedback')
        result = {**self.info, **value, 'fresh': True, 'fault': self.fault, 'time': time.time(),
                'limits': {k: self.cfg[k] for k in ('target_min', 'target_max', 'max_step',
                                                   'max_speed', 'max_current')},
                'postures': self.cfg['postures']}
        from pathlib import Path
        from services.json_io import write_private_json
        write_private_json(Path(self.root)/'operator'/f'revo-{self.cfg["handedness"]}-state.json', result)
        return result

    def require_armed(self):
        if not arming.read(self.root)['armed']:
            raise PermissionError('Revo motion is not armed or was disarmed')
        if self.fault:
            raise RuntimeError(f'Revo control fault requires operator reconnection: {self.fault}')

    def write(self, method, address, values, *args):
        if self.bus:
            self.bus.authorized_write = register_request(self.cfg['hand_id'], address, values)
            self.bus.write_acknowledged = self.bus.write_sent = False
        try:
            self.run(getattr(self.client, method), self.cfg['hand_id'], *args, write=True)
            if self.bus and not self.bus.write_acknowledged:
                raise RuntimeError('Revo motor write was not acknowledged; no retry')
        finally:
            if self.bus:
                self.bus.authorized_write = None

    def stationary(self, state):
        return max(map(abs, state['speeds'])) <= self.cfg['stopped_speed'] and 1 not in state['motor_states']

    def hold(self):
        # Zero speed is a native stop command; measuring a position alone cannot
        # cancel a running current/speed controller while feedback is delayed.
        self.write('set_finger_speeds', 1034, [0]*6, [0]*6)
        state = self.state()
        target = [int(round(v)) for v in state['positions']]
        self.write('set_finger_positions_and_durations', 1010,
                   [v for p in target for v in (p, 100)], target, [100]*6)
        deadline = time.monotonic() + self.cfg['stop_timeout_s']
        anchor, since = None, None
        while time.monotonic() < deadline:
            state = self.state()
            stable = anchor is not None and max(abs(a-b) for a, b in zip(anchor, state['positions'])) <= 2
            if self.stationary(state) and stable:
                if since is not None and time.monotonic() - since >= self.cfg['settle_s']:
                    self.active = False
                    return {'requested': True, 'confirmed': True, 'held': True, 'state': state}
            else:
                anchor, since = state['positions'], time.monotonic() if self.stationary(state) else None
            time.sleep(self.cfg['poll_period_s'])
        return {'requested': True, 'confirmed': False, 'held': False, 'state': state}

    def execute(self, name, arguments):
        args = validate_tool(name, arguments, self.cfg)
        if name == 'revo_status':
            return self.state()
        if name == 'revo_hold':
            return self.hold()
        if name == 'revo_touch':
            enabled = int(self.read('get_touch_sensor_enabled'))
            # One fingertip per native request fits a CAN-FD response. Do not ask
            # the transport to assemble an oversized five-finger register read.
            fingers = []
            for index in range(5):
                fingers.append(self.run(self.client.get_single_touch_sensor_status, self.cfg['hand_id'], index))
                self.state()  # Keep motor feedback fresh for the parent monitor.
            keys = ('status', 'normal_force1', 'tangential_force1', 'tangential_direction1',
                    'normal_force2', 'tangential_force2', 'tangential_direction2',
                    'normal_force3', 'tangential_force3', 'tangential_direction3',
                    'self_proximity1', 'self_proximity2', 'mutual_proximity', 'mutual_proximity1')
            return {'enabled_mask': enabled, 'units': 'native_sdk', 'fingers': {
                finger: {k: int(getattr(item, k)) for k in keys if hasattr(item, k)}
                for finger, item in zip(('thumb', 'index', 'middle', 'ring', 'pinky'), fingers)}}
        self.require_armed()
        before = self.state()
        if 'positions' in args and max(abs(a-b) for a, b in zip(args['positions'], before['positions'])) > self.cfg['max_step']:
            raise ValueError('Revo position step exceeds max_step; split the move')
        self.require_armed()
        self.active = True
        try:
            if 'positions' in args:
                target = args['positions']
                rates = args.get('speeds', [args.get('duration_ms')]*6)
                method = 'set_finger_positions_and_speeds' if 'speeds' in args else 'set_finger_positions_and_durations'
                self.write(method, 1022 if 'speeds' in args else 1010,
                           [v for pair in zip(target, rates) for v in pair], target, rates)
                started = time.monotonic()
                deadline, since, anchor = started + self.cfg['move_timeout_s'], None, None
                while time.monotonic() < deadline:
                    self.require_armed()
                    state = self.state()
                    reached = max(abs(a-b) for a, b in zip(target, state['positions'])) <= self.cfg['position_tolerance']
                    stable = anchor is not None and max(abs(a-b) for a, b in zip(anchor, state['positions'])) <= 2
                    acceptable = reached or (args['completion'] == 'contact' and
                                               time.monotonic() - started >= args.get('duration_ms', 200)/1000)
                    if acceptable and self.stationary(state) and stable:
                        if since is not None and time.monotonic() - since >= self.cfg['settle_s']:
                            self.active = False
                            return {'reached': reached, 'outcome': 'reached' if reached else 'stationary_obstruction',
                                    'target_positions': target, 'state': state, 'grasp_verified': False}
                    else:
                        anchor, since = state['positions'], time.monotonic() if self.stationary(state) else None
                    time.sleep(self.cfg['poll_period_s'])
                raise TimeoutError('Revo target did not settle before the deadline')
            key = 'speeds' if name == 'revo_set_speeds' else 'currents'
            self.write('set_finger_' + key, 1034 if key == 'speeds' else 1040, args[key], args[key])
            deadline = time.monotonic() + args['duration_ms']/1000
            while time.monotonic() < deadline:
                self.require_armed()
                state = self.state()
                if any(not lo <= p <= hi for p, lo, hi in zip(state['positions'], self.cfg['target_min'], self.cfg['target_max'])):
                    raise RuntimeError('Revo reached the configured position envelope during timed control')
                time.sleep(self.cfg['poll_period_s'])
            result = self.hold()
            if not result['confirmed']:
                raise RuntimeError('Revo timed control ended without confirmed standstill')
            return {'outcome': 'timed_control_complete', **result}
        except Exception as exc:
            self.fault = str(exc)
            try:
                self.hold()
            except Exception:
                pass
            raise

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.active:
                self.hold()
        finally:
            try:
                if self.bus:
                    self.bus.close()
                elif self.client is not None and self.cfg['transport'] == 'modbus':
                    self.run(self.sdk.modbus_close, self.client, write=True)
            finally:
                self.loop.close()


def main():
    output = os.fdopen(os.dup(1), 'w', buffering=1)
    os.dup2(2, 1)  # Native logging must never corrupt the internal JSON protocol.
    hardware = None
    try:
        setup = json.loads(sys.stdin.readline(65537))
        hardware = HandHardware(setup['settings'], setup['root'])
        output.write(json.dumps({'ready': True, 'hand': hardware.state()})+'\n')
        for line in sys.stdin:
            request = json.loads(line)
            try:
                if request['op'] == 'close':
                    hardware.close()
                    result = {'closed': True}
                else:
                    result = hardware.execute(request['op'], request.get('arguments', {}))
                output.write(json.dumps({'id': request['id'], 'result': result}, allow_nan=False)+'\n')
                if request['op'] == 'close':
                    break
            except Exception as exc:
                output.write(json.dumps({'id': request.get('id'), 'error': str(exc)})+'\n')
    except Exception as exc:
        output.write(json.dumps({'ready': False, 'error': str(exc)})+'\n')
        return 1
    finally:
        if hardware is not None:
            hardware.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
