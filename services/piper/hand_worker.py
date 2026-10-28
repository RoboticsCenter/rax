"""Isolated official BrainCo SDK + DaMiao owner. Internal JSON-lines protocol only."""
import asyncio
import json
import os
import sys
import time

from .damiao import DamiaoBus, native_context, position_request


class HandHardware:
    def __init__(self, settings, *, sdk=None, context=None):
        if sdk is None:
            from bc_stark_sdk import main_mod as sdk
        self.settings, self.sdk = settings, sdk
        self.loop = asyncio.new_event_loop()
        self.bus = None
        self.client = None
        self.info = None
        context = context if context is not None else native_context(settings['damiao_library'])
        errors = []
        try:
            for index in range(context.count):
                try:
                    self.bus = DamiaoBus(context, index, settings)
                    sdk.set_can_tx_callback(self.bus.send)
                    sdk.set_can_rx_callback(self.bus.receive)
                    self.client = sdk.init_device_handler(sdk.StarkProtocolType.CanFd, 1)
                    serial = self.run(self.client.get_device_sn, settings['hand_id'])
                    if serial != settings['hand_serial']:
                        raise RuntimeError('Selected hand serial not present on this adapter')
                    self._inspect()
                    break
                except Exception as exc:
                    errors.append(f'adapter {index}: {exc}')
                    if self.bus is not None:
                        self.bus.close()
                        self.bus = None
            if self.bus is None:
                raise RuntimeError('No verified right Revo 2: ' + '; '.join(errors))
        except BaseException:
            self.close()
            raise

    def run(self, method, *arguments, allow_none=False):
        async def invoke():
            return await asyncio.wait_for(method(*arguments), timeout=2.)
        result = self.loop.run_until_complete(invoke())
        if self.bus.error or self.bus.transaction_error:
            raise RuntimeError(self.bus.error or self.bus.transaction_error)
        if result is None and not allow_none:
            raise RuntimeError('BrainCo SDK returned no result')
        return result

    def _inspect(self):
        slave = self.settings['hand_id']
        info = self.run(self.client.get_device_info, slave)
        if info.serial_number != self.settings['hand_serial'] or info.firmware_version != self.settings['hand_firmware']:
            raise RuntimeError('Unexpected BrainCo serial or firmware')
        mode = self.run(self.client.get_finger_unit_mode, slave)
        currents = list(self.run(self.client.get_finger_protected_currents, slave))
        turbo = self.run(self.client.get_turbo_mode_enabled, slave)
        if int(mode) != int(self.sdk.FingerUnitMode.Normalized) or currents != [500] * 6 or bool(turbo):
            raise RuntimeError('Unexpected BrainCo units, protected currents or turbo mode; no settings changed')
        self.info = {'serial': info.serial_number, 'firmware': info.firmware_version,
                     'adapter_index': self.bus.index, 'adapter_firmware': self.bus.version,
                     'channel': self.bus.channel, 'hand_id': slave, 'protected_currents_ma': currents,
                     'units': 'normalized_0_1000', 'turbo': False, 'driver': 'bc-stark-sdk'}
        self.state()

    def state(self):
        status = self.run(self.client.get_motor_status, self.settings['hand_id'])
        value = {'positions': list(status.positions), 'speeds': list(status.speeds),
                 'current_normalized': list(status.currents),
                 'motor_states': [int(item) for item in status.states]}
        if (any(len(items) != 6 for items in value.values())
                or any(not 0 <= item <= 1000 for item in value['positions'])
                or any(item not in (0, 1, 2) for item in value['motor_states'])):
            raise RuntimeError('Invalid or faulted BrainCo feedback')
        return {**self.info, **value, 't': time.monotonic()}

    def move(self, positions, *, hold=False):
        if not hold and not self.settings['motion_commissioned']:
            raise RuntimeError('Hand motion is not commissioned')
        if len(positions) != 6 or any(type(item) is not int for item in positions):
            raise ValueError('Six integer hand targets required')
        if any(not lower <= item <= upper for item, lower, upper in zip(
                positions, self.settings['hand_target_min'], self.settings['hand_target_max'])):
            raise ValueError('Hand targets outside the commissioned envelope')
        current = self.state()['positions']
        if max(abs(target - actual) for target, actual in zip(positions, current)) > self.settings['hand_max_step']:
            raise ValueError('Hand step too large')
        milliseconds = 100 if hold else self.settings['hand_segment_ms']
        self.bus.authorized_write = position_request(self.settings['hand_id'], positions, milliseconds)
        self.bus.write_acknowledged = False
        self.bus.write_sent = False
        try:
            self.run(self.client.set_finger_positions_and_durations,
                     self.settings['hand_id'], positions, [milliseconds] * 6, allow_none=True)
            if self.bus.error or self.bus.transaction_error or not self.bus.write_acknowledged:
                raise RuntimeError('Hand position write unacknowledged; execution uncertain, no retry')
        finally:
            self.bus.authorized_write = None
        return {'acknowledged': True}

    def hold(self):
        return self.move(self.state()['positions'], hold=True)

    def close(self):
        try:
            if self.bus is not None:
                self.bus.close()
                self.bus = None
        finally:
            if not self.loop.is_closed():
                self.loop.close()


def main():
    output = os.fdopen(os.dup(1), 'w', buffering=1)
    os.dup2(2, 1)
    hardware = None
    try:
        from .config import options
        settings = options(json.loads(sys.stdin.readline(65537)))
        hardware = HandHardware(settings)
        output.write(json.dumps({'ready': True, 'hand': hardware.state()}) + '\n')
        for line in sys.stdin:
            request = json.loads(line)
            try:
                if set(request) - {'id', 'op', 'positions'}:
                    raise ValueError('Unexpected hand worker arguments')
                operation = request['op']
                if operation == 'state':
                    result = hardware.state()
                elif operation == 'move':
                    result = hardware.move(request['positions'])
                elif operation == 'hold':
                    result = hardware.hold()
                elif operation == 'close':
                    hardware.close()
                    output.write(json.dumps({'id': request['id'], 'result': {'closed': True}}) + '\n')
                    break
                else:
                    raise ValueError('Unknown hand worker operation')
                output.write(json.dumps({'id': request['id'], 'result': result}) + '\n')
            except Exception as exc:
                output.write(json.dumps({'id': request.get('id'), 'error': str(exc)}) + '\n')
    except Exception as exc:
        output.write(json.dumps({'ready': False, 'error': str(exc)}) + '\n')
        return 1
    finally:
        if hardware is not None:
            hardware.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
