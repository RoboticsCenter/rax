"""Native SDK boundary. Cached callbacks never issue motor commands."""
import fcntl
import math
import os
import threading
import time
from copy import deepcopy
from importlib.metadata import version
from pathlib import Path

from .config import JOINT_NAMES, vector

SDK_VERSION = '2026.10.12'


def private_directory():
    path = Path(f'/tmp/rax-wuji-{os.getuid()}')
    path.mkdir(mode=0o700, exist_ok=True)
    stat = path.lstat()
    if not path.is_dir() or path.is_symlink() or stat.st_uid != os.getuid() or stat.st_mode & 0o077:
        raise PermissionError(f'{path} must be an operator-owned private directory (0700)')
    return path


class DeviceLease:
    def __init__(self, serial, extra_paths=()):
        self.handles = []
        try:
            for path in [private_directory()/f'{serial}.lock', *map(Path, extra_paths)]:
                fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600)
                handle = os.fdopen(fd, 'a')
                self.handles.append(handle)
                fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.close()
            raise

    def close(self):
        for handle in self.handles:
            handle.close()
        self.handles.clear()


def decode(entries, index_of, fields):
    """NIDs skip tactile slots. Reject incomplete, duplicate and invalid frames."""
    result = {field: [None]*20 for field in fields}
    seen = set()
    for entry in entries:
        index = index_of(entry.nid)
        if type(index) is not int or not 0 <= index < 20 or index in seen:
            raise ValueError(f'invalid or duplicate joint NID {entry.nid}')
        seen.add(index)
        for field in fields:
            result[field][index] = getattr(entry, field)
    if len(seen) != 20:
        raise ValueError('feedback is missing one or more of the 20 joints')
    return result


class WujiDriver:
    def __init__(self, config, *, sdk=None, clock=time.monotonic, lease_factory=DeviceLease):
        self.cfg, self.sdk, self.clock, self.lease_factory = config, sdk, clock, lease_factory
        self.hand = self.publisher = self.lease = None
        self.subscriptions = []
        self.lock = threading.Lock()
        self.frames, self.received, self.headers, self.errors, self.offsets = {}, {}, {}, {}, {}
        self.error_details, self.warning_history, self.latched_faults = {}, {}, {}
        self.warnings = []
        self.identity = {'serial': config['serial'], 'handedness': config['handedness'], 'sdk_version': SDK_VERSION}
        self.profile = None

    def connect(self):
        if self.hand is not None:
            raise RuntimeError('hand is already connected')
        if self.sdk is None:
            if version('wuji-sdk') != SDK_VERSION:
                raise RuntimeError(f'Install wuji-sdk=={SDK_VERSION}; other versions need separate qualification')
            os.environ.setdefault('WUJI_TELEMETRY_DISABLE', '1')
            import wuji_sdk
            self.sdk = wuji_sdk
        self.lease = self.lease_factory(self.cfg['serial'], self.cfg['lock_paths'])
        try:
            locator = {'address': self.cfg['address']} if self.cfg['address'] else {'sn': self.cfg['serial']}
            self.hand = self.sdk.SdkManager.instance().connect(
                **locator, device_name='rax_wuji',
                options=self.sdk.ConnectOptions(enable_bridge=False, timeout_ms=500, retry_count=1))
            if not isinstance(self.hand, self.sdk.WujiHand2):
                raise RuntimeError('expected Wuji Hand 2')
            if self.hand.serial_number != self.cfg['serial']:
                raise RuntimeError('connected hand serial does not match the configured serial')
            if self.hand.handedness().get().lower() != self.cfg['handedness']:
                raise RuntimeError('connected hand has the wrong handedness')
            self.identity['firmware'] = self.hand.info.firmware_version
            self.identity['fingertip_online'] = list(self.hand.fingertip_online())
            self.subscriptions.append(self.hand.joint_states().subscribe_with_callback(
                self._state, lambda e: self._error('state', e)))
            self.subscriptions.append(self.hand.joint_diagnostics().subscribe_with_callback(
                self._diagnostics, lambda e: self._error('diagnostics', e)))
        except BaseException:
            self.close()
            raise

    def _error(self, stream, error):
        with self.lock:
            self.errors[stream] = str(error)

    def _save(self, stream, frame, data, *, warnings=(), faults=()):
        now = self.clock()
        sequence, timestamp = int(frame.header.seq), int(frame.header.timestamp_us)
        with self.lock:
            previous = self.headers.get(stream)
            if previous:
                difference = (sequence - previous['sequence']) % (1 << 32)
                if difference == 0 or difference >= (1 << 31):
                    return  # Duplicated/reordered frames cannot refresh the watchdog.
                if timestamp < previous['timestamp_us']:
                    raise ValueError('device timestamp regressed; reconnect explicitly')
            offset = now - timestamp / 1e6
            baseline = min(self.offsets.get(stream, offset), offset)
            if offset - baseline > self.cfg['max_feedback_age_s']:
                raise ValueError('delayed device stream/backlog')
            self.offsets[stream] = baseline
            self.frames[stream] = data
            self.received[stream] = now
            self.headers[stream] = {'sequence': sequence, 'timestamp_us': timestamp}
            self.errors.pop(stream, None)
            if stream == 'diagnostics':
                self.warnings = list(warnings)
                for warning in warnings:
                    key = (warning['joint'], warning['code'])
                    record = self.warning_history.setdefault(key, {**warning, 'frames': 0, 'first_seen': now})
                    record.update(frames=record['frames']+1, last_seen=now)
                # A brief device fault must not disappear between controller ticks.
                # Only explicit reconnect clears this history, after disable confirmation.
                for fault in faults:
                    self.latched_faults[(fault['joint'], fault.get('code'), fault.get('reason'))] = fault

    def _state(self, frame):
        try:
            data = decode(frame.joints, self.sdk.WujiHand2.nid_to_joint_index, ('position', 'velocity', 'effort'))
            for key in data:
                data[key] = vector(data[key], key)
            self._save('state', frame, data)
        except Exception as exc:
            self._error('state', exc)

    def _diagnostics(self, frame):
        try:
            data = decode(frame.joints, self.sdk.WujiHand2.nid_to_joint_index,
                          ('status_word', 'error_code_current', 'current', 'vbus_v_fb', 'mcu_temp_c_fb'))
            status = data.pop('status_word')
            data['states'] = [int(v.ext_state) for v in status]
            data['state_names'] = [str(v.ext_state_name) for v in status]
            for key in ('current', 'vbus_v_fb', 'mcu_temp_c_fb'):
                values = [float(v) for v in data[key]]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('nonfinite diagnostics')
                data[key] = values
            data['error_code_current'] = [int(v) for v in data['error_code_current']]
            warnings, faults = [], []
            for i, code in enumerate(data['error_code_current']):
                if not code:
                    continue
                if code not in self.error_details:
                    describe = getattr(self.sdk.WujiHand2, 'describe_error', lambda _: None)
                    detail = describe(code)
                    self.error_details[code] = ({k: detail.get(k) for k in ('name', 'severity', 'clear_policy')}
                                                if isinstance(detail, dict) else {})
                detail = self.error_details[code]
                record = {'joint': JOINT_NAMES[i], 'code': code, **detail}
                permitted = (self.cfg['allow_encoder_warning'] and code == 3
                             and detail.get('name') == 'Enc1BitRate' and detail.get('severity') == 'Warning'
                             and detail.get('clear_policy') == 'AutoClear')
                (warnings if permitted else faults).append(record)
            for i, temperature in enumerate(data['mcu_temp_c_fb']):
                if temperature >= self.cfg['max_temperature_c']:
                    faults.append({'joint': JOINT_NAMES[i], 'reason': 'temperature limit exceeded',
                                   'temperature_c': temperature})
            self._save('diagnostics', frame, data, warnings=warnings, faults=faults)
        except Exception as exc:
            self._error('diagnostics', exc)

    def snapshot(self):
        now = self.clock()
        with self.lock:
            # Frame values are lists of immutable scalars. Copy those lists rather
            # than recursively copying every scalar several times per 1 kHz tick.
            state = {k: list(v) for k, v in self.frames.get('state', {}).items()}
            diag = {k: list(v) for k, v in self.frames.get('diagnostics', {}).items()}
            ages = {key: None if key not in self.received else max(0., now-self.received[key])
                    for key in ('state', 'diagnostics')}
            errors = dict(self.errors)
            headers, received = {k: dict(v) for k, v in self.headers.items()}, dict(self.received)
            faults = [dict(v) for v in self.latched_faults.values()]
            warnings = [dict(v) for v in self.warnings]
            warning_history = [dict(v) for v in self.warning_history.values()]
        fresh = self.hand is not None and not errors and all(
            age is not None and age <= self.cfg['max_feedback_age_s'] for age in ages.values())
        return {**deepcopy(self.identity), 'connected': self.hand is not None,
                'positions': state.get('position'), 'velocity_rad_s': state.get('velocity'),
                'effort_a': state.get('effort'), 'diagnostics': diag,
                'fresh': fresh, 'age_s': ages, 'stream_errors': errors, 'faults': faults,
                'warnings': warnings, 'warning_history': warning_history,
                'enabled': fresh and diag.get('states') == [2]*20,
                'received_monotonic': received, 'source_headers': headers,
                'parameters': None if self.profile is None else {k: list(v) for k, v in self.profile.items()}}

    def apply_profile(self):
        self.hand.effort_limit().set(self.cfg['current_limit_a'])
        self.hand.mit_params().set(list(zip(self.cfg['kp'], self.cfg['kd'])))
        limits = list(self.hand.effort_limit().get())
        gains = list(self.hand.mit_params().get())
        if len(limits) != 20 or len(gains) != 20 or any(x is None for x in [*limits, *gains]):
            raise RuntimeError('incomplete control-parameter readback')
        actual = {'current_limit_a': limits, 'kp': [p.kp for p in gains], 'kd': [p.kd for p in gains]}
        for key, values in actual.items():
            if any(not math.isfinite(v) or abs(v-expected) > 1e-5 for v, expected in zip(values, self.cfg[key])):
                raise RuntimeError(f'{key} readback mismatch; hand remains unprepared')
        self.profile = actual

    def send(self, positions, velocities=None):
        q = vector(positions)
        v = [0.]*20 if velocities is None else vector(velocities, 'velocities')
        if any(abs(x) > self.cfg['max_velocity_rad_s']+1e-9 for x in v):
            raise ValueError('command velocity exceeds configured reference limit')
        if self.publisher is None:
            self.publisher = self.hand.joint_command().publish()
        self.publisher.send([self.sdk.JointCommand(position, velocity, 0.) for position, velocity in zip(q, v)])

    def enable(self):
        self.hand.enable()

    def stop(self, behavior='emergency'):
        if self.hand is None:
            raise ConnectionError('hand is disconnected; stop delivery unavailable')
        if behavior == 'disable':
            self.hand.disable()
        else:
            # On the tested Hand 2 / firmware 2.6.0, emergency_stop returned
            # while diagnostics still reported Enabled. RAX's emergency path
            # requires disabled feedback, so explicitly request motor disable
            # even when the first stop RPC fails.
            try:
                self.hand.emergency_stop()
            finally:
                self.hand.disable()

    def close(self):
        errors = []
        for obj in [*self.subscriptions, self.publisher]:
            if obj is not None:
                try:
                    obj.close()
                except Exception as exc:
                    errors.append(str(exc))
        self.subscriptions.clear()
        self.publisher = None
        if self.hand is not None:
            try:
                self.hand.disconnect()
            except Exception as exc:
                errors.append(str(exc))
        self.hand = None
        if self.lease:
            self.lease.close()
            self.lease = None
        with self.lock:
            self.frames.clear()
            self.received.clear()
            self.headers.clear()
            self.offsets.clear()
            self.errors.clear()
            self.error_details.clear()
            self.warning_history.clear()
            self.latched_faults.clear()
            self.warnings.clear()
        self.profile = None
        if errors:
            raise RuntimeError('; '.join(errors))
