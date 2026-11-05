"""Native SDK boundary. Cached callbacks never issue motor commands."""
import fcntl
import math
import os
import threading
import time
from copy import deepcopy
from importlib.metadata import version
from pathlib import Path

from .config import JOINT_NAMES, effort, impedance, vector
from .partial import (DISABLED_STATES, active_indices, control_enabled, disabled_indices,
                      effective_parameters, isolated_diagnostic, validate_isolated_targets)

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
        self.stall_warning_started = {}
        self.diagnostic_exceptions, self.diagnostic_exception_history = [], {}
        self.isolated_faults, self.isolated_fault_history = [], {}
        self.disabled_reference = None
        self.identity = {'serial': config['serial'], 'handedness': config['handedness'], 'sdk_version': SDK_VERSION}
        self.profile = None
        self.stop_initializing_until = None

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

    def _save(self, stream, frame, data, *, warnings=(), faults=(), exceptions=(), isolated=(), stall_warnings=()):
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
                warnings, faults = list(warnings), list(faults)
                active_stalls = {event['joint'] for event in stall_warnings}
                for joint in list(self.stall_warning_started):
                    if joint not in active_stalls:
                        del self.stall_warning_started[joint]
                for event in stall_warnings:
                    started = self.stall_warning_started.setdefault(event['joint'], now)
                    if now-started < self.cfg['stall_warning_grace_s']:
                        warnings.append({**event, 'grace_s': self.cfg['stall_warning_grace_s'],
                                         'elapsed_s': now-started})
                    else:
                        faults.append(event)
                self.warnings = list(warnings)
                self.diagnostic_exceptions = list(exceptions)
                self.isolated_faults = list(isolated)
                for events, history in ((warnings, self.warning_history),
                                        (exceptions, self.diagnostic_exception_history),
                                        (isolated, self.isolated_fault_history)):
                    for event in events:
                        key = (event['joint'], event['code'])
                        record = history.setdefault(key, {**event, 'frames': 0, 'first_seen': now})
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
            warnings, faults, exceptions, isolated, stall_warnings = [], [], [], [], []
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
                current_calibration_exception = (
                    code == 0x3208 and detail.get('name') == 'InitCurrentCalibFailed'
                    and detail.get('severity') == 'Fatal' and detail.get('clear_policy') == 'NonClearable'
                    and self.cfg['allow_init_current_calib_failed'].get(JOINT_NAMES[i])
                    == self.identity.get('firmware') and self.identity.get('firmware') is not None)
                transient_stall = (self.cfg['stall_warning_grace_s'] > 0 and code == 7
                                   and detail.get('name') == 'Stall' and detail.get('severity') == 'Warning'
                                   and detail.get('clear_policy') == 'AutoClear' and data['states'][i] == 2)
                if transient_stall:
                    stall_warnings.append(record)
                elif isolated_diagnostic(self.cfg, JOINT_NAMES[i], code, detail,
                                       self.identity.get('firmware'), data['states'][i]):
                    isolated.append({**record, 'isolated': True, 'native_state': data['state_names'][i]})
                elif current_calibration_exception:
                    # An operator-scoped exception to host admission, not a device
                    # fault clear. Preserve the SDK severity and native state/code.
                    exceptions.append({**record, 'allowed_by': 'allow_init_current_calib_failed'})
                else:
                    (warnings if permitted else faults).append(record)
            for i, temperature in enumerate(data['mcu_temp_c_fb']):
                if temperature >= self.cfg['max_temperature_c']:
                    faults.append({'joint': JOINT_NAMES[i], 'reason': 'temperature limit exceeded',
                                   'temperature_c': temperature})
            for i in disabled_indices(self.cfg):
                if data['states'][i] not in DISABLED_STATES:
                    stopping = (self.stop_initializing_until is not None
                                and self.clock() <= self.stop_initializing_until)
                    booting = ((self.profile is None or stopping) and data['states'][i] == 0
                               and data['state_names'][i] == 'Init'
                               and all(state in (0, *DISABLED_STATES) for state in data['states']))
                    if booting:
                        # Init is not disabled confirmation: admission still blocks
                        # preparation. At startup or during the bounded requested
                        # stop, retain this transient as a warning, not a fault latch.
                        warnings.append({'joint': JOINT_NAMES[i], 'code': None,
                                         'name': 'ExcludedJointInitializing', 'native_state': 'Init',
                                         'reason': 'preparation requires fresh disabled feedback',
                                         'during_requested_stop': stopping})
                    else:
                        faults.append({'joint': JOINT_NAMES[i], 'reason': 'excluded joint is not disabled',
                                       'native_state': data['state_names'][i]})
            self._save('diagnostics', frame, data, warnings=warnings, faults=faults,
                       exceptions=exceptions, isolated=isolated, stall_warnings=stall_warnings)
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
            exceptions = [dict(v) for v in self.diagnostic_exceptions]
            exception_history = [dict(v) for v in self.diagnostic_exception_history.values()]
            isolated = [dict(v) for v in self.isolated_faults]
            isolated_history = [dict(v) for v in self.isolated_fault_history.values()]
        fresh = self.hand is not None and not errors and all(
            age is not None and age <= self.cfg['max_feedback_age_s'] for age in ages.values())
        result = {**deepcopy(self.identity), 'connected': self.hand is not None,
                'positions': state.get('position'), 'velocity_rad_s': state.get('velocity'),
                'effort_a': state.get('effort'), 'diagnostics': diag,
                'fresh': fresh, 'age_s': ages, 'stream_errors': errors, 'faults': faults,
                'warnings': warnings, 'warning_history': warning_history,
                'diagnostic_exceptions': exceptions, 'diagnostic_exception_history': exception_history,
                'isolated_faults': isolated, 'isolated_fault_history': isolated_history,
                'partial_hand': bool(self.cfg['disabled_joints']),
                'disabled_joints': list(self.cfg['disabled_joints']),
                'active_joints': [JOINT_NAMES[i] for i in active_indices(self.cfg)],
                'enabled': fresh and diag.get('states') == [2]*20,
                'received_monotonic': received, 'source_headers': headers,
                'parameters': None if self.profile is None else {k: list(v) for k, v in self.profile.items()}}
        result['active_enabled'] = control_enabled(result, self.cfg)
        return result

    def apply_profile(self):
        desired = effective_parameters(self.cfg)
        if self.cfg['disabled_joints']:
            state = self.snapshot()
            if (not state['fresh'] or len(state['diagnostics'].get('states', [])) != 20
                    or any(s not in DISABLED_STATES for s in state['diagnostics']['states'])):
                raise RuntimeError('joint isolation requires complete fresh disabled feedback before profile writes')
        self.disabled_reference = None
        self.profile = None  # Never reuse an older readback after a partial parameter-write failure.
        self.hand.effort_limit().set(desired['current_limit_a'])
        self.hand.mit_params().set(list(zip(desired['kp'], desired['kd'])))
        limits = list(self.hand.effort_limit().get())
        gains = list(self.hand.mit_params().get())
        if len(limits) != 20 or len(gains) != 20 or any(x is None for x in [*limits, *gains]):
            raise RuntimeError('incomplete control-parameter readback')
        actual = {'current_limit_a': limits, 'kp': [p.kp for p in gains], 'kd': [p.kd for p in gains]}
        for key, values in actual.items():
            if any(not math.isfinite(v) or abs(v-expected) > 1e-5 for v, expected in zip(values, desired[key])):
                raise RuntimeError(f'{key} readback mismatch; hand remains unprepared')
        self.profile = actual

    @staticmethod
    def _check_readback(name, values, expected):
        if len(values) != 20 or any(v is None or not math.isfinite(v) or abs(v-e) > 1e-5
                                    for v, e in zip(values, expected)):
            raise RuntimeError(f'{name} readback mismatch')
        return list(values)

    def set_current_limits(self, values):
        if self.cfg['disabled_joints']:
            raise ValueError('partial-hand mode requires operator profile parameters')
        values = impedance({'current_limit_a': values}, self.cfg)['current_limit_a']
        if self.profile is None:
            raise RuntimeError('hand profile has not been prepared')
        self.hand.effort_limit().set(values)
        actual = self._check_readback('current_limit_a', list(self.hand.effort_limit().get()), values)
        self.profile = {**self.profile, 'current_limit_a': actual}

    def set_impedance(self, kp, kd):
        if self.cfg['disabled_joints']:
            raise ValueError('partial-hand mode requires operator profile parameters')
        values = impedance({'kp': kp, 'kd': kd}, self.cfg)
        if self.profile is None:
            raise RuntimeError('hand profile has not been prepared')
        self.hand.mit_params().set(list(zip(values['kp'], values['kd'])))
        actual = list(self.hand.mit_params().get())
        result = {key: self._check_readback(key, [getattr(p, key, None) for p in actual], values[key])
                  for key in ('kp', 'kd')}
        self.profile = {**self.profile, **result}

    def send(self, positions, velocities=None, effort_a=None):
        q = vector(positions)
        v = [0.]*20 if velocities is None else vector(velocities, 'velocities')
        if any(abs(x) > self.cfg['max_velocity_rad_s']+1e-9 for x in v):
            raise ValueError('command velocity exceeds configured reference limit')
        currents = effort([0.]*20 if effort_a is None else effort_a,
                          (self.profile or self.cfg)['current_limit_a'])
        excluded = disabled_indices(self.cfg)
        if excluded:
            if self.profile is None or any(self.profile[key][i] != 0.
                    for key in ('kp', 'kd', 'current_limit_a') for i in excluded):
                raise RuntimeError('excluded joints require verified zero gains and current limits')
            if any(v[i] != 0. or currents[i] != 0. for i in excluded):
                raise ValueError('excluded joints cannot receive velocity or effort commands')
            if self.disabled_reference is None:
                self.disabled_reference = list(q)
            validate_isolated_targets(self.disabled_reference, q, self.cfg)
        if self.publisher is None:
            self.publisher = self.hand.joint_command().publish()
        self.publisher.send([self.sdk.JointCommand(position, velocity, current)
                             for position, velocity, current in zip(q, v, currents)])

    def enable(self, *, check=None):
        self.stop_initializing_until = None
        excluded = disabled_indices(self.cfg)
        if not excluded:
            self.hand.enable()
            return
        if (self.profile is None or self.disabled_reference is None
                or any(self.profile[key][i] != 0. for key in ('kp', 'kd', 'current_limit_a') for i in excluded)):
            raise RuntimeError('partial-hand enable requires a verified profile and disabled seed')
        mask = [int(i in excluded) for i in range(20)]
        self.hand.disable(mask)
        since, deadline = self.clock(), time.monotonic()+self.cfg['stop_timeout_s']
        while time.monotonic() < deadline:
            if check is not None:
                check()  # A priority stop/disarm must cancel the wait before any enable.
            state = self.snapshot()
            if state['faults']:
                raise RuntimeError('blocking hand fault before selective enable')
            states = state['diagnostics'].get('states', [])
            if (state['fresh'] and len(states) == 20 and all(s in DISABLED_STATES for s in states)
                    and all(state['received_monotonic'].get(k, -1) > since for k in ('state', 'diagnostics'))):
                if check is not None:
                    check()
                self.hand.enable([1-v for v in mask])
                return
            time.sleep(.005)
        raise RuntimeError('excluded joint disable was not confirmed by fresh feedback')

    def stop(self, behavior='emergency'):
        if self.hand is None:
            raise ConnectionError('hand is disconnected; stop delivery unavailable')
        self.stop_initializing_until = self.clock()+self.cfg['stop_timeout_s']
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
        self.stop_initializing_until = None
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
            self.diagnostic_exception_history.clear()
            self.isolated_fault_history.clear()
            self.latched_faults.clear()
            self.warnings.clear()
            self.stall_warning_started.clear()
            self.diagnostic_exceptions.clear()
            self.isolated_faults.clear()
        self.disabled_reference = None
        self.profile = None
        if errors:
            raise RuntimeError('; '.join(errors))
