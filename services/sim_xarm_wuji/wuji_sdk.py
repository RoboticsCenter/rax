"""A mimic of the Wuji SDK (`wuji_sdk` 2026.10.12, as `import wuji_sdk`) for the simulated
Wuji Hand 2s, given to RAX's unchanged hand driver (services/wuji/sdk.py WujiDriver,
whose `sdk=` argument accepts the module) in place of the vendor's compiled module.

What it emulates, with the vendor's names and signatures (checked against the SDK's
own stubs by local_tests/unit/test_sim_xarm_wuji.py):
- SdkManager.instance().connect(sn=... | address=..., device_name, options=ConnectOptions)
  -> WujiHand2; disconnect() / disconnect_all();
- WujiHand2: serial_number, device_name, info.firmware_version, handedness().get(),
  fingertip_online(); joint_states() / joint_diagnostics() .subscribe_with_callback(cb,
  on_error) and .subscribe() (Subscription.recv(), set_rate(hz), close());
  joint_command().publish().send([JointCommand(position, velocity, effort)] * 20)
  (MIT targets plus feed-forward current; real-time streaming up to 1 kHz);
  effort_limit() / mit_params() .set() / .get() (scalar, one pair, or 20);
  enable / disable / clear_fault(joints mask), emergency_stop();
  set_constant(FeedForwardMode, joints), set_position_table(on, joints);
  joint(k) / joints() / fingers() handles; nid_to_joint_index, joint_index_to_nid,
  describe_error;
- frames: header.seq / timestamp_us; joints[].nid, position, velocity, effort;
  diagnostics joints[].status_word.ext_state / ext_state_name (a property, as the stubs
  have it), current, vbus_v_fb, mcu_temp_c_fb, error_code_current.

The transport is the sim server (server.py): parameter calls are RPCs, the 1 kHz streams
and commands are shared memory. Not emulated: discovery, the bridge, time sync,
tactile, IMU, firmware upgrade, Position mode (firmware >= 3.0.0).

configure(socket_path) (or $RAX_SIM_SOCKET) points the module at a sim server.
"""
from __future__ import annotations

import enum
import math
import os
import threading
import time
from typing import List, Optional

from . import shm
from .firmware import describe_error as _describe_error, index_of_nid, nid_of
from .rpc import Client, SimError
from .spec import STATE_NAMES

__version__ = '2026.10.12+rax.sim'
RAX_BACKEND_KIND = 'sim'  # services/wuji/sdk.py WujiDriver.backend_kind: labels traces as simulated
_socket = os.environ.get('RAX_SIM_SOCKET')
JOINT_LABELS = [f'{finger}_joint{j}' for finger in ('thumb', 'index', 'middle', 'ring', 'pinky') for j in range(1, 5)]


def configure(socket_path):
    global _socket
    _socket = str(socket_path)


class WujiException(Exception):
    pass


class NodeOfflineError(WujiException):
    pass


class Handedness(enum.Enum):
    Left = 'left'
    Right = 'right'


class FeedForwardMode(enum.IntEnum):
    Off = 0
    Constant = 1
    VelocityTable = 2
    ConstantAndVelocityTable = 3


class ControlMode(enum.IntEnum):
    Off = 0
    Mit = 1
    Position = 2
    Velocity = 3
    Current = 4


class ConnectOptions:
    def __init__(self, timeout_ms: int = 1000, retry_count: int = 3, enable_bridge: bool = True,
                 auto_time_sync_interval_ms: Optional[int] = 30000) -> None:
        self.timeout_ms, self.retry_count = int(timeout_ms), int(retry_count)
        self.enable_bridge, self.auto_time_sync_interval_ms = bool(enable_bridge), auto_time_sync_interval_ms


class LowPass:
    def __init__(self, cutoff_hz: float) -> None:
        self.cutoff_hz = float(cutoff_hz)


class DeviceInfo:
    def __init__(self, serial_number, firmware_version):
        self.serial_number, self.firmware_version = serial_number, firmware_version


class FrameHeader:
    __slots__ = ('seq', 'timestamp_us', 'frame_id')

    def __init__(self, seq: int, timestamp_us: int, frame_id: str = '') -> None:
        self.seq, self.timestamp_us, self.frame_id = int(seq), int(timestamp_us), frame_id


class JointStateEntry:
    __slots__ = ('nid', 'position', 'velocity', 'effort')

    def __init__(self, nid, position, velocity, effort):
        self.nid, self.position, self.velocity, self.effort = nid, position, velocity, effort


class JointStateFrame:
    __slots__ = ('header', 'joints', 'num_joints')

    def __init__(self, header, joints):
        self.header, self.joints, self.num_joints = header, joints, len(joints)

    def __len__(self):
        return len(self.joints)


class StatusWord:
    __slots__ = ('ext_state', 'position_limit_active', 'velocity_limit_active', 'current_limit_active')

    def __init__(self, ext_state, current_limit_active=False):
        self.ext_state = int(ext_state)
        self.position_limit_active = self.velocity_limit_active = False
        self.current_limit_active = bool(current_limit_active)

    @property
    def ext_state_name(self) -> str:
        return STATE_NAMES.get(self.ext_state, 'Unknown')


class JointDiagnosticsEntry:
    __slots__ = ('nid', 'status_word', 'current', 'vbus_v_fb', 'mcu_temp_c_fb', 'error_code_current',
                 'comm_response_rate_pct', 'comm_timeout_total')

    def __init__(self, nid, status_word, current, vbus, temperature, error):
        self.nid, self.status_word = nid, status_word
        self.current, self.vbus_v_fb, self.mcu_temp_c_fb = current, vbus, temperature
        self.error_code_current, self.comm_response_rate_pct, self.comm_timeout_total = int(error), 100, 0


class JointDiagnosticsFrame:
    __slots__ = ('header', 'joints', 'num_joints', 'comm')

    def __init__(self, header, joints):
        self.header, self.joints, self.num_joints, self.comm = header, joints, len(joints), None

    def __len__(self):
        return len(self.joints)


class JointCommand:
    __slots__ = ('position', 'velocity', 'effort')

    def __init__(self, position: float, velocity: float, effort: float) -> None:
        self.position, self.velocity, self.effort = float(position), float(velocity), float(effort)


class MitParam:
    __slots__ = ('kp', 'kd')

    def __init__(self, kp: float, kd: float) -> None:
        self.kp, self.kd = float(kp), float(kd)

    def __repr__(self):
        return f'MitParam(kp={self.kp}, kd={self.kd})'


_NIDS = [nid_of(i) for i in range(20)]


def _state_frame(seq, t_ns, value):
    q, dq, effort = value['q'].tolist(), value['dq'].tolist(), value['effort'].tolist()
    return JointStateFrame(FrameHeader(seq, t_ns // 1000),
                           [JointStateEntry(_NIDS[i], q[i], dq[i], effort[i]) for i in range(20)])


def _diag_frame(seq, t_ns, value):
    states, error = value['states'].tolist(), value['error'].tolist()
    current, vbus, temp = value['current'].tolist(), value['vbus'].tolist(), value['temperature'].tolist()
    limited = value['current_limited'].tolist()
    return JointDiagnosticsFrame(FrameHeader(seq, t_ns // 1000), [
        JointDiagnosticsEntry(_NIDS[i], StatusWord(states[i], limited[i]), current[i], vbus[i], temp[i], error[i])
        for i in range(20)])


class _Stream:
    """Delivers one shared-memory stream's new frames to its subscribers (a reader thread
    per hand connection, as the SDK's native receive thread)."""

    def __init__(self, link, name, block, builder, native_hz):
        self.link, self.name, self.block, self.builder = link, name, block, builder
        self.native_hz = float(native_hz)
        self.decimation = 1
        self.callbacks, self.queues = [], []
        self.last = block.count()
        self.seq = 0

    def poll(self):
        count = self.block.count()
        if count == self.last:
            return False
        self.last = count
        self.seq = (self.seq + 1) % (1 << 32)
        if count % self.decimation:
            return True
        _, t_ns, value = self.block.read()
        frame = self.builder(self.seq, t_ns, value)
        for subscription, callback, on_error in list(self.callbacks):
            try:
                callback(frame)
                subscription.received_count += 1
            except Exception as exc:  # noqa: BLE001 - the SDK hands callback errors to on_error
                subscription.error_count += 1
                if on_error is not None:
                    on_error(str(exc))
        for subscription in list(self.queues):
            subscription._latest = frame
        return True


class CallbackSubscription:
    def __init__(self, stream, callback, on_error):
        self._stream, self.dtype = stream, stream.name
        self.is_active, self.received_count, self.error_count = True, 0, 0
        self._entry = (self, callback, on_error)
        stream.callbacks.append(self._entry)

    def close(self) -> None:
        if self.is_active:
            self.is_active = False
            try:
                self._stream.callbacks.remove(self._entry)
            except ValueError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class Subscription:
    def __init__(self, stream):
        self._stream, self.dtype, self.stream_id = stream, stream.name, id(stream)
        self._latest = None
        stream.queues.append(self)

    def recv(self):
        if self._stream.link.closed:
            raise WujiException('connection disconnected')
        frame, self._latest = self._latest, None
        return frame

    def set_rate(self, frequency_hz: int) -> int:
        frequency_hz = int(frequency_hz)
        if not 0 <= frequency_hz <= 65535:
            raise OverflowError('frequency_hz outside 0..65535')
        stream = self._stream
        stream.decimation = 1 if frequency_hz == 0 else max(1, round(stream.native_hz / frequency_hz))
        return int(round(stream.native_hz / stream.decimation))

    def close(self) -> None:
        try:
            self._stream.queues.remove(self)
        except ValueError:
            pass


class _StreamResource:
    def __init__(self, stream):
        self._stream = stream

    def subscribe(self) -> Subscription:
        return Subscription(self._stream)

    def subscribe_with_callback(self, callback, on_error=None) -> CallbackSubscription:
        return CallbackSubscription(self._stream, callback, on_error)


class JointCommandPublisher:
    def __init__(self, link):
        self._link, self._open = link, True

    def send(self, joints: List[JointCommand]) -> None:
        if not self._open or self._link.closed:
            raise WujiException('the publisher is closed')
        if len(joints) != 20:
            raise WujiException(f'a Wuji Hand 2 command carries 20 joints, not {len(joints)}')
        q = [float(j.position) for j in joints]
        dq = [float(j.velocity) for j in joints]
        effort = [float(j.effort) for j in joints]
        if not all(math.isfinite(v) for v in (*q, *dq, *effort)):
            raise WujiException('joint command values must be finite')
        self._link.command.write(time.monotonic_ns(), {'q': q, 'dq': dq, 'effort': effort})

    def close(self) -> None:
        self._open = False


class JointCommandResource:
    def __init__(self, link):
        self._link = link

    def publish(self) -> JointCommandPublisher:
        return JointCommandPublisher(self._link)


class _DeviceEffortLimitResource:
    def __init__(self, hand):
        self._hand = hand

    def set(self, value) -> None:
        values = float(value) if isinstance(value, (int, float)) else [float(v) for v in value]
        self._hand._rpc('set_effort_limit', value=values)

    def get(self) -> List[Optional[float]]:
        return [float(v) for v in self._hand._rpc('get_effort_limit')]


class _DeviceMitParamsResource:
    def __init__(self, hand):
        self._hand = hand

    def set(self, value) -> None:
        values = value if isinstance(value, list) else [value] * 20
        pairs = [[p.kp, p.kd] if isinstance(p, MitParam) else [float(p[0]), float(p[1])] for p in values]
        self._hand._rpc('set_mit_params', value=pairs)

    def get(self) -> List[Optional[MitParam]]:
        return [MitParam(kp, kd) for kp, kd in self._hand._rpc('get_mit_params')]


class _ValueResource:
    def __init__(self, value):
        self._value = value

    def get(self):
        return self._value


class _JointResource:
    def __init__(self, hand, index, what):
        self._hand, self._index, self._what = hand, index, what

    def get(self):
        if self._what == 'effort_limit':
            return self._hand._rpc('joint', index=self._index, what='get_effort_limit')
        value = self._hand._rpc('joint', index=self._index, what=self._what)
        if self._what == 'status_word':
            return StatusWord(value['ext_state'], value['current_limit_active'])
        return value

    def set(self, value):
        what = 'set_effort_limit' if self._what == 'effort_limit' else self._what
        self._hand._rpc('joint', index=self._index, what=what, value=value)


class JointHandle:
    def __init__(self, hand, index):
        self._hand, self._index = hand, int(index)

    @property
    def label(self) -> str:
        return JOINT_LABELS[self._index]

    @property
    def index(self) -> int:
        return self._index

    def effort_limit(self):
        return _JointResource(self._hand, self._index, 'effort_limit')

    def error_code(self):
        return _JointResource(self._hand, self._index, 'error_code')

    def status_word(self):
        return _JointResource(self._hand, self._index, 'status_word')

    def enable(self) -> None:
        self._hand._rpc('joint', index=self._index, what='enable')

    def disable(self) -> None:
        self._hand._rpc('joint', index=self._index, what='disable')

    def clear_fault(self) -> None:
        self._hand._rpc('joint', index=self._index, what='clear_fault')

    def constant(self):
        return _JointResource(self._hand, self._index, 'constant')

    def position_table(self):
        return _JointResource(self._hand, self._index, 'position_table')

    def __repr__(self):
        return f'JointHandle({self.label})'


class FingerHandle:
    def __init__(self, hand, finger):
        self._joints = [JointHandle(hand, finger * 4 + j) for j in range(4)]

    def j0(self):
        return self._joints[0]

    def j1(self):
        return self._joints[1]

    def j2(self):
        return self._joints[2]

    def j3(self):
        return self._joints[3]

    def joints(self):
        return list(self._joints)


class _Link:
    """One hand connection: the RPC client and the shared-memory streams."""

    def __init__(self, socket_path, serial, timeout_s):
        self.client = Client(socket_path, timeout=max(timeout_s, 5.0))
        self.serial = serial
        self.closed = False
        self.info = self.client.call('wuji.connect', serial=serial)
        paths = self.info['shm']
        self.command = shm.Block(paths['command'], shm.COMMAND_FIELDS)
        self.state = _Stream(self, 'JointStateFrame', shm.Block(paths['state'], shm.STATE_FIELDS), _state_frame,
                             self.info['state_rate_hz'])
        self.diagnostics = _Stream(self, 'JointDiagnosticsFrame', shm.Block(paths['diag'], shm.DIAG_FIELDS),
                                   _diag_frame, self.info['diagnostics_rate_hz'])
        self.period = 1.0 / max(self.info['state_rate_hz'], 1.0)
        self.thread = threading.Thread(target=self._receive, name=f'wuji-sim-{serial}', daemon=True)
        self.thread.start()

    def _receive(self):
        while not self.closed:
            try:
                fresh = self.state.poll()
                fresh = self.diagnostics.poll() or fresh
            except Exception as exc:  # noqa: BLE001 - like the SDK's thread: report, keep receiving
                fresh = False
                for stream in (self.state, self.diagnostics):
                    for _, _, on_error in list(stream.callbacks):
                        if on_error is not None:
                            on_error(f'{type(exc).__name__}: {exc}')
            time.sleep(self.period / 4 if fresh else self.period / 8)

    def close(self):
        self.closed = True
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=1)
        self.client.close()


class WujiHand2:
    def __init__(self, link, device_name):
        self._link = link
        self.device_name = device_name
        self.serial_number = link.info['serial_number']
        self.info = DeviceInfo(link.info['serial_number'], link.info['firmware_version'])
        self._handedness = link.info['handedness']

    @property
    def device(self):
        return self

    def _rpc(self, name, **args):
        if self._link.closed:
            raise WujiException('the hand is disconnected')
        try:
            return self._link.client.call(f'wuji.{name}', serial=self.serial_number, **args)
        except (SimError, OSError) as exc:
            raise WujiException(str(exc)) from None

    def handedness(self):
        # The stubs type .get() as int; the hand answers 'Left' / 'Right' and RAX's driver
        # calls .lower() on it (as mooncake's mimic found on the rig).
        return _ValueResource(self._handedness)

    def fingertip_online(self) -> List[bool]:
        return list(self._link.info['fingertip_online'])

    def joint_states(self):
        return _StreamResource(self._link.state)

    def joint_diagnostics(self):
        return _StreamResource(self._link.diagnostics)

    def joint_command(self):
        return JointCommandResource(self._link)

    def effort_limit(self):
        return _DeviceEffortLimitResource(self)

    def mit_params(self):
        return _DeviceMitParamsResource(self)

    def enable(self, joints: Optional[List[int]] = None) -> None:
        self._rpc('enable', joints=None if joints is None else [int(v) for v in joints])

    def disable(self, joints: Optional[List[int]] = None) -> None:
        self._rpc('disable', joints=None if joints is None else [int(v) for v in joints])

    def clear_fault(self, joints: Optional[List[int]] = None) -> None:
        self._rpc('clear_fault', joints=None if joints is None else [int(v) for v in joints])

    def emergency_stop(self) -> None:
        self._rpc('emergency_stop')

    def set_constant(self, mode, joints: Optional[List[int]] = None) -> None:
        self._rpc('set_constant', mode=int(mode), joints=None if joints is None else [int(v) for v in joints])

    def set_position_table(self, on: bool, joints: Optional[List[int]] = None) -> None:
        self._rpc('set_position_table', on=bool(on), joints=None if joints is None else [int(v) for v in joints])

    def joints(self) -> List[JointHandle]:
        return [JointHandle(self, i) for i in range(20)]

    def fingers(self) -> List[FingerHandle]:
        return [FingerHandle(self, f) for f in range(5)]

    def joint(self, joint_index: int) -> JointHandle:
        if not 0 <= int(joint_index) <= 19:
            raise WujiException(f'joint index {joint_index} is outside 0..19')
        return JointHandle(self, joint_index)

    def online_joints_count(self):
        return _ValueResource(20)

    @staticmethod
    def nid_to_joint_index(nid: int) -> int:
        index = index_of_nid(nid)
        if index is None:
            raise WujiException(f'nid {nid} is not a joint (joint nids are 1-4, 6-9, 11-14, 16-19, 21-24)')
        return index

    @staticmethod
    def joint_index_to_nid(joint_index: int) -> int:
        if not 0 <= int(joint_index) <= 19:
            raise WujiException(f'joint index {joint_index} is outside 0..19')
        return nid_of(joint_index)

    @staticmethod
    def describe_error(code: int):
        return _describe_error(code) if int(code) else None

    def disconnect(self) -> None:
        self._link.close()
        SdkManager.instance()._forget(self)

    def __repr__(self):
        return f'WujiHand2(sim, serial={self.serial_number})'


class SdkManager:
    _instance = None

    def __init__(self):
        self._devices = {}

    @staticmethod
    def instance() -> 'SdkManager':
        if SdkManager._instance is None:
            SdkManager._instance = SdkManager()
        return SdkManager._instance

    def connect(self, *, sn: Optional[str] = None, address: Optional[str] = None, handedness=None,
                device_name: str, options: Optional[ConnectOptions] = None):
        if sum(v is not None for v in (sn, address, handedness)) != 1:
            raise WujiException('connect takes exactly one of sn, address, handedness')
        if sn is None:
            raise WujiException('the simulated hands are reached by serial number (address and handedness '
                                'discovery are not simulated)')
        if not _socket:
            raise WujiException('no sim server configured (wuji_sdk.configure or RAX_SIM_SOCKET)')
        options = options or ConnectOptions()
        last = None
        for _ in range(max(1, options.retry_count + 1)):
            try:
                hand = WujiHand2(_Link(_socket, sn, options.timeout_ms / 1000.0), device_name)
                self._devices[device_name] = hand
                return hand
            except (OSError, SimError, ValueError) as exc:
                last = exc
                time.sleep(0.05)
        raise WujiException(f'could not connect to the simulated hand {sn}: {last}')

    def get_device(self, device_name: str):
        return self._devices.get(device_name)

    def get_connected_devices(self):
        return list(self._devices.items())

    def disconnect(self, device_name: str) -> None:
        hand = self._devices.pop(device_name, None)
        if hand is not None:
            hand._link.close()

    def disconnect_all(self) -> None:
        for name in list(self._devices):
            self.disconnect(name)

    def _forget(self, hand):
        for name, value in list(self._devices.items()):
            if value is hand:
                del self._devices[name]
