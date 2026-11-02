from types import SimpleNamespace

import pytest

from services.revo2.worker import HandHardware
from services.xarm import arming
from test_xarm_backend import make_backend as make_backend


def hand_config(side='right', **overrides):
    return {'type': 'brainco_revo2', 'handedness': side, 'serial': 'serial-'+side,
            'hand_id': 126 if side == 'left' else 127, 'damiao_library': '/tmp/libdm_device.so',
            'postures': {'home': [100, 0, 0, 0, 0, 0]}, **overrides}


class NativeClient:
    def __init__(self):
        self.positions = [100, 0, 0, 0, 0, 0]
        self.writes = []
        self.blocked = False
        self.disarm = None
        self.faulted = False

    async def get_device_info(self, sid):
        return SimpleNamespace(serial_number='serial-right', firmware_version='1.0.22.U',
                               uses_revo2_motor_api=lambda: True, uses_revo2_touch_api=lambda: True)

    async def get_finger_unit_mode(self, sid):
        return 0

    async def get_finger_protected_currents(self, sid):
        return [500]*6

    async def get_turbo_mode_enabled(self, sid):
        return False

    async def get_motor_status(self, sid):
        return SimpleNamespace(positions=self.positions[:], speeds=[0]*6, currents=[0]*6,
                               states=[3 if self.faulted else 0]*6)

    async def set_finger_positions_and_durations(self, sid, positions, durations):
        self.writes.append(('duration', sid, positions[:], durations[:]))
        if not self.blocked:
            self.positions = positions[:]

    async def set_finger_positions_and_speeds(self, sid, positions, speeds):
        self.writes.append(('position_speed', sid, positions[:], speeds[:]))
        self.positions = positions[:]

    async def set_finger_speeds(self, sid, values):
        self.writes.append(('speed', sid, values[:]))
        if self.disarm and any(values):
            self.disarm()

    async def set_finger_currents(self, sid, values):
        self.writes.append(('current', sid, values[:]))

    async def get_touch_sensor_enabled(self, sid):
        return 31

    async def get_single_touch_sensor_status(self, sid, index):
        return SimpleNamespace(status=0, normal_force1=123, tangential_force1=10)


@pytest.fixture
def hardware(tmp_path):
    client = NativeClient()
    cfg = hand_config(allow_current_control=True, allow_speed_control=True, touch=True,
                      settle_s=.01, poll_period_s=.01, move_timeout_s=.1, stop_timeout_s=.1)
    hw = HandHardware(cfg, tmp_path, sdk=SimpleNamespace(FingerUnitMode=SimpleNamespace(Normalized=0)), client=client)
    yield hw, client, tmp_path
    hw.close()


def test_native_read_only_connection_and_six_motor_duration_mapping(hardware):
    hw, client, root = hardware
    assert client.writes == []
    with pytest.raises(PermissionError):
        hw.execute('revo_move', {'positions': [150, 20, 20, 20, 20, 20]})
    assert not client.writes
    arming.write(root, True)
    result = hw.execute('revo_move', {'positions': [150, 20, 20, 20, 20, 20], 'duration_ms': 120})
    assert result['reached'] and not result['grasp_verified']
    assert client.writes == [('duration', 127, [150, 20, 20, 20, 20, 20], [120]*6)]
    assert hw.state()['positions'] == result['state']['positions']


def test_native_speed_position_and_finite_current_mapping(hardware):
    hw, client, root = hardware
    arming.write(root, True)
    hw.execute('revo_move', {'positions': [160, 0, 0, 0, 0, 0], 'speeds': [100]*6})
    assert client.writes[-1] == ('position_speed', 127, [160, 0, 0, 0, 0, 0], [100]*6)
    result = hw.execute('revo_set_currents', {'currents': [200, 0, 100, 100, 100, 100], 'duration_ms': 20})
    assert result['held']
    assert client.writes[-3][0] == 'current'
    assert client.writes[-2] == ('speed', 127, [0]*6)
    assert client.writes[-1][0] == 'duration'


def test_disarm_cancels_native_speed_and_latches_fault(hardware):
    hw, client, root = hardware
    arming.write(root, True)
    client.disarm = lambda: arming.write(root, False)
    with pytest.raises(PermissionError):
        hw.execute('revo_set_speeds', {'speeds': [100]*6, 'duration_ms': 100})
    assert ('speed', 127, [0]*6) in client.writes
    assert hw.state()['fault']
    arming.write(root, True)
    with pytest.raises(RuntimeError, match='operator reconnection'):
        hw.execute('revo_home', {})


def test_obstruction_is_distinct_from_position_completion(hardware):
    hw, client, root = hardware
    arming.write(root, True)
    client.blocked = True
    result = hw.execute('revo_move', {'positions': [200, 100, 100, 100, 100, 100],
                                     'duration_ms': 20, 'completion': 'contact'})
    assert result['outcome'] == 'stationary_obstruction' and not result['reached']
    with pytest.raises(TimeoutError):
        hw.execute('revo_move', {'positions': [200, 100, 100, 100, 100, 100]})
    assert ('speed', 127, [0]*6) in client.writes


def test_touch_and_fault_feedback_are_not_fabricated(hardware):
    hw, client, _ = hardware
    result = hw.execute('revo_touch', {})
    assert result['fingers']['index']['normal_force1'] == 123 and not client.writes
    client.faulted = True
    with pytest.raises(RuntimeError, match='feedback'):
        hw.state()


@pytest.mark.parametrize('arguments', [
    {'positions': [0]*20}, {'positions': [True]*6}, {'positions': [150.5]*6},
    {'positions': [1001]*6}, {'positions': [0]*6}, {'positions': [float('nan')]*6},
    {'positions': [100]*6, 'duration_ms': 100, 'speeds': [100]*6},
])
def test_invalid_revo_targets_never_dispatch(hardware, arguments):
    hw, client, root = hardware
    arming.write(root, True)
    with pytest.raises(ValueError):
        hw.execute('revo_move', arguments)
    assert not client.writes


def test_identity_units_and_large_step_fail_before_writes(hardware, tmp_path):
    hw, client, root = hardware
    arming.write(root, True)
    with pytest.raises(ValueError, match='max_step'):
        hw.execute('revo_move', {'positions': [500]*6})
    with pytest.raises(RuntimeError, match='mismatch'):
        HandHardware(hand_config(serial='wrong'), tmp_path,
                     sdk=hw.sdk, client=client)
    assert not client.writes


class FakeHand:
    def __init__(self, side):
        self.side = side
        self.calls, self.stops = [], 0
        self.fail = False
        self.ready = True
        self.closed = False

    def status(self):
        return {'positions': [100, 0, 0, 0, 0, 0], 'fresh': True, 'serial': 'serial-'+self.side}

    def require_ready(self):
        if not self.ready:
            raise RuntimeError('hand feedback unavailable')
        return self.status()

    def execute(self, name, args):
        self.calls.append((name, args))
        if self.fail:
            raise RuntimeError('injected native write failure')
        return self.status() if name == 'revo_status' else {'reached': True}

    def stop(self):
        self.stops += 1
        return {'confirmed': True}

    def close(self):
        self.closed = True


def test_single_xarm_revo_tools_and_arming(make_backend):
    hand = FakeHand('right')
    backend, arm, root = make_backend(hand=hand, end_effector=hand_config())
    names = {t['name'] for t in backend.tools()}
    assert 'revo_move' in names and 'xarm_gripper' not in names and 'wuji_move_joints' not in names
    with pytest.raises(PermissionError):
        backend.call('revo_move', {'positions': [150, 0, 0, 0, 0, 0]})
    arming.write(root, True)
    assert backend.call('revo_move', {'positions': [150, 0, 0, 0, 0, 0]})[0]['reached']
    assert arm.commands == 0
    with pytest.raises(ValueError):
        backend.call('wuji_status', {})
