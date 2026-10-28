from types import SimpleNamespace as NS

import pytest
from wuji_fakes import Clock, hand_config

from services.wuji.sdk import DeviceLease, WujiDriver


def index_of(nid):
    bus, slot = divmod(nid-1, 5)
    if not 0 <= bus < 5 or not 0 <= slot < 4:
        raise ValueError('invalid NID')
    return bus*4+slot


def frame(clock, seq=1):
    joints = [NS(nid=i//4*5+i%4+1, position=i/100, velocity=0., effort=.1,
                 status_word=NS(ext_state=1, ext_state_name='Ready'), error_code_current=0,
                 current=.1, vbus_v_fb=12., mcu_temp_c_fb=35.) for i in range(20)]
    return NS(header=NS(seq=seq, timestamp_us=int(clock()*1e6)), joints=joints)


class Resource:
    def __init__(self, value=None):
        self.value = value
        self.callback = None
        self.closed = False

    def get(self):
        return self.value

    def set(self, value):
        self.value = value

    def subscribe_with_callback(self, callback, on_error):
        self.callback, self.error = callback, on_error
        return self

    def close(self):
        self.closed = True


class Hand:
    nid_to_joint_index = staticmethod(index_of)

    def __init__(self):
        self.serial_number = 'FAKE-HAND-ONLY'
        self.side = 'right'
        self.info = NS(firmware_version='2.6.0')
        self.states, self.diagnostics = Resource(), Resource()
        self.efforts, self.gains = Resource([.5]*20), Resource([NS(kp=1.5, kd=.1)]*20)
        self.disconnected = False
        self.writes = []

    def handedness(self):
        return Resource(self.side)

    def fingertip_online(self):
        return [False]*5

    def joint_states(self):
        return self.states

    def joint_diagnostics(self):
        return self.diagnostics

    def effort_limit(self):
        return self.efforts

    def mit_params(self):
        return self.gains

    def disconnect(self):
        self.disconnected = True


def driver_setup(**options):
    hand, clock, connection = Hand(), Clock(), {}
    def connect(**kwargs):
        connection.update(kwargs)
        return hand
    sdk = NS(WujiHand2=Hand, SdkManager=NS(instance=lambda: NS(connect=connect)), ConnectOptions=lambda **kw: kw)
    lease = NS(close=lambda: None)
    driver = WujiDriver(hand_config(**options), sdk=sdk, clock=clock, lease_factory=lambda *a: lease)
    return driver, hand, clock, connection


@pytest.mark.parametrize('allowed,detail,expected', [
    (True, {'name': 'Enc1BitRate', 'severity': 'Warning', 'clear_policy': 'AutoClear'}, True),
    (False, {'name': 'Enc1BitRate', 'severity': 'Warning', 'clear_policy': 'AutoClear'}, False),
    (True, {'name': 'Other', 'severity': 'Warning', 'clear_policy': 'AutoClear'}, False),
    (True, {'name': 'Enc1BitRate', 'severity': 'Fault', 'clear_policy': 'AutoClear'}, False),
    (True, {'name': 'Enc1BitRate', 'severity': 'Warning', 'clear_policy': 'ManualClear'}, False),
    (True, None, False),
])
def test_encoder_warning_requires_explicit_opt_in_and_exact_metadata(monkeypatch, allowed, detail, expected):
    driver, hand, clock, _ = driver_setup(allow_encoder_warning=allowed)
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: detail), raising=False)
    driver.connect()
    value = frame(clock)
    value.joints[1].error_code_current = 3
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    status = driver.snapshot()
    assert bool(status['warnings']) is expected and bool(status['faults']) is not expected
    assert status['fresh']  # Warning classification never masks a missing feedback stream.
    if expected:
        hand.diagnostics.callback(value)  # A duplicate must not count as another event.
        assert driver.snapshot()['warning_history'][0]['frames'] == 1
        clock.now += .001
        hand.diagnostics.callback(frame(clock, 2))
        assert not driver.snapshot()['warnings'] and driver.snapshot()['warning_history'][0]['frames'] == 1
    else:
        clock.now += .001
        hand.diagnostics.callback(frame(clock, 2))
        assert driver.snapshot()['faults']  # A transient fault persists until reconnect.


def test_other_codes_are_never_exempt_and_reconnect_clears_history(monkeypatch):
    driver, hand, clock, _ = driver_setup(allow_encoder_warning=True)
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: {
        'name': 'Enc1BitRate', 'severity': 'Warning', 'clear_policy': 'AutoClear'}), raising=False)
    driver.connect()
    value = frame(clock)
    value.joints[-1].error_code_current = 4
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    assert driver.snapshot()['faults'][0]['joint'] == 'pinky_S4'
    assert not driver.snapshot()['warnings']
    driver.close()
    driver.connect()
    assert not driver.snapshot()['faults']


@pytest.mark.parametrize('temperature', [65., 100., float('nan')])
def test_overtemperature_and_invalid_diagnostics_refuse_control(temperature):
    driver, hand, clock, _ = driver_setup()
    driver.connect()
    value = frame(clock)
    value.joints[0].mcu_temp_c_fb = temperature
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    status = driver.snapshot()
    assert status['faults'] or not status['fresh']


def test_snapshot_does_not_expose_mutable_callback_cache():
    driver, hand, clock, _ = driver_setup()
    driver.connect()
    hand.states.callback(frame(clock))
    hand.diagnostics.callback(frame(clock))
    status = driver.snapshot()
    status['positions'][0] = 99
    status['diagnostics']['states'][0] = 2
    status['source_headers']['state']['sequence'] = 99
    fresh = driver.snapshot()
    assert fresh['positions'][0] == 0 and fresh['diagnostics']['states'][0] == 1
    assert fresh['source_headers']['state']['sequence'] == 1


def test_position_velocity_command_has_zero_feedforward_and_validates_before_write():
    driver, _, _, _ = driver_setup()
    sent = []
    driver.sdk.JointCommand = lambda q, v, effort: (q, v, effort)
    driver.publisher = NS(send=sent.append)
    driver.send([.1]*20, [.2]*20)
    assert sent == [[(.1, .2, 0.)]*20]
    driver.send([.1]*20)
    assert sent[-1] == [(.1, 0., 0.)]*20
    for bad in [[.6]*20, [float('nan')]*20, [True]*20, [.2]*19]:
        with pytest.raises(ValueError):
            driver.send([.1]*20, bad)
    assert len(sent) == 2


def test_serial_side_and_private_connection_without_motor_commands():
    driver, hand, _, connection = driver_setup()
    driver.connect()
    assert connection['sn'] == 'FAKE-HAND-ONLY'
    assert connection['options']['enable_bridge'] is False
    assert not driver.snapshot()['fresh'] and not hand.writes
    driver.close()
    driver.close()
    assert hand.states.closed and hand.diagnostics.closed and hand.disconnected


@pytest.mark.parametrize('emergency_fails', [False, True])
def test_emergency_also_disables_even_when_first_rpc_fails(emergency_fails):
    driver, hand, _, _ = driver_setup()
    driver.connect()
    def emergency():
        hand.writes.append('emergency')
        if emergency_fails:
            raise ConnectionError('emergency RPC failed')
    hand.emergency_stop = emergency
    hand.disable = lambda: hand.writes.append('disable')
    if emergency_fails:
        with pytest.raises(ConnectionError, match='emergency RPC failed'):
            driver.stop()
    else:
        driver.stop()
    assert hand.writes == ['emergency', 'disable']


@pytest.mark.parametrize('key,value', [('serial_number', 'ANOTHER-HAND'), ('side', 'left')])
def test_wrong_device_is_disconnected_without_writes(key, value):
    driver, hand, _, _ = driver_setup()
    setattr(hand, key, value)
    with pytest.raises(RuntimeError, match='serial|handedness'):
        driver.connect()
    assert hand.disconnected and not hand.writes


def test_node_mapping_handles_reordered_frames_and_drops_duplicates():
    driver, hand, clock, _ = driver_setup()
    driver.connect()
    value = frame(clock)
    value.joints.reverse()
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    assert driver.snapshot()['fresh']
    assert driver.snapshot()['positions'] == [i/100 for i in range(20)]
    original = driver.snapshot()['received_monotonic']['state']
    clock.now += .3
    hand.states.callback(value)  # Old duplicate cannot revive feedback.
    assert driver.snapshot()['received_monotonic']['state'] == original
    assert not driver.snapshot()['fresh']


@pytest.mark.parametrize('bad', ['missing', 'duplicate', 'tactile', 'nan'])
def test_bad_state_is_marked_invalid_without_fabricating_zero_positions(bad):
    driver, hand, clock, _ = driver_setup()
    driver.connect()
    value = frame(clock)
    if bad == 'missing':
        value.joints.pop()
    elif bad == 'duplicate':
        value.joints.append(value.joints[0])
    elif bad == 'tactile':
        value.joints[-1].nid = 5
    else:
        value.joints[-1].position = float('nan')
    hand.states.callback(value)
    hand.diagnostics.callback(frame(clock))
    state = driver.snapshot()
    assert not state['fresh'] and state['positions'] is None and state['stream_errors']


def test_diagnostic_dropout_not_hidden_by_fresh_positions_and_recovery_needs_new_frame():
    driver, hand, clock, _ = driver_setup()
    driver.connect()
    hand.states.callback(frame(clock))
    hand.diagnostics.callback(frame(clock))
    clock.now += .3
    hand.states.callback(frame(clock, 2))
    assert not driver.snapshot()['fresh']
    hand.diagnostics.callback(frame(clock, 2))
    assert driver.snapshot()['fresh']
    hand.diagnostics.error(RuntimeError('stream broken'))
    clock.now += .01
    hand.states.callback(frame(clock, 3))
    assert not driver.snapshot()['fresh']


def test_timestamp_backlog_and_clock_regression_refuse_freshness():
    driver, hand, clock, _ = driver_setup()
    driver.connect()
    hand.states.callback(frame(clock))
    hand.diagnostics.callback(frame(clock))
    old = frame(clock, 2)
    clock.now += .4
    hand.states.callback(old)
    assert 'backlog' in driver.snapshot()['stream_errors']['state']
    old.header.seq = 3
    old.header.timestamp_us -= 1
    hand.states.callback(old)
    assert 'regressed' in driver.snapshot()['stream_errors']['state']


def test_profile_must_read_back_all_twenty_joints():
    driver, hand, _, _ = driver_setup()
    driver.connect()
    hand.gains.set = lambda values: setattr(hand.gains, 'value', [NS(kp=k, kd=d) for k, d in values])
    driver.apply_profile()
    assert driver.profile['current_limit_a'] == [.5]*20
    hand.gains.get = lambda: [None]*20
    with pytest.raises(RuntimeError, match='incomplete'):
        driver.apply_profile()


def test_device_lease_excludes_another_session_and_releases_after_close(tmp_path, monkeypatch):
    monkeypatch.setattr('services.wuji.sdk.private_directory', lambda: tmp_path)
    first = DeviceLease('same-hand')
    try:
        with pytest.raises(BlockingIOError):
            DeviceLease('same-hand')
    finally:
        first.close()
    DeviceLease('same-hand').close()
