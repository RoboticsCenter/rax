"""Partial-hand invariants, exercised exclusively with fake hardware."""
from copy import deepcopy
import json
import multiprocessing
import threading
import time
from types import SimpleNamespace as NS

import pytest

from test_wuji_sdk import Hand, driver_setup, frame
from test_xarm_backend import make_backend as make_backend
from wuji_fakes import Clock, FakeHand, hand_config
from services.wuji.config import JOINT_NAMES, validate_tool
from services.wuji.controller import Controller, Trajectory
from services.wuji.partial import ISOLATABLE, active_indices, control_enabled, effective_parameters
from services.wuji.preparation import preparation_action

EXCLUDED = ['thumb_S3', 'middle_S1']
RULES = {'thumb_S3': {'code': 0x2108, 'firmware': '2.6.0'},
         'middle_S1': {'code': 0x3208, 'firmware': '2.6.0'}}


def cfg(**kw):
    return hand_config(disabled_joints=EXCLUDED, isolated_joint_faults=RULES, **kw)


@pytest.mark.parametrize('value', [True, 'thumb_S3', ['bad'], ['thumb_S3']*2, list(JOINT_NAMES), [2]])
def test_invalid_exclusions_rejected(value):
    with pytest.raises(ValueError, match='disabled_joints'):
        hand_config(disabled_joints=value)


@pytest.mark.parametrize('rules', [True, {'index_S1': RULES['thumb_S3']},
    {'thumb_S3': {'code': 0x2102, 'firmware': '2.6.0'}},
    {'thumb_S3': {'code': 0x2108, 'firmware': 'latest'}},
    {'thumb_S3': {'code': 0x2108, 'firmware': '2.6.0', 'ignore': True}}])
def test_isolation_policy_is_exact_and_excludes_electrical_faults(rules):
    with pytest.raises(ValueError, match='isolated_joint_faults'):
        hand_config(disabled_joints=EXCLUDED, isolated_joint_faults=rules)


def test_no_combined_admission_exception():
    with pytest.raises(ValueError, match='cannot combine'):
        cfg(allow_init_current_calib_failed={'middle_S1': '2.6.0'})


def sdk_setup(monkeypatch, **options):
    driver, hand, clock, _ = driver_setup(disabled_joints=EXCLUDED, isolated_joint_faults=RULES, **options)
    def describe(code):
        if code in ISOLATABLE:
            return dict(zip(('name', 'severity', 'clear_policy'), ISOLATABLE[code]))
        return {'name': 'Other', 'severity': 'ImmediateStop', 'clear_policy': 'ManualClear'}
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(describe), raising=False)
    driver.connect()
    value = frame(clock)
    for i, code in ((2, 0x2108), (8, 0x3208)):
        value.joints[i].error_code_current = code
        value.joints[i].status_word.ext_state = 3
        value.joints[i].status_word.ext_state_name = 'Stopped'
    def emit():
        clock.now += .01
        value.header.seq += 1
        value.header.timestamp_us = int(clock()*1e6)
        hand.states.callback(value)
        hand.diagnostics.callback(value)
    emit()
    return driver, hand, clock, value, emit


def test_isolation_preserves_native_faults_and_does_not_fake_full_enable(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    before = driver.snapshot()
    assert not before['faults'] and len(before['isolated_faults']) == 2
    assert before['diagnostics']['error_code_current'][8] == 0x3208
    assert before['isolated_faults'][1]['severity'] == 'Fatal'
    assert preparation_action(before, driver.cfg) == 'prepare'
    for i in active_indices(driver.cfg):
        value.joints[i].status_word.ext_state = 2
        value.joints[i].status_word.ext_state_name = 'Enabled'
    emit()
    state = driver.snapshot()
    assert state['active_enabled'] and not state['enabled']
    assert state['partial_hand'] and len(state['active_joints']) == 18
    state['isolated_faults'][0]['code'] = 0
    assert driver.snapshot()['isolated_faults'][0]['code'] == 0x2108
    clock.now += 1
    assert not driver.snapshot()['active_enabled']


@pytest.mark.parametrize('change', ['old_owner', 'different_mask', 'active_joint_enabled'])
def test_preparation_refuses_owner_profile_mismatch_or_any_enabled_joint(monkeypatch, change):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    state = driver.snapshot()
    if change == 'old_owner':
        state.pop('partial_hand'); state.pop('disabled_joints')
    elif change == 'different_mask':
        state['disabled_joints'] = ['thumb_S3']
    else:
        state['diagnostics']['states'][4] = 2
    with pytest.raises(RuntimeError, match='not loaded|confirmed disabled'):
        preparation_action(state, driver.cfg)


@pytest.mark.parametrize('change', ['firmware', 'metadata', 'new_code', 'active_fault', 'excluded_enabled',
                                  'excluded_init', 'temperature', 'missing_feedback'])
def test_unapproved_conditions_still_block_and_latch(monkeypatch, change):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    if change == 'firmware': driver.identity['firmware'] = '2.13.1'
    elif change == 'metadata': driver.error_details[0x2108]['severity'] = 'Fatal'
    elif change == 'new_code': value.joints[2].error_code_current = 0x2102
    elif change == 'active_fault': value.joints[4].error_code_current = 0x2108
    elif change == 'excluded_enabled': value.joints[2].status_word.ext_state = 2
    elif change == 'excluded_init': value.joints[2].status_word.ext_state = 0
    elif change == 'temperature': value.joints[8].mcu_temp_c_fb = 70
    elif change == 'missing_feedback': value.joints.pop()
    emit()
    state = driver.snapshot()
    assert state['faults'] or not state['fresh']
    with pytest.raises(RuntimeError):
        preparation_action(state, driver.cfg)
    if state['faults']:
        value.joints[2].error_code_current = 0x2108
        value.joints[2].status_word.ext_state = 3
        emit()
        assert driver.snapshot()['faults']  # No automatic recovery from transient isolation violations.


def test_masked_enable_zero_profile_and_wire_commands(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    hand.gains.set = lambda values: setattr(hand.gains, 'value', [NS(kp=p, kd=d) for p, d in values])
    hand.disable = lambda mask: hand.writes.append(('disable', mask))
    hand.enable = lambda mask: hand.writes.append(('enable', mask))
    monkeypatch.setattr('services.wuji.sdk.time.sleep', lambda _: emit())
    commands = []
    driver.sdk.JointCommand = lambda q, v, a: (q, v, a)
    hand.joint_command = lambda: NS(publish=lambda: NS(send=lambda value: commands.append(value)))
    driver.apply_profile()
    assert driver.profile == effective_parameters(driver.cfg)
    assert all(driver.profile[key][i] == 0 for key in ('kp', 'kd', 'current_limit_a') for i in (2, 8))
    q = driver.snapshot()['positions']
    driver.send(q)
    driver.enable()
    assert hand.writes == [('disable', [int(i in (2, 8)) for i in range(20)]),
                           ('enable', [int(i not in (2, 8)) for i in range(20)])]
    q[4] += .02
    driver.send(q)
    assert all(commands[-1][i] == commands[0][i] for i in (2, 8))
    q[2] += .01
    with pytest.raises(ValueError, match='disabled'):
        driver.send(q)
    velocity = [0.]*20; velocity[8] = .01
    with pytest.raises(ValueError, match='velocity or effort'):
        driver.send([v[0] for v in commands[0]], velocity)
    with pytest.raises(ValueError, match='operator profile'):
        driver.set_current_limits([.1]*20)


def test_no_enable_when_excluded_disable_feedback_never_arrives(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch, stop_timeout_s=.1)
    driver.profile = effective_parameters(driver.cfg)
    driver.disabled_reference = driver.snapshot()['positions']
    hand.disable = lambda mask: hand.writes.append(('disable', mask))
    hand.enable = lambda mask: hand.writes.append(('enable', mask))
    with pytest.raises(RuntimeError, match='disable was not confirmed'):
        driver.enable()
    assert [x[0] for x in hand.writes] == ['disable']


def test_zero_gain_readback_failure_prevents_enable(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    driver.profile = effective_parameters(driver.cfg)  # An earlier preparation must not remain trusted.
    hand.gains.set = lambda values: None  # Device ignores the requested zeros.
    with pytest.raises(RuntimeError, match='readback mismatch'):
        driver.apply_profile()
    assert driver.profile is None and not hand.writes
    with pytest.raises(RuntimeError, match='verified profile'):
        driver.enable()


def test_stop_during_disable_confirmation_cannot_be_followed_by_enable(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    driver.profile = effective_parameters(driver.cfg)
    driver.disabled_reference = driver.snapshot()['positions']
    hand.disable = lambda mask: hand.writes.append(('disable', mask))
    hand.enable = lambda mask: hand.writes.append(('enable', mask))
    cancelled = []
    def waiting(_):
        emit(); cancelled.append(True)
    def check():
        if cancelled: raise RuntimeError('priority stop')
    monkeypatch.setattr('services.wuji.sdk.time.sleep', waiting)
    with pytest.raises(RuntimeError, match='priority stop'):
        driver.enable(check=check)
    assert [x[0] for x in hand.writes] == ['disable']


def test_firmware_ignoring_enable_mask_is_immediately_latched(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    for joint in value.joints:
        joint.status_word.ext_state = 2
        joint.status_word.ext_state_name = 'Enabled'
    emit()
    s = driver.snapshot()
    assert s['enabled'] and not s['active_enabled']  # Truthful native state, unsafe isolation.
    assert any(f.get('reason') == 'excluded joint is not disabled' for f in s['faults'])
    with pytest.raises(RuntimeError):
        preparation_action(s, driver.cfg)


def test_isolation_history_does_not_erase_a_preexisting_latched_fault(monkeypatch):
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    driver.cfg['isolated_joint_faults'] = {}
    emit(); assert driver.snapshot()['faults']
    driver.cfg['isolated_joint_faults'] = deepcopy(RULES)
    emit(); state = driver.snapshot()
    assert state['isolated_faults'] and state['faults']
    history = state['isolated_fault_history']
    hand.diagnostics.callback(value)
    assert driver.snapshot()['isolated_fault_history'] == history  # Duplicates cannot refresh proof.


def test_client_accepts_only_confirmed_active_subset(monkeypatch):
    from services.wuji.client import WujiClient
    engine, hand, step = rig()
    client = WujiClient.__new__(WujiClient); client.cfg = engine.cfg; client.path = '/fake-only'
    monkeypatch.setattr('services.wuji.client.request', lambda *a: engine.status())
    assert client.require_ready()['active_enabled']
    hand.states[2] = 2; engine._publish()
    with pytest.raises(RuntimeError, match='not prepared'):
        client.require_ready()


def test_operator_readiness_checks_native_excluded_states():
    from scripts.prepare_xarm_wuji import hand_ready
    from services.xarm_bimanual.operator import ready_hand
    engine, hand, step = rig()
    state = engine.status()
    assert hand_ready(state) and ready_hand(state)
    state['diagnostics']['states'][2] = 2
    assert state['active_enabled']  # A stale convenience flag cannot override native states.
    assert not hand_ready(state) and not ready_hand(state)


class PartialFake(FakeHand):
    def enable(self, *, check=None):
        if check: check()
        self.writes.append(('enable',))
        for i in active_indices(self.cfg):
            self.states[i], self.names[i] = 2, 'Enabled'

    def refresh(self, state=True, diagnostics=True):
        previous = list(self.q)
        super().refresh(state, diagnostics)
        for i in (2, 8):
            self.q[i] = previous[i]


def rig(mode='timed'):
    config = cfg(motion_mode=mode, command_hz=1000 if mode == 'scurve' else 120)
    clock = Clock(); hand = PartialFake(config, clock); hand.connect()
    engine = Controller(config, hand, lambda: True, clock=clock)
    engine.mode = 'disabled'
    def step():
        clock.now += .001 if mode == 'scurve' else .01
        engine.heartbeat(); hand.refresh(); engine.tick()
    engine.submit('prepare', 'prepare'); step(); step()
    assert engine.result('prepare')['ok']
    return engine, hand, step


@pytest.mark.parametrize('mode', ['timed', 'feedback', 'scurve'])
def test_active_joints_move_and_passive_drift_is_not_reported_as_tracking(mode):
    engine, hand, step = rig(mode)
    q = list(engine.target); q[4] += .04
    engine.submit('move', 'move', {'positions': q})
    for _ in range(8000):
        hand.q[2] = .4; hand.q[8] = -.3
        step()
        if engine.result('move')['done']: break
    result = engine.result('move')
    assert result['ok'] and result['completion_scope'] == 'active_joints'
    assert hand.q[4] == pytest.approx(.04, abs=.005)
    assert not engine.status()['enabled'] and engine.status()['active_enabled']
    assert engine.status()['positions'][2] == .4  # Raw measured position is never rewritten.
    assert all(w[1][2] == 0. and w[1][8] == 0. for w in hand.writes if w[0] == 'send')
    engine.submit('stop', 'emergency'); step(); step()
    assert engine.result('stop')['ok'] and not engine.owns_control


@pytest.mark.parametrize('mode', ['timed', 'feedback', 'scurve'])
@pytest.mark.parametrize('violation', ['excluded_enabled', 'active_disabled', 'active_fault', 'stale', 'tracking'])
def test_runtime_violation_stops_every_joint(mode, violation):
    engine, hand, step = rig(mode)
    if violation == 'excluded_enabled': hand.states[2], hand.names[2] = 2, 'Enabled'
    elif violation == 'active_disabled': hand.states[4], hand.names[4] = 1, 'Ready'
    elif violation == 'active_fault': hand.faults = [{'joint': 'index_S1', 'code': 0x2102}]
    elif violation == 'stale': hand.connected = False
    elif violation == 'tracking': hand.follow = False; hand.q[4] = .8
    step(); step()
    assert ('emergency',) in hand.writes
    assert engine.mode != 'holding'


def test_excluded_target_and_other_control_modes_are_rejected():
    engine, hand, step = rig()
    q = list(engine.target); q[2] = .02
    with pytest.raises(ValueError, match='disabled'):
        Trajectory(engine.target, q, engine.cfg)
    for name, args in [('wuji_pose', {'name': 'test_pose'}), ('wuji_home', {}),
                       ('wuji_set_effort', {'effort_a': [0.]*20}),
                       ('wuji_set_impedance', {'kp': [.1]*20}),
                       ('wuji_move_joints', {'positions': [0.]*20, 'completion': 'contact'})]:
        with pytest.raises(ValueError, match='partial-hand'):
            validate_tool(name, args, engine.cfg)
    engine.submit('private-effort', 'set_effort', {'effort_a': [0.]*20})
    step()
    assert not engine.result('private-effort')['ok']


def test_missing_active_enable_confirmation_never_prepares():
    config = cfg(stop_timeout_s=.1); clock = Clock(); hand = PartialFake(config, clock); hand.connect()
    hand.enable = lambda **kwargs: None
    engine = Controller(config, hand, lambda: True, clock=clock); engine.mode = 'disabled'
    engine.submit('prepare', 'prepare')
    for _ in range(20):
        clock.now += .01; engine.heartbeat(); hand.refresh(); engine.tick()
    assert not engine.result('prepare')['ok'] and ('emergency',) in hand.writes


def serve_partial(config, root, path):
    from services.wuji.controller import serve
    serve(config, root, path, driver=PartialFake(config, continuous=True))


@pytest.mark.parametrize('mode', ['timed', 'feedback', 'scurve'])
def test_mcp_to_private_controller_process_completes_only_active_motion(make_backend, tmp_path, monkeypatch, mode):
    from services.wuji.client import WujiClient, operation, request
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    from services.xarm import arming
    from test_wuji_service import until
    config = cfg(motion_mode=mode, command_hz=1000 if mode == 'scurve' else 120,
                 max_scheduler_gap_s=.4, parent_timeout_s=2.)
    path = tmp_path/'partial-fake.sock'
    class IPCClient(WujiClient):
        def __init__(self):
            self.cfg, self.path = config, path
        def status(self):
            return request(path, 'status')
        def close(self):
            pass
    client = IPCClient()
    backend, arm, root = make_backend(hand=client, end_effector=config)
    client.root = root
    monkeypatch.setattr('services.wuji.client.socket_path', lambda root: path)
    process = multiprocessing.get_context('fork').Process(target=serve_partial, args=(config, root, path))
    stop_heartbeat = threading.Event(); heartbeat = None; front = None
    process.start()
    try:
        until(lambda: path.exists())
        until(lambda: request(path, 'status').get('fresh'))
        def pulse():
            while not stop_heartbeat.is_set():
                request(path, 'heartbeat'); stop_heartbeat.wait(.03)
        heartbeat = threading.Thread(target=pulse, daemon=True); heartbeat.start()
        arming.write(root, True)
        operation(root, 'prepare')
        until(lambda: request(path, 'status').get('active_enabled'))
        front = Frontend(Session(backend.config), backend)
        target = list(request(path, 'status')['target_positions']); target[4] += .02
        response = front.call('wuji_move_joints', {'positions': target})
        assert not response.get('isError'), response
        result = json.loads(response['content'][0]['text'])
        assert result['partial_hand'] and result['completion_scope'] == 'active_joints'
        state = request(path, 'status')
        assert state['positions'][4] == pytest.approx(.02, abs=.001)
        assert state['active_enabled'] and not state['enabled']
        assert state['diagnostics']['states'][2] == state['diagnostics']['states'][8] == 1
        target[2] += .02
        refused = front.call('wuji_move_joints', {'positions': target})
        assert refused.get('isError')
        assert json.loads(refused['content'][0]['text'])['error']['no_action_executed']
        assert arm.commands == 0
        operation(root, 'emergency')
        assert not request(path, 'status')['owns_control']
    finally:
        if front: front.close()
        stop_heartbeat.set()
        if heartbeat: heartbeat.join(1)
        process.terminate(); process.join(3)
        if process.is_alive(): process.kill(); process.join(1)
