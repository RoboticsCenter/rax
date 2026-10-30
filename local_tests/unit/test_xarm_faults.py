"""Controller faults and operator recovery using simulated hardware only."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from test_xarm_backend import FakeArm, START_Q, make_backend as make_backend
from test_wuji_backend import make_wuji
from services.controller import operator
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.xarm import arming
from services.xarm.faults import ControllerFault, check_controller
from services.xarm.sdk import _code_result


def value(response):
    return json.loads(response['content'][0]['text'])


@pytest.mark.parametrize('code', [*range(1, 256), -1, 999])
def test_every_nonzero_controller_error_is_a_fault(code):
    state = {**FakeArm().state(), 'error_code': code}
    with pytest.raises(ControllerFault, match=f'controller error {code}') as fault:
        check_controller(state)
    assert fault.value.operator_required and fault.value.robot_state['error_code'] == code


@pytest.mark.parametrize('updates', [
    {'state': 3}, {'state': 4}, {'state': 5}, {'mode': 1}, {'mode': 2},
    {'error_code': None}, {'warn_code': None}, {'state': None}, {'mode': None},
    {'connected': False}, {'joints': [float('nan')]*6}, {'tcp_position': [0, 0]},
    {'tcp_quaternion': [0]*4},
])
def test_invalid_controller_feedback_and_unexpected_states_require_operator(updates):
    with pytest.raises(ControllerFault) as fault:
        check_controller({**FakeArm().state(), **updates})
    json.dumps(fault.value.robot_state, allow_nan=False)


@pytest.mark.parametrize('state', [0, 1, 2])
def test_warning_alone_does_not_stop_a_healthy_controller(state):
    report = {**FakeArm().state(), 'state': state, 'warn_code': 11}
    assert check_controller(report) is report


class ReportingArm(FakeArm):
    def __init__(self, *, failure=None, initially_faulted=False):
        super().__init__()
        self.failure = failure
        self.error = 21 if initially_faulted else 0
        self.stopped = False
        self.motion_reads = 0

    def state(self):
        state = super().state()
        state.update(error_code=self.error, state=4 if self.stopped else 0)
        if self.commands:
            if not self.holds:
                self.motion_reads += 1
            if self.failure == 'fault':
                state.update(error_code=21, state=4)
            elif self.failure == 'paused' and not self.holds:
                state['state'] = 3
            elif self.failure == 'mode' and not self.holds:
                state['mode'] = 1
            elif self.failure == 'missing':
                state['error_code'] = None
            elif self.failure == 'disconnect':
                raise ConnectionError('injected feedback loss')
        return state

    def hold(self):
        super().hold()
        self.stopped = True


@pytest.fixture
def clock(monkeypatch):
    now = [100.]
    def sleep(seconds):
        now[0] += seconds
    monkeypatch.setattr('services.xarm.backend.time', SimpleNamespace(
        monotonic=lambda: now[0], time=lambda: now[0], sleep=sleep))
    return now


@pytest.mark.parametrize('tool', ['xarm_move_pose', 'xarm_move_joints'])
@pytest.mark.parametrize('failure', ['fault', 'paused', 'mode', 'missing', 'disconnect'])
def test_fault_aborts_before_timeout_even_at_target_and_alerts_operator(make_backend, clock, tool, failure):
    arm = ReportingArm(failure=failure)
    backend, arm, hand, root = make_wuji(make_backend, arm=arm, stream_enabled=False, settle_s=0)
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    args = {'position': [.44, 0, .3]} if tool == 'xarm_move_pose' else {
        'positions': (START_Q + [2.25, 0, 0, 0, 0, 0]).tolist()}
    clips = []
    backend._schedule_motion_clip = lambda *args: clips.append(args)
    try:
        reply = front.call(tool, args)
        error = value(reply)['error']
        assert reply['isError'] and error['type'] == 'ControllerFault'
        assert error['operator_required'] and error['operator_request']['id']
        assert error['stage'] == 'robot_execution' and not error['no_action_executed']
        assert arm.commands == 1 and arm.motion_reads == 1
        assert arm.holds and hand.stops and not arming.read(root)['armed']
        assert clock[0] - 100 < 5  # No 300-second progress deadline or second joint segment.
        assert len(operator.pending(root)) == 1
        assert clips and clips[0][3] == 'failed'
        if failure == 'fault':
            assert error['robot_state']['error_code'] == 21
        assert error['control_uncertain'] is (failure == 'disconnect')
    finally:
        front.close()


@pytest.mark.parametrize('tool,args', [
    ('xarm_move_pose', {'position': [.44, 0, .3]}),
    ('wuji_move_joints', {'positions': [.1]*20}),
])
def test_preexisting_arm_fault_blocks_all_motion_and_opens_one_alert(make_backend, clock, tool, args):
    backend, arm, hand, root = make_wuji(make_backend, arm=ReportingArm(initially_faulted=True))
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    try:
        error = value(front.call(tool, args))['error']
        assert error['stage'] == 'validation' and error['no_action_executed']
        assert error['operator_required'] and not arm.commands and not hand.moves
        assert not arming.read(root)['armed'] and arm.holds and hand.stops
        again = ControllerFault('xArm controller error 21: ' + error['reason'].split(': ', 1)[1],
                                state=arm.state())
        backend.handle_control_fault(again)
        assert len(operator.pending(root)) == 1
        assert again.operator_request['id'] == error['operator_request']['id']
    finally:
        front.close()


def test_native_sdk_rejection_also_opens_operator_alert(make_backend, clock):
    class RejectedArm(ReportingArm):
        def move_pose(self, *args, **kwargs):
            _code_result(-2, 'set_position')
    backend, arm, hand, root = make_wuji(make_backend, arm=RejectedArm(), stream_enabled=False)
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    try:
        error = value(front.call('xarm_move_pose', {'position': [.44, 0, .3]}))['error']
        assert error['operator_required'] and error['robot_state']['sdk_code'] == -2
        assert len(operator.pending(root)) == 1 and not arm.commands
    finally:
        front.close()


def test_armed_telemetry_detects_idle_arm_fault_without_agent_call(make_backend, clock):
    backend, arm, hand, root = make_wuji(make_backend, arm=ReportingArm(initially_faulted=True))
    arming.write(root, True)
    backend._watch_controller(arm.state())
    backend._watch_controller(arm.state())
    assert not arming.read(root)['armed'] and arm.holds == hand.stops == 1
    assert len(operator.pending(root)) == 1
    assert backend.status()['last_controller_fault']['robot_state']['error_code'] == 21


@pytest.mark.parametrize('hand_stops', [True, False])
def test_verified_hold_clears_uncertainty_without_motion_or_arming(make_backend, clock, hand_stops):
    backend, arm, hand, root = make_wuji(make_backend, arm=ReportingArm())
    hand.fail_stop = not hand_stops
    front = Frontend(Session(backend.config), backend)
    front.control_uncertain = backend.control_uncertain = True
    try:
        reply = value(front.call('xarm_hold', {}))
        assert reply['control_certain'] is hand_stops
        assert front.status()['control_uncertain'] is (not hand_stops)
        assert not arm.commands and not hand.moves and not arming.read(root)['armed']
    finally:
        front.close()


def test_background_uncertainty_is_reported_by_frontend_status(make_backend, clock):
    backend, arm, hand, root = make_wuji(make_backend, arm=ReportingArm(initially_faulted=True))
    hand.fail_stop = True
    arming.write(root, True)
    backend._watch_controller(arm.state())
    front = Frontend(Session(backend.config), backend)
    try:
        assert value(front.call('status', {}))['control_uncertain']
    finally:
        front.close()


def test_stop_verification_does_not_clear_controller_errors(make_backend, clock):
    backend, arm, hand, root = make_wuji(make_backend, arm=ReportingArm(initially_faulted=True))
    front = Frontend(Session(backend.config), backend)
    front.control_uncertain = True
    try:
        assert value(front.call('xarm_hold', {}))['control_certain']
        assert not front.status()['control_uncertain']
        assert arm.error == 21 and not arming.read(root)['armed'] and not arm.commands
        arming.write(root, True)
        assert value(front.call('xarm_move_pose', {'position': [.44, 0, .3]}))['error']['operator_required']
        assert arm.commands == 0
    finally:
        front.close()


def test_notification_write_failure_does_not_mask_fault_or_skip_stop(make_backend, clock, monkeypatch):
    backend, arm, hand, root = make_wuji(make_backend, arm=ReportingArm(initially_faulted=True))
    arming.write(root, True)
    def fail(*args):
        raise OSError('disk full')
    monkeypatch.setattr(operator, 'fault_request', fail)
    front = Frontend(Session(backend.config), backend)
    try:
        error = value(front.call('xarm_move_pose', {'position': [.44, 0, .3]}))['error']
        assert error['type'] == 'ControllerFault' and error['operator_notification_failed']
        assert arm.holds and hand.stops and not arming.read(root)['armed']
    finally:
        front.close()


def test_an_intentional_stop_does_not_generate_a_controller_fault(make_backend, clock):
    class WatchedArm(ReportingArm):
        def hold(self):
            super().hold()
            backend._watch_controller(self.state())
    backend, arm, hand, root = make_wuji(make_backend, arm=WatchedArm())
    arming.write(root, True)
    result, _ = backend.call('xarm_hold', {})
    assert result['control_certain'] and not arming.read(root)['armed']
    assert not operator.pending(root)


def test_sdk_transport_timeout_is_not_a_motion_deadline(make_backend, clock):
    class DisconnectedArm(ReportingArm):
        def move_pose(self, *args, **kwargs):
            raise TimeoutError('command socket timed out')
    backend, arm, hand, root = make_wuji(make_backend, arm=DisconnectedArm(), stream_enabled=False)
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    try:
        error = value(front.call('xarm_move_pose', {'position': [.44, 0, .3]}))['error']
        assert error['type'] == 'ControllerFault' and error['operator_required']
        assert 'dispatch failed' in error['reason'] and len(operator.pending(root)) == 1
        assert not arming.read(root)['armed'] and not arm.commands
    finally:
        front.close()
