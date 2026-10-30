"""Stalled commands report promptly without accepting an inaccurate endpoint."""
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_xarm_backend import FakeArm, START_Q, make_backend as make_backend
from test_xarm_faults import clock as clock
from test_wuji_backend import make_wuji
from services.controller import operator
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.xarm import arming
from services.xarm.progress import MotionProgress


class ProgressArm(FakeArm):
    def __init__(self, clock, *, idle=True, accurate=False, duration=9.):
        super().__init__()
        self.clock, self.idle, self.accurate, self.duration = clock, idle, accurate, duration
        self.started = None
        self.stopped = False

    def move_pose(self, position, quaternion, **kwargs):
        self.commands += 1
        self.started = self.clock[0]
        self.start_pos = self.pos.copy()
        self.goal_pos = np.asarray(position)
        self.goal_quat = np.asarray(quaternion)
        self.kind = 'pose'

    def move_joints(self, joints, **kwargs):
        self.commands += 1
        self.started = self.clock[0]
        self.start_q = self.q.copy()
        self.goal_q = np.asarray(joints)
        self.kind = 'joints'

    def state(self):
        fraction = 0 if self.started is None else min(1., (self.clock[0]-self.started)/self.duration)
        if self.started is not None and not self.stopped:
            if self.kind == 'pose':
                self.pos = self.start_pos + fraction*(self.goal_pos-self.start_pos)
                drift = 0 if self.accurate else 2*fraction
                self.quat = (Rotation.from_euler('z', drift, degrees=True)
                             * Rotation.from_quat(self.goal_quat)).as_quat()
            else:
                endpoint = self.goal_q.copy()
                if not self.accurate:
                    endpoint[0] -= .05
                self.q = self.start_q + fraction*(endpoint-self.start_q)
        result = super().state()
        result['state'] = 4 if self.stopped else 2 if self.idle and fraction >= 1 else 1
        return result

    def hold(self):
        super().hold()
        self.stopped = True


@pytest.mark.parametrize('idle,deadline', [(True, 2.), (False, 5.)])
def test_position_reached_but_rotation_short_stops_and_posts_operator_alert(make_backend, clock, idle, deadline):
    backend, arm, hand, root = make_wuji(make_backend, arm=ProgressArm(clock, idle=idle),
                                        stream_enabled=False)
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call('xarm_move_pose', {'position': [.42, 0, .277], 'speed_mps': .005,
                                                'timeout_s': 120.})
        error = json.loads(response['content'][0]['text'])['error']
        assert response['isError'] and error['operator_required']
        assert 'pose incomplete' in error['reason']
        assert 'rotation error 2.00 deg' in error['reason']
        assert 9.+deadline-.3 <= clock[0]-arm.started < 9.+deadline+3.
        assert error['robot_state']['rotation_error_deg'] == pytest.approx(2.)
        assert error['robot_state']['position_error_mm'] == pytest.approx(0.)
        assert not error['control_uncertain']
        assert not arming.read(root)['armed'] and hand.stops and arm.holds
        assert len(operator.pending(root)) == 1
        assert error['operator_request']['id'] == operator.pending(root)[0]['id']
        assert arm.commands == 1  # No retry or orientation correction.
    finally:
        front.close()


def test_incomplete_joint_segment_cancels_remaining_segments(make_backend, clock):
    backend, arm, hand, root = make_wuji(make_backend, arm=ProgressArm(clock, duration=3.),
                                        stream_enabled=False)
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call('xarm_move_joints', {'positions': (START_Q + [2.25, 0, 0, 0, 0, 0]).tolist()})
        error = json.loads(response['content'][0]['text'])['error']
        assert 'joint move incomplete' in error['reason'] and arm.commands == 1
        assert error['robot_state']['max_joint_error_rad'] == pytest.approx(.05)
        assert len(operator.pending(root)) == 1 and hand.stops
    finally:
        front.close()


def test_normal_completed_motion_is_not_stopped(make_backend, clock):
    backend, arm, hand, root = make_wuji(make_backend, arm=ProgressArm(clock, accurate=True),
                                        stream_enabled=False)
    arming.write(root, True)
    result, _ = backend.call('xarm_move_pose', {'position': [.42, 0, .277]})
    assert result['reached'] and result['seconds'] < 10
    assert not arm.holds and not hand.stops and not operator.pending(root)


@pytest.mark.parametrize('motion', ['translation', 'rotation', 'joints'])
def test_slow_motion_in_any_coordinate_keeps_progress_alive(motion):
    guard = MotionProgress()
    initial = FakeArm().state()
    initial['state'] = 2  # Measured progress also wins over premature idle feedback.
    for t in np.arange(0, 20, .05):
        state = {**initial}
        if motion == 'translation':
            state['tcp_position'] = initial['tcp_position'] + [0, 0, .001*t]
        elif motion == 'rotation':
            state['tcp_quaternion'] = (Rotation.from_euler('z', .2*t, degrees=True)
                                       * Rotation.from_quat(initial['tcp_quaternion'])).as_quat()
        else:
            state['joints'] = initial['joints'] + [.002*t, 0, 0, 0, 0, 0]
        assert guard.check(state, t) is None


@pytest.mark.parametrize('controller_state,seconds', [(0, 5.), (1, 5.), (2, 2.)])
def test_small_feedback_noise_cannot_reset_stationary_deadline(controller_state, seconds):
    guard = MotionProgress()
    state = FakeArm().state()
    state['state'] = controller_state
    p = state['tcp_position'].copy()
    for i in range(int(seconds/.05)+2):
        state['tcp_position'] = p + [1e-5*(-1)**i, 0, 0]
        reason = guard.check(state, i*.05)
        if reason:
            break
    assert reason is not None
    assert i*.05 == pytest.approx(seconds)


def test_brief_initial_idle_can_transition_to_moving():
    guard = MotionProgress()
    state = FakeArm().state()
    state['state'] = 2
    assert guard.check(state, 0) is None
    assert guard.check(state, 1.5) is None
    state['state'] = 1
    state['tcp_position'] += [0, 0, .001]
    assert guard.check(state, 1.6) is None
    assert guard.check(state, 3) is None
