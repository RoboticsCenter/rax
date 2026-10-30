"""Deliberate-contact completion with the real controller and simulated joints."""
import json
import math
from types import SimpleNamespace

import pytest

from test_wuji_controller import Rig
from test_xarm_backend import make_backend as make_backend
from wuji_fakes import FakeHandClient, hand_config
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.wuji.client import WujiClient
from services.wuji.config import validate_tool
from services.xarm import arming


def rig(mode='scurve'):
    r = Rig(motion_mode=mode, command_hz=1000 if mode == 'scurve' else 100,
            max_velocity_rad_s=.6, max_acceleration_rad_s2=3., max_jerk_rad_s3=2.,
            tolerance_rad=.06, settle_s=.4, settle_timeout_s=1., max_step_rad=.5,
            max_tracking_error_rad=.35, kp=8., kd=.15, current_limit_a=1.5)
    r.prepare()
    r.driver.follow = False
    return r


def step_contact(r, *, oscillate=False, velocity=0.):
    r.driver.q = [min(q, .1253) if j < 2 else q for j, q in enumerate(r.driver.goal)]
    if oscillate:
        r.driver.q[0] += .01 * math.sin(r.clock.now*35)
    r.driver.velocity = [velocity]*20
    r.step(1/r.cfg['command_hz'])


def finish(r, *, step=step_contact, ident='move'):
    for _ in range(12000):
        step(r)
        result = r.engine.result(ident)
        if result['done']:
            return result
    pytest.fail('simulated move did not finish')


@pytest.mark.parametrize('mode', ['scurve', 'timed', 'feedback'])
def test_stable_contact_preserves_target_bias_and_gains_without_claiming_grasp(mode):
    r = rig(mode)
    r.engine.command_effort = [.05]*20
    parameters = dict(r.engine.control_parameters)
    r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'contact'})
    result = finish(r)
    assert result['ok'] and result['outcome'] == 'held_short_of_target' and not result['reached']
    assert result['stable'] and result['holding'] and result['trajectory_complete']
    assert result['contact_verified'] is False
    assert result['max_joint_error_rad'] == pytest.approx(.0747)
    assert result['shortfall_joints'] == ['thumb_S1', 'thumb_S2']
    assert result['accepted_tolerance_rad'] == .06
    assert r.engine.mode == 'holding' and r.engine.target == [.2]*20
    for _ in range(100):
        step_contact(r)
    assert r.engine.mode == 'holding' and r.engine.target != r.driver.q
    assert r.engine.command_effort == r.driver.command_effort == [.05]*20
    assert r.engine.control_parameters == parameters and not r.engine.fault
    assert not any(w[0] in ('emergency', 'disable') for w in r.driver.writes)
    # The next deliberate release is an ordinary position move from the held target.
    r.engine.submit('release', 'move', {'positions': [0.]*20})
    released = finish(r, ident='release')
    assert released['reached'] and r.engine.target == [0.]*20


@pytest.mark.parametrize('mode', ['scurve', 'timed', 'feedback'])
def test_position_completion_still_rejects_the_same_shortfall(mode):
    r = rig(mode)
    r.engine.submit('move', 'move', {'positions': [.2]*20})
    result = finish(r)
    assert not result['ok'] and not result['reached'] and result['outcome'] == 'target_not_reached'
    assert result['max_joint_error_rad'] == pytest.approx(.0747)


@pytest.mark.parametrize('mode', ['scurve', 'timed', 'feedback'])
@pytest.mark.parametrize('unstable', ['oscillating', 'moving'])
def test_contact_requires_stability_and_low_measured_velocity(mode, unstable):
    r = rig(mode)
    r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'contact'})
    result = finish(r, step=lambda r: step_contact(r, oscillate=unstable == 'oscillating',
                                                 velocity=.06 if unstable == 'moving' else 0.))
    assert not result['ok'] and result['outcome'] == 'target_not_reached'


def test_contact_mode_does_not_waive_tracking_bound_during_closure_or_hold():
    r = rig()
    r.engine.submit('move', 'move', {'positions': [.5]*20, 'completion': 'contact'})
    result = finish(r)
    assert not result['ok'] and result['outcome'] == 'fault' and 'tracking error' in result['reason']
    r = rig()
    r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'contact'})
    assert finish(r)['ok']
    r.driver.q[0] = -.2
    r.step(.001)
    assert r.engine.mode == 'faulted' and 'tracking error' in r.engine.fault


def test_contact_mode_does_not_accept_stale_feedback_or_native_faults():
    for fault in ('stale', 'encoder'):
        r = rig()
        r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'contact'})
        step_contact(r)
        if fault == 'encoder':
            r.driver.faults = [{'code': 4, 'reason': 'injected encoder fault'}]
        for _ in range(400):
            r.step(.001, state=fault != 'stale')
            if r.engine.result('move')['done']:
                break
        assert r.engine.result('move')['outcome'] == 'fault'


def test_feedback_reference_that_pauses_before_goal_is_not_contact_completion():
    r = rig('feedback')
    r.cfg.update(feedback_slow_error_rad=.01, feedback_pause_error_rad=.03)
    r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'contact'})
    result = finish(r, step=lambda r: r.step(.01))
    assert not result['ok'] and result['outcome'] == 'target_not_reached'
    assert not result['trajectory_complete']


def test_completion_policy_is_validated_by_the_owner_as_well_as_mcp():
    r = rig()
    before = len(r.driver.writes)
    r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'disable_checks'})
    r.step(.001)
    result = r.engine.result('move')
    assert result['outcome'] == 'rejected' and not result['ok']
    assert r.engine.target == [0.]*20 and r.engine.mode == 'holding'
    assert all(write[1] == [0.]*20 for write in r.driver.writes[before:] if write[0] == 'send')


@pytest.mark.parametrize('mode', ['scurve', 'timed', 'feedback'])
def test_contact_result_waits_for_final_send_and_cannot_mask_write_failure(mode):
    r = rig(mode)
    def fail_final_send():
        if r.engine.active and r.engine.active.get('completion', (None,))[0] == 'held_short_of_target':
            raise ConnectionError('injected final send failure')
    r.driver.on_send = fail_final_send
    r.engine.submit('move', 'move', {'positions': [.2]*20, 'completion': 'contact'})
    result = finish(r)
    assert result['outcome'] == 'fault' and not result['ok']


@pytest.mark.parametrize('completion', [None, True, 'anything', 1, [], {}])
def test_invalid_completion_policy_is_rejected(completion):
    with pytest.raises(ValueError, match='completion'):
        validate_tool('wuji_move_joints', {'positions': [0]*20, 'completion': completion}, hand_config())


@pytest.mark.parametrize('tool,args', [('wuji_home', {}), ('wuji_pose', {'name': 'test_pose'})])
def test_home_and_named_poses_cannot_opt_out_of_position_accuracy(tool, args):
    with pytest.raises(ValueError, match='unknown arguments'):
        validate_tool(tool, {**args, 'completion': 'contact'}, hand_config())


@pytest.mark.parametrize('completion', ['contact', 'position'])
def test_controller_result_survives_client_and_mcp_without_disabling_contact_hold(make_backend, monkeypatch, completion):
    r = rig()
    hand = FakeHandClient(r.cfg)
    hand.status = hand.require_ready = r.engine.status
    backend, arm, root = make_backend(hand=hand, end_effector=r.cfg, stream_enabled=False)
    arming.write(root, True)
    client = WujiClient.__new__(WujiClient)
    client.root, client.cfg = root, r.cfg
    hand.move = client.move
    def request(path, op, **kwargs):
        if op == 'move':
            return r.engine.submit(kwargs['id'], op, kwargs['args'])
        if op == 'result':
            step_contact(r)
            return r.engine.result(kwargs['id'])
        raise AssertionError(op)
    monkeypatch.setattr('services.wuji.client.request', request)
    monkeypatch.setattr('services.wuji.client.time', SimpleNamespace(monotonic=r.clock, sleep=lambda _: None))
    front = Frontend(Session(backend.config), backend)
    try:
        reply = front.call('wuji_move_joints', {'positions': [.2]*20, 'completion': completion})
        result = json.loads(reply['content'][0]['text'])
        if completion == 'contact':
            assert not reply.get('isError') and result['outcome'] == 'held_short_of_target'
            assert not result['reached'] and result['holding'] and not result['contact_verified']
            assert not arm.holds and not hand.stops and not front.control_uncertain
            assert arming.read(root)['armed'] and r.engine.status()['enabled']
        else:
            assert reply['isError'] and arm.holds and hand.stops
    finally:
        front.close()
