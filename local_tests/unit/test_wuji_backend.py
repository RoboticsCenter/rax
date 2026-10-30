import json
import socket
import threading
from pathlib import Path

import pytest
from test_xarm_backend import FakeArm
from test_xarm_backend import (
    make_backend as make_backend,  # Re-export the shared pytest fixture.
)
from wuji_fakes import FakeHandClient, hand_config

from auto_research_agent.api.runtime import Context
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.workspace import Setup, build, recipe_for
from services.xarm import arming


def make_wuji(make_backend, *, hand_options=None, **kwargs):
    cfg = hand_config(**(hand_options or {}))
    hand = FakeHandClient(cfg)
    backend, arm, root = make_backend(hand=hand, end_effector=cfg, **kwargs)
    return backend, arm, hand, root


def test_wuji_tools_replace_gripper_and_status_has_real_hand_shape(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    names = {t['name'] for t in backend.tools()}
    assert {'wuji_status', 'wuji_move_joints', 'wuji_pose', 'wuji_home', 'wuji_hold', 'xarm_move_pose',
            'wuji_set_effort', 'wuji_set_impedance'} <= names
    assert 'xarm_gripper' not in names
    status, _ = backend.call('xarm_status', {})
    assert status['hand']['positions'] == [0]*20 and 'gripper' not in status
    with pytest.raises(ValueError):
        backend.call('xarm_gripper', {'action': 'open'})
    with pytest.raises(PermissionError):
        backend.call('wuji_move_joints', {'positions': [.1]*20})
    assert not hand.moves
    arming.write(root, True)
    value, _ = backend.call('wuji_pose', {'name': 'test_pose'})
    assert value['reached'] and hand.moves == [[.1]*20]
    assert backend.last_motion['tool'] == 'wuji_pose'
    assert backend.mutating == {'xarm_move_pose', 'xarm_move_joints', 'wuji_move_joints', 'wuji_pose', 'wuji_home',
                               'wuji_set_effort', 'wuji_set_impedance'}


def test_hand_home_routes_to_existing_move_and_preserves_tuned_parameters(make_backend):
    home = [.1, -.1, .15, .2] * 5
    backend, arm, hand, root = make_wuji(make_backend, hand_options={
        'postures': {'home': home}, 'motion_mode': 'scurve', 'command_hz': 1000,
        'kp': 8., 'kd': .15, 'current_limit_a': 1.5})
    before = hand.status()['control_parameters']
    with pytest.raises(PermissionError):
        backend.call('wuji_home', {})
    assert not hand.moves and arm.commands == 0
    arming.write(root, True)
    result, _ = backend.call('wuji_home', {})
    assert result['reached'] and hand.moves == [home]
    assert backend.last_motion['tool'] == 'wuji_home'
    assert hand.status()['control_parameters'] == before and not hand.control_calls
    assert arm.commands == 0  # Hand-home does not move the arm implicitly.


def test_home_refuses_missing_posture_and_failed_move_stops_both_devices(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    arming.write(root, True)
    with pytest.raises(ValueError, match='home posture is not configured'):
        backend.call('wuji_home', {})
    assert not hand.moves
    backend.hand_config['postures']['home'] = [.1]*20
    hand.fail_move = True
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call('wuji_home', {})
        assert response.get('isError') and front.control_uncertain
        assert hand.stops and arm.holds
    finally:
        front.close()


def test_frontend_validation_and_uncertain_failure_are_distinct(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call('wuji_move_joints', {'positions': [0]*19})
        value = json.loads(response['content'][0]['text'])
        assert value['error']['no_action_executed'] and not front.control_uncertain and not hand.moves
        from services.wuji.client import HandError
        def failed_move(*args, **kwargs):
            raise HandError({'outcome': 'target_not_reached', 'reason': 'injected settling failure'})
        hand.move = failed_move
        response = front.call('wuji_move_joints', {'positions': [.1]*20})
        value = json.loads(response['content'][0]['text'])
        assert value['error']['control_uncertain'] and front.control_uncertain
        assert value['error']['robot_state']['hand'] is not None
        assert 'assembly_stop' in value['error']['robot_state']
        assert hand.stops and arm.holds
        # Observation/hold stays available even while uncertain and disarmed.
        arming.write(root, False)
        assert not front.call('wuji_status', {}).get('isError')
        assert not front.call('xarm_hold', {}).get('isError')
    finally:
        front.close()
    assert hand.closed


def test_hand_fault_cancels_remaining_arm_joint_segments(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    original = hand.require_ready

    def require_ready():
        if arm.commands:
            raise RuntimeError('injected hand feedback loss')
        return original()

    hand.require_ready = require_ready
    arming.write(root, True)
    with pytest.raises(RuntimeError, match='hand feedback loss'):
        backend.call('xarm_move_joints', {'positions': (arm.q + [2.25, 0, 0, 0, 0, 0]).tolist()})
    assert arm.commands == 1 and hand.stops and arm.holds


def test_hand_admission_uses_target_from_same_fresh_owner_snapshot(make_backend, monkeypatch):
    backend, arm, hand, root = make_wuji(make_backend)
    arming.write(root, True)
    fresh = {**hand.status(), 'target_positions': [.4]*20}
    monkeypatch.setattr(hand, 'require_ready', lambda: fresh)
    # Simulate a parent cache predating preparation. The next move must use
    # the owner's reference, not this missing/stale target.
    monkeypatch.setattr(hand, 'status', lambda: {**fresh, 'target_positions': None})
    result, _ = backend.call('wuji_move_joints', {'positions': [.5]*20})
    assert result['reached'] and hand.moves == [[.5]*20]


def test_each_device_is_stopped_even_when_the_other_stop_fails(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    hand.fail_stop = True
    value, _ = backend.call('xarm_hold', {})
    assert arm.holds and not value['assembly_stop']['hand']['sdk_returned']
    assert not value['assembly_stop']['arm']['confirmed']
    def fail():
        raise RuntimeError('arm transport failure')
    arm.hold = fail
    hand.fail_stop = False
    value, _ = backend.call('xarm_hold', {})
    assert value['assembly_stop']['hand']['confirmed']
    assert 'arm transport failure' in value['assembly_stop']['arm']['error']


def test_wuji_fault_rejects_arm_motion_before_dispatch(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    arming.write(root, True)
    def not_ready():
        raise RuntimeError('stale diagnostics')
    hand.require_ready = not_ready
    with pytest.raises(RuntimeError, match='stale'):
        backend.call('xarm_move_pose', {'position': [.44, 0, .3]})
    assert arm.commands == 0


@pytest.mark.parametrize('failure', [None, 'state', 'joint_drift', 'rotation_drift', 'fault',
                                   'nonfinite', 'feedback_loss', 'hand_stop'])
def test_arm_timeout_only_latches_uncertainty_when_stop_is_unconfirmed(make_backend, monkeypatch, failure):
    import numpy as np
    from scipy.spatial.transform import Rotation

    class StoppingArm(FakeArm):
        def state(self):
            state = super().state()
            if self.holds:
                state['state'] = 4
                if failure == 'state':
                    state['state'] = 1
                elif failure == 'joint_drift':
                    self.q = self.q + .002
                    state['joints'] = self.q.copy()
                elif failure == 'rotation_drift':
                    self.quat = (Rotation.from_rotvec([0, 0, .02]) * Rotation.from_quat(self.quat)).as_quat()
                    state['tcp_quaternion'] = self.quat.copy()
                elif failure == 'fault':
                    state['error_code'] = 19
                elif failure == 'nonfinite':
                    state['tcp_position'] = np.full(3, np.nan)
                elif failure == 'feedback_loss':
                    raise ConnectionError('feedback disconnected after stop ACK')
            return state

    backend, arm, hand, root = make_wuji(make_backend, arm=StoppingArm(), stream_enabled=False)
    arming.write(root, True)
    if failure == 'hand_stop':
        hand.fail_stop = True

    def expire(*args, **kwargs):
        raise TimeoutError('injected arm motion deadline')

    monkeypatch.setattr(backend, '_monitor_pose', expire)
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call('xarm_move_pose', {'position': [.44, 0, .3]})
        assert response['isError']
        error = json.loads(response['content'][0]['text'])['error']
        assert error['type'] == 'TimeoutError' and not error['no_action_executed']
        assert error['control_uncertain'] is (failure is not None)
        assert front.status()['control_uncertain'] is (failure is not None)
        stop = error['robot_state']['assembly_stop']
        if failure is None:
            assert stop['arm']['confirmed'] and stop['hand']['confirmed']
            assert stop['arm']['samples'] >= 5 and stop['arm']['stationary_s'] >= .4
        else:
            assert not all(item['confirmed'] for item in stop.values())
        assert arm.commands == 1 and arm.holds and hand.stops  # No reset or replay.
    finally:
        front.close()


def test_joint_guard_failure_stops_both_devices(make_backend):
    def outside(arm):
        arm.pos[2] = -1
    backend, arm, hand, root = make_wuji(make_backend, arm=FakeArm(on_step=outside))
    arming.write(root, True)
    with pytest.raises(ValueError, match='workspace box'):
        backend.call('xarm_move_joints', {'preset': 'home'})
    assert arm.holds and hand.stops


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_workspace_contract_matches_backend_and_installs_hand_skill(make_backend, tmp_path, profile):
    backend, _, _, _ = make_wuji(make_backend)
    setup = Setup(robot='xarm', observation_profile=profile, backend_options=backend.config.backend_options, gemini=False)
    workspace = tmp_path/profile
    manifest = build(recipe_for(), setup, workspace)
    names = {t['name'] for t in backend.tools()} | {'status', 'request_operator'}
    assert set(manifest['contract']['tools']) == names
    assert manifest['contract']['end_effector'] == 'wuji_hand2'
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text
        assert 'xarm_gripper' not in text
        assert 'gripper open' not in text
    assert (workspace/'.agents/skills/hand-control/SKILL.md').is_file()
    assert 'Wuji Hand 2' in (workspace/'AGENTS.md').read_text()
    assert 'wuji_home' in manifest['contract']['tools']
    assert 'wuji_home' in (workspace/'AGENTS.md').read_text()
    assert 'before the next scene reset' in (workspace/'AGENTS.md').read_text()
    hand_skill = (workspace/'.agents/skills/hand-control/SKILL.md').read_text()
    assert 'wuji_home()' in hand_skill and 'xarm_move_joints(preset="home")' in hand_skill
    assert 'wuji_home' in (workspace/'.agents/skills/autonomous-control/SKILL.md').read_text()


def test_g2_workspace_retains_tools_and_omits_wuji_skill(tmp_path):
    ws = tmp_path/'g2'
    # Regeneration must remove the previous hand contract/skill too.
    build(recipe_for(), Setup(robot='xarm', backend_options={'end_effector': hand_config()}), ws)
    build(recipe_for(), Setup(robot='xarm', backend_options={'gripper_type': 'g2'}), ws)
    tools = json.loads((ws/'MCP_CONTRACT.json').read_text())['tools']
    assert 'xarm_gripper' in tools and 'wuji_move_joints' not in tools
    assert 'wuji_home' not in tools and 'wuji_home' not in (ws/'AGENTS.md').read_text()
    assert not (ws/'.agents/skills/hand-control').exists()


def test_camera_start_failure_closes_hand_and_arm(make_backend, monkeypatch):
    from test_xarm_backend import FakeCameras

    from services.xarm.backend import XArmBackend
    existing, arm, _, _ = make_wuji(make_backend)
    hand = FakeHandClient()
    closed = []
    class FailingCamera(FakeCameras):
        def __init__(self, *args, **kwargs):
            pass

        def start(self):
            raise RuntimeError('camera startup failed')

        def close(self):
            closed.append('camera')

    monkeypatch.setattr('services.xarm.backend.CameraService', FailingCamera)
    arm.close = lambda: closed.append('arm')
    with pytest.raises(RuntimeError, match='camera startup failed'):
        XArmBackend(existing.config, arm=arm, hand=hand)
    assert hand.closed and set(closed) == {'camera', 'arm'}


def test_observation_records_hand_feedback_and_profile(make_backend):
    import cv2
    import numpy as np
    backend, _, hand, _ = make_wuji(make_backend)
    def grab(payload):
        directory = Path(payload['out_dir'])
        directory.mkdir(parents=True)
        rgb = directory/'left.jpg'
        assert cv2.imwrite(str(rgb), np.zeros((10, 10, 3), dtype=np.uint8))
        return {'cameras': {'left': {'rgb': str(rgb), 'depth': None, 'K': np.eye(3).tolist(),
                                    'width': 10, 'height': 10, 'timestamp_ns': 123456789}}}
    backend.cameras.request = grab
    result, images = backend.call('xarm_observe', {})
    meta = json.loads((backend.frames_root/result['frame_id']/'meta.json').read_text())
    assert result['hand'] == meta['hand'] == hand.status()
    assert images[0]['mimeType'] == 'image/jpeg'


def test_real_mcp_socket_and_python_client_with_fake_hardware(make_backend, tmp_path):
    backend, arm, hand, root = make_wuji(make_backend, hand_options={
        'postures': {'test_pose': [.1]*20, 'home': [.05]*20}})
    arming.write(root, True)
    front = Frontend(Session(backend.config), backend)
    stop = threading.Event()
    path = tmp_path/'mcp.sock'
    with socket.socket(socket.AF_UNIX) as listener:
        listener.bind(str(path))
        listener.listen()
        thread = threading.Thread(target=front.serve_socket, args=(listener, stop), daemon=True)
        thread.start()
        try:
            with Context(str(path)) as ctx:
                listing = ctx.request('tools/list', {})
                assert 'wuji_pose' in {t['name'] for t in listing['tools']}
                assert len(ctx.value('wuji_status')['positions']) == 20
                assert ctx.value('wuji_pose', name='test_pose')['reached']
                assert ctx.value('wuji_home')['reached']
                assert hand.moves[-1] == [.05]*20
                assert ctx.value('wuji_set_impedance', kp=[1.]*20, current_limit_a=[.25]*20)['outcome'] == 'impedance_set'
                assert ctx.value('wuji_set_effort', effort_a=[.1]*20)['outcome'] == 'effort_set'
                with pytest.raises(RuntimeError):
                    ctx.value('wuji_set_effort', effort_a=[.3]*20)  # Below profile, above active cap.
                assert not front.control_uncertain  # Pure admission failure did not dispatch anything.
                with pytest.raises(RuntimeError):
                    ctx.value('wuji_set_impedance', kp=[.8]*20)  # Clear the current bias first.
                assert ctx.value('wuji_set_effort', effort_a=[0.]*20)['outcome'] == 'effort_set'
                assert ctx.value('wuji_hold')['outcome'] == 'held'
                assert ctx.value('xarm_hold')['assembly_stop']['hand']['confirmed']
                with pytest.raises(RuntimeError):
                    ctx.value('xarm_gripper', action='open')
        finally:
            stop.set()
            thread.join(2)
            front.close()


def test_snapshot_retains_hand_state_when_arm_dashboard_read_fails(tmp_path, monkeypatch):
    import time

    from services.dashboard.sessions import Sessions
    from services.json_io import write_private_json
    from services.wuji.client import recorded_status
    root = tmp_path/'sessions'/'test'
    cfg = hand_config()
    write_private_json(root/'operator-input.json', {'backend_options': {'end_effector': cfg}})
    write_private_json(root/'operator'/'wuji-state.json', {
        'recorded_unix': time.time(), 'fresh': True, 'enabled': False, 'mode': 'disabled', 'serial': cfg['serial']})
    class FailedArm(FakeArm):
        def state(self):
            raise ConnectionError('arm cable disconnected')
    dashboard = Sessions(robot_driver=FailedArm)
    monkeypatch.setattr(dashboard, 'backend_options', lambda: {'end_effector': cfg})
    monkeypatch.setattr(dashboard, 'roots', lambda: [root])
    assert dashboard.robot()['hand']['serial'] == cfg['serial']
    assert not dashboard.robot()['arm']['ok']
    assert 'gripper' not in dashboard.robot()
    write_private_json(root/'operator'/'wuji-state.json', {'recorded_unix': time.time()-10, 'fresh': True})
    assert not recorded_status(root)['fresh']


@pytest.mark.parametrize('tool,args', [
    ('wuji_set_effort', {'effort_a': [.1]*20}),
    ('wuji_set_impedance', {'kp': [.8]*20}),
])
def test_force_controls_require_arming_and_failures_stop_both_devices(make_backend, tool, args):
    backend, arm, hand, root = make_wuji(make_backend)
    with pytest.raises(PermissionError):
        backend.call(tool, args)
    assert not hand.control_calls
    arming.write(root, True)

    def fail(**kwargs):
        raise RuntimeError('injected control write/readback failure')

    setattr(hand, tool.removeprefix('wuji_'), fail)
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call(tool, args)
        assert response.get('isError') and front.control_uncertain
        assert hand.stops and arm.holds
    finally:
        front.close()
