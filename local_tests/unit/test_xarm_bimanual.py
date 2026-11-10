"""Bimanual routing, paired admission, fault propagation and raw camera contract."""
from copy import deepcopy
import json
from pathlib import Path
import threading
import time

import cv2 as cv
import numpy as np
import pytest

from test_xarm_backend import FakeArm
from wuji_fakes import FakeHandClient, hand_config
from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.workspace import Setup, build, recipe_for
from services.xarm import arming
from services.xarm_bimanual.backend import XArmBimanualBackend
from services.xarm_bimanual.config import SIDES, validate_options
from services.wuji.client import socket_path, state_path, WujiClient


class Arm(FakeArm):
    def __init__(self):
        super().__init__()
        self.stopped = False
        self.error = 0
        self.closed = False

    def state(self):
        return {**super().state(), 'state': 4 if self.stopped else 0, 'error_code': self.error}

    def hold(self):
        super().hold()
        self.stopped = True

    def close(self):
        self.closed = True


class Cameras:
    def __init__(self):
        self.calls = []
        self.closed = False

    def request(self, payload):
        self.calls.append(payload)
        folder = Path(payload['out_dir'])
        folder.mkdir(parents=True)
        result = {}
        for name in payload['cameras']:
            rgb = folder/f'{name}.jpg'
            cv.imwrite(str(rgb), np.full((48, 64, 3), 120, np.uint8))
            result[name] = {'rgb': str(rgb), 'depth': None, 'width': 64, 'height': 48,
                            'K': [[50, 0, 32], [0, 50, 24], [0, 0, 1]], 'timestamp_ns': time.time_ns()}
        return {'cameras': result}

    def close(self):
        self.closed = True


def options():
    return {'arms': {s: {'xarm_ip': f'192.0.2.{i+1}', 'settle_s': 0,
                         'end_effector': hand_config(serial=f'FAKE-{s}', handedness=s,
                            allow_encoder_warning=(s == 'right'))} for i, s in enumerate(SIDES)},
            'cameras': [{'name': name, 'serial': str(i+1), 'mount': mount, **extra}
                        for i, (name, mount, extra) in enumerate([
                            ('side_a', 'fixed', {}), ('side_b', 'fixed', {}),
                            ('wrist_left', 'tcp', {'arm': 'left'}), ('wrist_right', 'tcp', {'arm': 'right'})])],
            'stream_enabled': False, 'motion_archive_enabled': False}


@pytest.fixture
def rig(tmp_path):
    cfg = Configuration(root=tmp_path/'session', task='bimanual', robot='xarm_bimanual',
                        backend_options=options(), observation_profile='rgb-only')
    cfg.root.mkdir()
    arms = {s: Arm() for s in SIDES}
    hands = {s: FakeHandClient(cfg.backend_options['arms'][s]['end_effector']) for s in SIDES}
    cameras = Cameras()
    backend = XArmBimanualBackend(cfg, arms=arms, hands=hands, cameras=cameras)
    yield backend, arms, hands, cameras
    backend.close()


def test_selectors_are_required_and_never_default_to_an_arm(rig):
    backend, arms, hands, _ = rig
    for name, args in [('xarm_move_pose', {'position': [.42, 0, .31]}),
                       ('wuji_move_joints', {'positions': [.1]*20}), ('wuji_status', {}),
                       ('wuji_hold', {}), ('xarm_move_joints', {'preset': 'home'})]:
        with pytest.raises(ValueError, match='missing'):
            backend.call(name, args)
    with pytest.raises(ValueError, match='one of'):
        backend.call('wuji_status', {'hand': 'both'})
    assert all(a.commands == 0 for a in arms.values())
    assert all(not h.moves for h in hands.values())


def test_each_side_routes_to_its_existing_arm_and_hand_controller(rig):
    backend, arms, hands, _ = rig
    arming.write(backend.config.root, True)
    value, _ = backend.call('xarm_move_pose', {'arm': 'right', 'position': [0, 0, .01], 'relative': True})
    assert value['arm'] == 'right' and value['base_frame'] == 'right_base'
    assert arms['left'].commands == 0 and arms['right'].commands == 1
    backend.call('wuji_move_joints', {'hand': 'left', 'positions': [.1]*20})
    assert hands['left'].moves == [[.1]*20] and not hands['right'].moves
    assert backend.arms['right'].hand_config['allow_encoder_warning'] is True
    assert backend.arms['left'].hand_config['allow_encoder_warning'] is False


def test_short_scurve_duration_does_not_stop_either_assembly(rig):
    backend, arms, hands, _ = rig
    backend.arms['left'].hand_config['motion_mode'] = 'scurve'
    arming.write(backend.config.root, True)
    value, _ = backend.call('wuji_move_joints', {
        'hand': 'left', 'positions': [.2]*20, 'duration_s': .1})
    assert value['reached'] and hands['left'].moves == [[.2]*20]
    assert arming.read(backend.config.root)['armed']
    assert all(not hand.stops for hand in hands.values())
    assert all(not arm.holds for arm in arms.values())


def test_scurve_infeasible_path_is_rejected_before_any_assembly_moves(rig):
    backend, arms, hands, _ = rig
    backend.arms['left'].hand_config.update(motion_mode='scurve', max_duration_s=.1)
    arming.write(backend.config.root, True)
    with pytest.raises(ValueError, match='duration limit'):
        backend.call('wuji_move_joints', {'hand': 'left', 'positions': [.2]*20})
    assert arming.read(backend.config.root)['armed']
    assert all(not hand.stops and not hand.moves for hand in hands.values())
    assert all(not arm.holds and not arm.commands for arm in arms.values())


@pytest.mark.parametrize('tool,bad', [
    ('xarm_move_both', {'left': {'position': [.42, 0, .31]}, 'right': {'position': [2, 0, .31]}}),
    ('xarm_move_joints_both', {'left': {'preset': 'home'}, 'right': {'positions': [0]*6, 'preset': 'home'}}),
    ('wuji_move_both', {'left': {'positions': [.1]*20}, 'right': {'positions': [2.]*20}}),
])
def test_both_targets_are_validated_before_either_moves(rig, tool, bad):
    backend, arms, hands, _ = rig
    arming.write(backend.config.root, True)
    front = Frontend(Session(backend.config), backend)
    result = front.call(tool, bad)
    assert result['isError']
    error = json.loads(result['content'][0]['text'])['error']
    assert error['no_action_executed']
    assert all(a.commands == 0 for a in arms.values())
    assert all(not h.moves for h in hands.values())


def test_paired_moves_overlap_and_return_side_results(rig):
    backend, _, hands, _ = rig
    entered = {s: threading.Event() for s in SIDES}
    for side in SIDES:
        original = hands[side].move
        def move(*args, side=side, original=original, **kwargs):
            entered[side].set()
            assert entered['right' if side == 'left' else 'left'].wait(1), 'moves were serialized'
            return original(*args, **kwargs)
        hands[side].move = move
    arming.write(backend.config.root, True)
    result, _ = backend.call('wuji_move_both', {'left': {'positions': [.1]*20},
                                              'right': {'positions': [-.1]*20}})
    assert all(result['results'][s]['reached'] for s in SIDES)
    assert result['synchronized_trajectory'] is False
    assert hands['left'].target == [.1]*20 and hands['right'].target == [-.1]*20


def test_failed_paired_move_stops_and_disarms_all_four(rig):
    backend, arms, hands, _ = rig
    hands['left'].fail_move = True
    arming.write(backend.config.root, True)
    with pytest.raises(Exception) as failure:
        backend.call('wuji_move_both', {'left': {'positions': [.1]*20}, 'right': {'positions': [.1]*20}})
    assert 'injected hand transport loss' in str(failure.value.robot_state['failed_sides'])
    assert not arming.read(backend.config.root)['armed']
    assert all(a.holds for a in arms.values()) and all(h.stops for h in hands.values())


def test_hold_verifies_all_four_and_one_failed_stop_keeps_uncertainty(rig):
    backend, arms, hands, _ = rig
    hands['right'].fail_stop = True
    result, _ = backend.call('xarm_hold', {})
    assert set(result['assembly_stop']) == {'left_arm', 'left_hand', 'right_arm', 'right_hand'}
    assert result['assembly_stop']['left_hand']['confirmed']
    assert not result['held'] and backend.control_uncertain
    hands['right'].fail_stop = False
    result, _ = backend.call('xarm_hold', {})
    assert result['held'] and not backend.control_uncertain


def test_peer_controller_fault_blocks_motion_and_stops_the_entire_rig(rig):
    backend, arms, hands, _ = rig
    arms['right'].error = 23
    arming.write(backend.config.root, True)
    with pytest.raises(Exception):
        backend.call('wuji_move_joints', {'hand': 'left', 'positions': [.1]*20})
    assert not hands['left'].moves
    assert all(a.holds for a in arms.values()) and all(h.stops for h in hands.values())
    assert not arming.read(backend.config.root)['armed']


def test_controller_watch_runs_without_recording(rig):
    backend, arms, hands, _ = rig
    assert backend.stream is None
    arming.write(backend.config.root, True)
    arms['right'].error = 23
    deadline = time.monotonic()+2
    while arming.read(backend.config.root)['armed'] and time.monotonic() < deadline:
        time.sleep(.01)
    assert not arming.read(backend.config.root)['armed']
    while backend._last_control_fault is None and time.monotonic() < deadline:
        time.sleep(.01)
    assert backend._last_control_fault is not None or arms['left'].holds


def test_recording_failure_does_not_stop_controller_supervision(rig, tmp_path):
    backend, arms, _, _ = rig
    backend._stop.set()
    backend._telemetry.join(2)
    backend._stop.clear()
    backend.stream = tmp_path/'blocked-recording'
    backend.stream.write_text('not a directory')
    backend._telemetry = threading.Thread(target=backend._log_robot, daemon=True)
    backend._telemetry.start()
    deadline = time.monotonic()+2
    while backend._robot_log_error is None and time.monotonic() < deadline:
        time.sleep(.01)
    assert backend._robot_log_error is not None
    arming.write(backend.config.root, True)
    arms['right'].error = 23
    while arming.read(backend.config.root)['armed'] and time.monotonic() < deadline:
        time.sleep(.01)
    assert not arming.read(backend.config.root)['armed']
    assert backend._telemetry.is_alive()


def test_stop_reaches_all_devices_even_if_disarm_file_cannot_be_written(rig, monkeypatch):
    backend, arms, hands, _ = rig
    def fail(*args, **kwargs):raise OSError('disk full')
    monkeypatch.setattr(arming, 'write', fail)
    result = backend._stop_assembly()
    assert all(a.holds for a in arms.values()) and all(h.stops for h in hands.values())
    assert all(v['disarm_error'] == 'disk full' and not v['confirmed'] for v in result.values())


def test_different_side_speed_limits_are_advertised_and_enforced(rig):
    backend, _, _, _ = rig
    backend.arms['left'].options['cartesian_speed_mps'] = .02
    backend.arms['right'].options['cartesian_speed_mps'] = .08
    arming.write(backend.config.root, True)
    payload = {'position': [0, 0, .01], 'relative': True, 'speed_mps': .06}
    backend.validate('xarm_move_pose', {'arm': 'right', **payload})
    with pytest.raises(ValueError):
        backend.validate('xarm_move_pose', {'arm': 'left', **payload})


def test_paired_arm_move_uses_both_existing_planners(rig):
    backend, arms, _, _ = rig
    arming.write(backend.config.root, True)
    result, _ = backend.call('xarm_move_both', {
        'left': {'position': [0, 0, .01], 'relative': True},
        'right': {'position': [0, 0, .02], 'relative': True}})
    assert all(a.commands == 1 for a in arms.values())
    assert all(result['results'][side]['reached'] for side in SIDES)
    assert result['results']['left']['measured_position'] != result['results']['right']['measured_position']


def test_raw_observation_has_four_identities_and_no_invented_transforms(rig):
    backend, _, _, cameras = rig
    value, images = backend.call('xarm_observe', {})
    assert len(images) == 4 and len(cameras.calls) == 1
    meta = json.loads((backend.workspace/value['directory']/'meta.json').read_text())
    assert set(meta['arms']) == set(SIDES)
    assert meta['cameras']['wrist_right']['arm'] == 'right'
    assert all('T_base_cam' not in c for c in meta['cameras'].values())
    assert value['calibration_status'] == 'pending'
    with pytest.raises(ValueError, match='unknown'):
        backend.call('xarm_pixel_to_position', {})
    with pytest.raises(ValueError, match='unknown'):
        backend.call('xarm_observe', {'project': [[0, 0, 0]]})


def test_workspace_contract_matches_actual_tools_for_both_observation_profiles(rig, tmp_path):
    backend, _, _, _ = rig
    for profile in ('rgb-only', 'rgbd'):
        setup = Setup('xarm_bimanual', observation_profile=profile, backend_options=backend.config.backend_options)
        workspace = tmp_path/profile
        build(recipe_for(), setup, workspace)
        names = set(json.loads((workspace/'MCP_CONTRACT.json').read_text())['tools'])
        assert names == {'status', 'request_operator', 'gemini_generate', *(t['name'] for t in backend.tools())}
        assert not setup.camera_geometry
        for p in workspace.rglob('*.md'):
            text = p.read_text()
            assert '{{' not in text and '{%' not in text, p
        text = (workspace/'.agents/skills/robot-interface/SKILL.md').read_text()
        assert 'arm="left"' in text and 'hand="left"' in text
        assert (workspace/'.agents/skills/object-localization/SKILL.md').exists()


@pytest.mark.parametrize('change', ['same_ip', 'same_hand', 'wrong_hand', 'same_camera', 'wrong_wrist', 'old_calibration'])
def test_ambiguous_identity_and_stale_calibration_are_rejected(change):
    cfg = options()
    if change == 'same_ip':cfg['arms']['right']['xarm_ip'] = cfg['arms']['left']['xarm_ip']
    if change == 'same_hand':cfg['arms']['right']['end_effector']['serial'] = cfg['arms']['left']['end_effector']['serial']
    if change == 'wrong_hand':cfg['arms']['left']['end_effector']['handedness'] = 'right'
    if change == 'same_camera':cfg['cameras'][1]['serial'] = cfg['cameras'][0]['serial']
    if change == 'wrong_wrist':cfg['cameras'][-1]['arm'] = 'left'
    if change == 'old_calibration':cfg['cameras'][0]['extrinsics'] = 'old.yaml'
    with pytest.raises(ValueError):validate_options(cfg)


def test_hand_ipc_and_snapshots_are_namespaced_without_splitting_arming(tmp_path, monkeypatch):
    assert len({socket_path(tmp_path), *(socket_path(tmp_path, s) for s in SIDES)}) == 3
    assert state_path(tmp_path, 'left') != state_path(tmp_path, 'right')
    calls = []
    monkeypatch.setattr('services.wuji.client.operation', lambda root, op, args=None, **kw:
                        calls.append((root, op, kw.get('side'))))
    for side in SIDES:
        client = WujiClient.__new__(WujiClient)
        client.root, client.side, client.cfg = tmp_path, side, hand_config()
        client.move([.1]*20)
        client.hold()
        client.stop()
    assert calls == [(tmp_path, op, side) for side in SIDES for op in ('move', 'hold', 'emergency')]


def test_three_camera_configuration():
    cfg = options()
    cfg['cameras'] = cfg['cameras'][1:]
    cfg['cameras'][0]['name'] = 'front'
    validate_options(cfg)
    cfg['cameras'] = cfg['cameras'][1:]
    with pytest.raises(ValueError, match='fixed'):
        validate_options(cfg)
