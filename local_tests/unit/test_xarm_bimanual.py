from copy import deepcopy
import json
import threading
import time

import numpy as np
import pytest
import yaml

from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.workspace import Setup, build, recipe_for
from services.xarm import arming
from services.xarm.bimanual import BimanualXArmBackend
from services.xarm.bimanual_config import validate_config
from test_revo2 import FakeHand, hand_config
from test_xarm_backend import FakeArm, FakeCameras


class StoppingArm(FakeArm):
    def state(self):
        return {**super().state(), 'state': 4 if self.holds else 0}


@pytest.fixture
def rig(tmp_path):
    extrinsic = tmp_path/'camera.yaml'
    extrinsic.write_text(yaml.safe_dump({'matrix': np.eye(4).tolist()}))
    options = {'arms': {side: {'xarm_ip': f'192.168.100.{14 if side == "left" else 16}',
                       'end_effector': hand_config(side), 'stream_enabled': False, 'settle_s': 0,
                       'cameras': [{'name': 'overview', 'serial': 'camera1', 'mount': 'fixed',
                                    'extrinsics': str(extrinsic)}]}
                       for side in ('left', 'right')}}
    config = Configuration(root=tmp_path/'session', task='test', robot='xarm_bimanual', backend_options=options)
    arms = {side: StoppingArm() for side in ('left', 'right')}
    hands = {side: FakeHand(side) for side in arms}
    backend = BimanualXArmBackend(config, arms=arms, hands=hands, cameras=FakeCameras(), monitor=False)
    yield backend, arms, hands
    backend.close()


def test_mcp_side_selection_and_tool_contract(rig):
    backend, arms, hands = rig
    front = Frontend(Session(backend.config), backend)
    names = {t['name'] for t in backend.tools()}
    assert {'xarm_move_pose', 'xarm_move_bimanual', 'revo_move', 'revo_home'} <= names
    assert 'xarm_gripper' not in names and not any(t.startswith('wuji_') for t in names)
    arming.write(backend.config.root, True)
    response = front.call('revo_move', {'positions': [120, 0, 0, 0, 0, 0]})
    assert response['isError'] and not front.control_uncertain
    assert not any(h.calls for h in hands.values())
    response = front.call('revo_move', {'arm': 'left', 'positions': [120, 0, 0, 0, 0, 0]})
    assert not response.get('isError')
    assert len(hands['left'].calls) == 1 and not hands['right'].calls
    value = backend.call('xarm_move_pose', {'arm': 'right', 'position': [.43, 0, .3]})[0]
    assert value['coordinate_frame'] == 'right_base'
    assert arms['right'].commands == 1 and arms['left'].commands == 0


def test_all_four_devices_stop_on_native_failure(rig):
    backend, arms, hands = rig
    arming.write(backend.config.root, True)
    hands['left'].fail = True
    front = Frontend(Session(backend.config), backend)
    response = front.call('revo_move', {'arm': 'left', 'positions': [120, 0, 0, 0, 0, 0]})
    assert response['isError'] and front.control_uncertain
    assert all(a.holds for a in arms.values()) and all(h.stops for h in hands.values())
    assert not arming.read(backend.config.root)['armed']
    response = front.call('xarm_hold', {})
    assert not response.get('isError') and not front.control_uncertain


def test_both_trajectories_validated_before_either_arm_moves(rig):
    backend, arms, _ = rig
    arming.write(backend.config.root, True)
    with pytest.raises(ValueError):
        backend.call('xarm_move_bimanual', {'left': {'position': [.43, 0, .3]},
                                           'right': {'position': [8, 0, .3]}})
    assert all(a.commands == 0 for a in arms.values())
    result = backend.call('xarm_move_bimanual', {'left': {'position': [.43, 0, .3]},
                                               'right': {'position': [.44, 0, .3]}})[0]
    assert set(result['arms']) == {'left', 'right'} and not result['synchronized']
    assert all(a.commands == 1 for a in arms.values())


def test_unhealthy_other_hand_prevents_dispatch(rig):
    backend, arms, hands = rig
    arming.write(backend.config.root, True)
    hands['right'].ready = False
    with pytest.raises(RuntimeError, match='feedback'):
        backend.call('xarm_move_pose', {'arm': 'left', 'position': [.43, 0, .3]})
    assert all(a.commands == 0 for a in arms.values())


def test_saved_frame_cannot_be_interpreted_in_the_other_base(rig):
    backend, _, _ = rig
    frame = backend.sides['left'].frames_root/'abc123'
    frame.mkdir(parents=True)
    (frame/'meta.json').write_text(json.dumps({'arm': 'left', 'cameras': {}}))
    assert backend.sides['left']._frame_dir('abc123') == frame
    with pytest.raises(ValueError, match='different arm'):
        backend.call('xarm_pixel_to_position', {'arm': 'right', 'frame_id': 'abc123',
                                               'camera': 'overview', 'pixels': [[10, 10]]})


@pytest.mark.parametrize('change', ['ip', 'serial', 'side', 'camera'])
def test_ambiguous_pairing_is_rejected(rig, change):
    backend, _, _ = rig
    options = deepcopy(backend.config.backend_options)
    left, right = options['arms']['left'], options['arms']['right']
    if change == 'ip':
        right['xarm_ip'] = left['xarm_ip']
    elif change == 'serial':
        right['end_effector']['serial'] = left['end_effector']['serial']
    elif change == 'side':
        right['end_effector']['handedness'] = 'left'
    else:
        right['cameras'][0]['mount'] = 'tcp'
    with pytest.raises(ValueError):
        validate_config(options)


def test_generated_workspace_advertises_actual_revo_tools(rig, tmp_path):
    backend, _, _ = rig
    setup = Setup.from_config(backend.config, gemini=False)
    workspace = tmp_path/'generated'
    build(recipe_for(), setup, workspace)
    assert set(setup.robot_facts['tools']) == {t['name'] for t in backend.tools()}
    interface = (workspace/'.agents/skills/robot-interface/SKILL.md').read_text()
    assert 'arm="left"' in interface and 'xarm_move_bimanual' in interface
    for path in workspace.rglob('*.md'):
        assert 'wuji_' not in path.read_text(), path


def test_arming_file_failure_cannot_suppress_physical_stops(rig, monkeypatch):
    backend, arms, hands = rig

    def fail(*args, **kwargs):
        raise OSError('arming filesystem unavailable')
    monkeypatch.setattr(arming, 'write', fail)
    result = backend.call('xarm_hold', {})[0]
    assert not result['control_certain']
    assert all(a.holds for a in arms.values()) and all(h.stops for h in hands.values())
    assert all('disarm_error' in outcome for outcome in result['assembly_stop'].values())


def test_background_hand_fault_stops_both_pairs_without_an_mcp_call(rig):
    backend, arms, hands = rig
    arming.write(backend.config.root, True)
    hands['right'].ready = False
    thread = threading.Thread(target=backend._watch)
    thread.start()
    try:
        deadline = time.monotonic()+3
        while not backend.last_fault and time.monotonic() < deadline:
            time.sleep(.01)
        assert backend.last_fault and backend.control_uncertain
        assert all(a.holds for a in arms.values()) and all(h.stops for h in hands.values())
        assert not arming.read(backend.config.root)['armed']
    finally:
        backend._stop.set()
        thread.join(3)
