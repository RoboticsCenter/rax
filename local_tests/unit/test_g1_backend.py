import math
import threading
import time

import pytest

from services.controller.config import Configuration
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.g1.backend import G1Backend
from services.g1.config import options
from services.g1.control import MotionError
from services.g1.transport import DryRunTransport, RobotLease
from services.wuji import arming


def wait_for(predicate, timeout=2):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError('condition did not become true')


@pytest.fixture
def robot(tmp_path):
    config = Configuration(root=tmp_path / 'g1', task='walk', robot='g1',
                           observation_profile='rgb-only', backend_options={'motion_commissioned': True})
    driver = DryRunTransport(options(config.backend_options))
    backend = G1Backend(config, transport=driver)
    wait_for(lambda: 'joints' in backend.status()['body'])
    yield backend, driver
    backend.close()


def command(duration=.1):
    return {'vx': .1, 'vy': 0, 'yaw_rate': 0, 'duration_s': duration}


def test_backend_discovery_does_not_import_unitree_sdk(robot):
    backend, _ = robot
    import sys
    assert 'unitree_sdk2py' not in sys.modules
    assert not backend.status()['capabilities']['cartesian_arms']
    assert not any(t['name'] == 'humanoid_move_hands' for t in backend.tools())


def test_requires_arming_and_rejects_invalid_commands_before_dispatch(robot):
    backend, driver = robot
    with pytest.raises(ValueError, match='armed'):
        backend.call('humanoid_move_base', command())
    arming.write(backend.config.root, True)
    for change in ({'vx': math.nan}, {'vx': True}, {'duration_s': 99}, {'foo': 1}, {'vy': .5}):
        with pytest.raises(ValueError):
            backend.call('humanoid_move_base', {**command(), **change})
    assert driver.commands == []


def test_bounded_command_ends_with_zero_and_honest_outcome(robot):
    backend, driver = robot
    arming.write(backend.config.root, True)
    result, _ = backend.call('humanoid_move_base', command())
    assert result['outcome'] == 'command_window_completed'
    assert result['target_reached'] is None
    assert result['stationary_verified'] is False
    assert any(c[0] == .1 for c in driver.commands)
    assert driver.commands[-1][:3] == (0, 0, 0)
    assert all(0 < c[3] <= .25 for c in driver.commands)


def test_disarm_interrupts_while_frontend_dispatch_is_blocked(robot):
    backend, driver = robot
    arming.write(backend.config.root, True)
    frontend = Frontend(Session(backend.config), backend)
    replies = []
    thread = threading.Thread(target=lambda: replies.append(frontend.call('humanoid_move_base', command(2))))
    thread.start()
    wait_for(lambda: any(c[0] for c in driver.commands))
    arming.write(backend.config.root, False)
    thread.join(1)
    assert not thread.is_alive()
    assert replies[0]['isError']
    assert driver.commands[-1][:3] == (0, 0, 0)
    assert backend.worker.snapshot()['fault']
    frontend.s.close()


@pytest.mark.parametrize('field,value', [('feedback_age_s', 1), ('rpy', [.5, 0, 0]),
                                         ('motor_faults', [1] + [0] * 28), ('joints', [math.nan] * 29)])
def test_fault_during_motion_stops_and_latches(robot, field, value):
    backend, driver = robot
    arming.write(backend.config.root, True)
    failures = []
    def move():
        try:
            backend.call('humanoid_move_base', command(2))
        except MotionError as exc:
            failures.append(exc)
    thread = threading.Thread(target=move)
    thread.start()
    wait_for(lambda: any(c[0] for c in driver.commands))
    driver._state[field] = value
    thread.join(1)
    assert failures
    assert driver.commands[-1][:3] == (0, 0, 0)
    assert backend.worker.snapshot()['fault']


def test_fsm_query_failure_cannot_leave_ready_cached_state(robot):
    backend, driver = robot
    arming.write(backend.config.root, True)
    def fail():
        raise RuntimeError('FSM unavailable')
    driver.state = fail
    wait_for(lambda: backend.status()['body'].get('feedback_error'))
    with pytest.raises(MotionError, match='FSM unavailable'):
        backend.call('humanoid_move_base', command())
    assert not driver.commands


def test_stop_error_is_not_reported_as_confirmed(robot):
    backend, driver = robot
    def fail(*args):
        raise RuntimeError('DDS timeout')
    driver.velocity = fail
    result, _ = backend.call('humanoid_stop', {})
    assert result['base']['acknowledged'] is False
    assert backend.status()['fault']


def test_owner_lock_is_exclusive_and_released(tmp_path):
    first = RobotLease(str(tmp_path))
    try:
        with pytest.raises(RuntimeError, match='owner'):
            RobotLease(str(tmp_path))
    finally:
        first.close()
    RobotLease(str(tmp_path)).close()


def test_hardware_commissioning_and_dry_run_hand_separation():
    with pytest.raises(ValueError, match='both Wuji'):
        options({'transport': 'unitree', 'robot_id': 'test', 'motion_commissioned': True})
    with pytest.raises(ValueError, match='physical Wuji'):
        options({'arms': {'left': {}}})


def test_close_is_idempotent(robot):
    backend, _ = robot
    backend.close()
    backend.close()
    assert not backend.worker.thread.is_alive()


def test_workspace_contract_matches_backend_tools(robot, tmp_path):
    from services.workspace import Setup, build, recipe_for
    backend, _ = robot
    setup = Setup.from_config(backend.config, gemini=False)
    assert set(setup.tools) == {'status', 'request_operator', *(t['name'] for t in backend.tools())}
    target = tmp_path / 'workspace'
    build(recipe_for(), setup, target)
    assert (target / '.agents/skills/robot-interface/SKILL.md').is_file()


def test_dashboard_g1_routes_do_not_contact_franka(robot, monkeypatch):
    from services.dashboard.sessions import Sessions
    backend, _ = robot
    backend.config.root.mkdir(parents=True, exist_ok=True)
    (backend.config.root / 'operator-input.json').write_text(__import__('json').dumps(backend.config.to_dict()))
    def forbidden(*args):
        raise AssertionError('G1 dashboard must not contact Franka bridges')
    dashboard = Sessions(http_get=forbidden, http_post=forbidden)
    monkeypatch.setattr(dashboard, 'roots', lambda: [backend.config.root])
    monkeypatch.setenv('RAX_ROBOT', 'g1')
    wait_for(lambda: (backend.config.root / 'operator/g1-state.json').exists())
    assert dashboard.robot()['robot'] == 'g1'
    assert 'G1 disarmed' in dashboard.hold()


def test_wuji_routing_uses_side_specific_profiles_and_blocks_unprepared_peer(tmp_path):
    from wuji_fakes import hand_config
    class Hand:
        def __init__(self, side):
            self.cfg = hand_config(serial=side, handedness=side)
            self.ready = True
            self.moves = []
        def status(self):
            return {'fresh': True, 'enabled': self.ready, 'mode': 'holding', 'target_positions': [0.] * 20}
        def require_ready(self):
            if not self.ready:
                raise RuntimeError('not ready')
            return self.status()
        def move(self, **goal):
            self.moves.append(goal)
            return {'reached': True}
        def hold(self):
            return {'confirmed': True}
        def close(self):
            pass
    hands = {s: Hand(s) for s in ('left', 'right')}
    cfg = Configuration(root=tmp_path/'hands', task='test', robot='g1', observation_profile='rgb-only',
                        backend_options={'transport': 'unitree', 'robot_id': 'fake-only',
                                         'motion_commissioned': True, 'wuji_mounts_commissioned': True,
                                         'allowed_fsm_ids': [500],
                                         'arms': {s: {'end_effector': h.cfg} for s, h in hands.items()}})
    driver = DryRunTransport({})
    with_backend = G1Backend(cfg, transport=driver, hands=hands)
    try:
        wait_for(lambda: 'joints' in with_backend.status()['body'])
        arming.write(cfg.root, True)
        with_backend.call('wuji_pose', {'hand': 'left', 'name': 'test_pose'})
        assert len(hands['left'].moves) == 1 and not hands['right'].moves
        hands['right'].ready = False
        with pytest.raises(MotionError, match='Both Wuji'):
            with_backend.call('wuji_move_joints', {'hand': 'left', 'positions': [0.] * 20})
        assert len(hands['left'].moves) == 1
    finally:
        with_backend.close()
