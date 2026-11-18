"""Smooth-layer MCP tools, trace tools and the ctx helper, with fake arm/hand clients only."""
import json
import os
import socket
import threading

import pytest
from test_wuji_backend import make_wuji
from test_xarm_backend import make_backend as make_backend  # noqa: F401 -- pytest fixture
from test_xarm_bimanual import rig as rig  # noqa: F401 -- pytest fixture

from auto_research_agent.api.hand import Hand, Rate, stream_both
from auto_research_agent.api.runtime import Context
from services.controller.frontend import Frontend
from services.controller.session import Session
from services.wuji.config import LEGACY_TOOL_NAMES, STREAM_TOOL_NAMES, TRACE_TOOL_NAMES
from services.wuji.trace import load_trial
from services.xarm import arming
from services.xarm.backend import _validate_arguments

I = 4  # index_S1


def body(response):
    return json.loads(response['content'][0]['text'])


def test_tool_listing_keeps_legacy_tools_and_adds_smooth_and_trace_tools(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    tools = {t['name']: t for t in backend.tools()}
    assert set(LEGACY_TOOL_NAMES) | set(STREAM_TOOL_NAMES) | set(TRACE_TOOL_NAMES) <= set(tools)
    assert {'wuji_stream', 'wuji_trajectory', 'wuji_set_profile'} <= backend.mutating
    assert not {'wuji_wait', 'wuji_stream_config', 'trace_start', 'trace_stop'} & backend.mutating
    for name, args in [('wuji_stream', {'fingers': {'index': [.1, 0, .2, .1]}, 'duration_s': .5}),
                       ('wuji_stream_config', {'max_velocity_rad_s': [.3]*20, 'interpolation_s': None}),
                       ('wuji_set_profile', {'name': 'soft', 'current_limit_a': .4}),
                       ('trace_start', {'directory': 'x', 'sides': ['right'], 'params': {'a': [1, {'b': 2}]}}),
                       ('trace_stop', {'success': None})]:
        _validate_arguments(args, tools[name]['inputSchema'])
    with pytest.raises(ValueError):
        _validate_arguments({'joints': {'index_S9': 1.}}, tools['wuji_stream']['inputSchema'])


def test_stream_rejections_are_validation_errors_without_stops(make_backend):
    backend, arm, hand, root = make_wuji(make_backend)
    front = Frontend(Session(backend.config), backend)
    try:
        response = front.call('wuji_stream', {'joints': {'index_S1': .2}})
        assert response.get('isError') and body(response)['error']['no_action_executed']  # Disarmed.
        arming.write(root, True)
        response = front.call('wuji_stream', {'joints': {'index_S1': 5.}})
        assert response.get('isError') and body(response)['error']['no_action_executed']
        assert not front.control_uncertain and not hand.stops and arm.holds == 0
        response = front.call('wuji_stream', {'joints': {'index_S1': .2}})
        assert not response.get('isError') and body(response)['accepted']
        assert hand.target[I] == .2 and hand.target[0] == 0. and arm.commands == 0
        assert backend.last_motion is None or backend.last_motion['tool'] != 'wuji_stream'
        hand.fail_move = True
        response = front.call('wuji_stream', {'joints': {'index_S1': .3}})
        assert response.get('isError') and front.control_uncertain and hand.stops and arm.holds  # Transport loss.
    finally:
        front.close()


def test_trajectory_record_and_trace_tools_write_trials_in_the_workspace(make_backend, tmp_path):
    backend, arm, hand, root = make_wuji(make_backend)
    arming.write(root, True)
    value, _ = backend.call('wuji_trajectory', {
        'waypoints': [{'t': .5, 'joints': {'index_S1': .2}}, {'t': 1., 'joints': {'index_S1': 0.}}],
        'record': {'controller': 'M4t_trajectory', 'skill': 'probe', 'params': {'speed': .3}}})
    assert value['trace'].startswith('runtime/traces/')
    trial = load_trial(backend.workspace/value['trace'])
    assert trial['meta']['controller'] == 'M4t_trajectory' and trial['meta']['sides'] == ['right']
    assert trial['meta']['current_limit_a'] == .5 and 'right' in trial['hands']
    outside = tmp_path/'elsewhere'/'trial-7'
    started, _ = backend.call('trace_start', {'directory': str(outside), 'label': 'trial-7', 'controller': 'M1'})
    assert started['requested_directory'] == str(outside) and started['path'].startswith('runtime/traces/')
    with pytest.raises(RuntimeError, match='still recording'):
        backend.call('trace_start', {})
    stopped, _ = backend.call('trace_stop', {'success': True})
    assert stopped['directory'] == started['directory'] and not outside.exists()
    meta = json.loads((backend.workspace/stopped['path']/'meta.json').read_text())
    assert meta['success'] is True and meta['controller'] == 'M1'
    inside = backend.workspace/'output'/'trial-8'
    inside.mkdir(parents=True)
    started, _ = backend.call('trace_start', {'directory': str(inside)})
    assert started['directory'] == str(inside.resolve()) and 'requested_directory' not in started
    backend.call('trace_stop', {})
    assert (inside/'meta.json').is_file() and (inside/'hand_right.npz').is_file()
    escape = backend.workspace/'output'/'link'
    os.symlink(tmp_path, escape)  # A planted symlink cannot redirect host writes outside.
    started, _ = backend.call('trace_start', {'directory': str(escape/'x')})
    assert 'requested_directory' in started
    backend.call('trace_stop', {})
    with pytest.raises(RuntimeError, match='no trace'):
        backend.call('trace_stop', {})


def test_ctx_helper_through_the_real_mcp_socket(make_backend, tmp_path):
    backend, arm, hand, root = make_wuji(make_backend)
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
                h = ctx.hand()
                assert isinstance(h, Hand)
                assert h.configure(friction=True, filter_tau_s=.02)['applies']
                assert h.profile('soft', current_limit_a=.4)['outcome'] == 'profile_set'
                with h.recording(controller='M2_stream_friction_ff', skill='unit', params={'k': 1}) as trial:
                    rate = Rate(200)
                    for k in range(5):
                        h.stream(index=[.02*k, 0, 0, 0])
                        rate.sleep()
                    h.stream(joints={'thumb_S1': .1})
                    h.trajectory([(.3, {'index_S2': .1}), (.6, [0.]*20)])
                    trial.success = h.wait()['settled']
                assert trial.path.startswith('runtime/traces/') and trial.result['recording'] is False
                meta = json.loads((backend.workspace/trial.path/'meta.json').read_text())
                assert meta['success'] is True and meta['skill'] == 'unit'
                kinds = [c[0] for c in hand.control_calls]
                assert kinds.count('stream') == 6 and 'trajectory' in kinds and 'set_profile' in kinds
                with pytest.raises(RuntimeError):
                    with h.recording(controller='x'):
                        raise RuntimeError('skill failed')
                with pytest.raises(ValueError):
                    h.stream(index=[0.]*4, joints={'thumb_S1': 0.})
                # Function API used by skill libraries (dexlib.io.bind_streaming).
                from auto_research_agent.api import hand as api
                assert api.available(ctx) and not api.available(ctx, {'wuji_move_joints'})
                assert api.stream(ctx, 'right', [.05]*20)['accepted'] and hand.target == [.05]*20
                reply = api.follow(ctx, 'right', [.3, .6], [[.1]*20, [0.]*20])
                assert reply['waited'] and hand.target == [0.]*20
                assert api.profile(ctx, 'right', 'default', current_limit_a=.5)['outcome'] == 'profile_set'
                inside = backend.workspace/'output'/'lib-trial'
                with api.record(ctx, ['right'], inside, controller='S1', skill='lib') as started:
                    api.stream(ctx, 'right', [0.]*20)
                assert started['directory'] == str(inside.resolve()) and (inside/'meta.json').is_file()
                assert started['stopped']['recording'] is False
        finally:
            stop.set()
            front.close()


def test_bimanual_selectors_paired_stream_and_two_hand_trace(rig, tmp_path):
    backend, arms, hands, _ = rig
    tools = {t['name']: t for t in backend.tools()}
    for name in STREAM_TOOL_NAMES:
        assert 'hand' in tools[name]['inputSchema']['required']
    assert {'wuji_stream_both', 'trace_start', 'trace_stop'} <= set(tools)
    arming.write(backend.config.root, True)
    with pytest.raises(ValueError, match='missing'):
        backend.call('wuji_stream', {'joints': {'index_S1': .1}})
    value, _ = backend.call('wuji_stream', {'hand': 'left', 'joints': {'index_S1': .1}})
    assert value['hand'] == 'left' and hands['left'].target[I] == .1 and hands['right'].target[I] == 0.
    with pytest.raises(ValueError):  # Both are validated before either is accepted.
        backend.call('wuji_stream_both', {'left': {'joints': {'index_S1': .2}}, 'right': {'joints': {'index_S1': 9.}}})
    assert hands['left'].target[I] == .1
    value, _ = backend.call('wuji_stream_both', {'left': {'joints': {'index_S1': .2}},
                                                 'right': {'fingers': {'ring': [.1, 0, .1, 0]}}})
    assert set(value['results']) == {'left', 'right'} and hands['right'].target[12] == .1
    assert not any(a.holds for a in arms.values())
    started, _ = backend.call('trace_start', {'controller': 'M1_stream_1khz', 'skill': 'bimanual-probe'})
    assert started['sides'] == ['left', 'right']
    stopped, _ = backend.call('trace_stop', {'success': None})
    trial = load_trial(backend.workspace/stopped['path'])
    assert trial['meta']['sides'] == ['left', 'right'] and set(trial['hands']) == {'left', 'right'}
    started, _ = backend.call('trace_start', {'sides': ['right']})
    backend.call('trace_stop', {})
    assert load_trial(started['directory'])['meta']['sides'] == ['right']
    with pytest.raises(ValueError):
        backend.call('trace_start', {'sides': ['both']})
    js = stream_both  # Helper accepts plain 20-vectors or payload dicts.
    assert callable(js)
