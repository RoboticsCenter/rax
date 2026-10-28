"""Exercise live qualification abort/cleanup without any device connection."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import pytest


@pytest.fixture
def rig(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location(
        'qualify_wuji_effort', Path(__file__).resolve().parents[2] / 'scripts/qualify_wuji_effort.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    baseline = {key: [value] * 20 for key, value in module.PLAN['baseline'].items()}
    hand = dict(fresh=True, fault=None, faults=[], enabled=False, diagnostics={'states': [1]*20},
                command_effort_a=[0.]*20, profile_revision='right-hand2-scurve-20261028-v13',
                profile_control_parameters=baseline, control_parameters=deepcopy(baseline),
                positions=[.1]*20, velocity_rad_s=[0.]*20)
    events = []
    arm = dict(controller_state=4, error_code=0, motion={'armed': False}, tcp_position=[.3, 0, .3])

    def arming(root, enabled, **kw):
        events.append(('arming', enabled))
        if not enabled:
            hand.update(enabled=False, diagnostics={'states': [1]*20}, command_effort_a=[0.]*20)

    def operation(root, name, **kw):
        events.append(('operator', name))
        hand.update(enabled=name == 'prepare', diagnostics={'states': [2 if name == 'prepare' else 1]*20})
        if name == 'emergency':
            hand['command_effort_a'] = [0.]*20
        return {'ok': True}

    class Context:
        failure = None
        def __init__(self, *a): pass
        def __enter__(self): return self
        def __exit__(self, *a): pass
        def value(self, name, **kw):
            if name == 'xarm_status': return deepcopy(arm)
            if name == 'wuji_status': return deepcopy(hand)
            events.append(('mcp', name))
            if self.failure == name:
                raise RuntimeError('injected MCP failure')
            if name == 'wuji_set_effort':
                hand['command_effort_a'] = kw['effort_a']
            elif name == 'wuji_set_impedance':
                hand['control_parameters'] = deepcopy(kw)
            return {'ok': True, 'readback_verified': True}

    monkeypatch.setattr(module, 'Context', Context)
    monkeypatch.setattr(module, 'operation', operation)
    monkeypatch.setattr(module, 'request', lambda *a, **kw: deepcopy(hand))
    monkeypatch.setattr(module.arming, 'write', arming)
    original_sleep = module.time.sleep
    monkeypatch.setattr(module.time, 'sleep', lambda duration: original_sleep(.02))
    return module, tmp_path, hand, arm, events, Context


def test_success_restores_parameters_and_finishes_disabled(rig):
    module, root, hand, arm, events, context = rig
    assert module.qualify(root, root/'fake.sock')
    assert hand['control_parameters'] == hand['profile_control_parameters']
    assert not hand['enabled'] and hand['command_effort_a'] == [0.]*20
    assert [name for kind, name in events if kind == 'mcp'] == ['wuji_set_effort']*5 + ['wuji_set_impedance']*2


def test_mcp_failure_stops_without_retry_or_rearming(rig):
    module, root, hand, arm, events, context = rig
    context.failure = 'wuji_set_effort'
    assert not module.qualify(root, root/'fake.sock')
    assert [name for kind, name in events if kind == 'mcp'] == ['wuji_set_effort']
    assert events.count(('arming', True)) == 1
    assert events[-2:] == [('arming', False), ('operator', 'emergency')]
    assert not hand['enabled']
    assert 'injected MCP failure' in json.loads((root/'qualification-report.json').read_text())['error']


def test_unstopped_arm_rejects_before_hand_enable(rig):
    module, root, hand, arm, events, context = rig
    arm['controller_state'] = 0
    assert not module.qualify(root, root/'fake.sock')
    assert ('arming', True) not in events and ('operator', 'prepare') not in events
    assert not any(kind == 'mcp' for kind, name in events)


def test_excursion_monitor_aborts_before_impedance(rig, monkeypatch):
    module, root, hand, arm, events, context = rig
    original_value = context.value
    def value(self, name, **args):
        result = original_value(self, name, **args)
        if name == 'wuji_set_effort' and any(args['effort_a']):
            hand['positions'][10] += .04
        return result
    monkeypatch.setattr(context, 'value', value)
    assert not module.qualify(root, root/'fake.sock')
    report = json.loads((root/'qualification-report.json').read_text())
    assert 'excursion' in str(report['guard_errors'])
    assert ('mcp', 'wuji_set_impedance') not in events
    assert not hand['enabled'] and events.count(('arming', True)) == 1
