"""Dashboard HTTP paths select the bimanual operator instead of a default arm."""
import json

from test_dashboard import dashboard, robot, configure, call, PROJECT
from services.xarm import arming
from services.xarm_bimanual import operator


def test_bimanual_dashboard_uses_matching_session_for_status_and_stop(dashboard, monkeypatch):
    base, _, sessions = dashboard
    monkeypatch.setenv('RAX_BACKEND_OPTIONS', str(PROJECT/'examples/xarm-wuji-bimanual.json'))
    created = configure(base, task='bimanual')
    root = sessions.root(created['id'])
    saved = json.loads((root/'operator-input.json').read_text())
    assert saved['robot'] == 'xarm_bimanual'
    assert sessions.summary(root)['prepare_on_reply']
    calls = []
    def status(selected, options):
        calls.append(('status', selected, options['arms']))
        return {'bimanual': True, 'arms': {'left': {}, 'right': {}}}
    def stop(selected, options):
        calls.append(('stop', selected, options['arms']))
        arming.write(selected, False)
        return {f'{side}_{device}': {'requested': True} for side in ('left', 'right')
                for device in ('arm', 'hand')}
    monkeypatch.setattr(operator, 'status', status)
    monkeypatch.setattr(operator, 'stop', stop)
    code, value = call(base, '/api/robot')
    assert code == 200 and value['bimanual'] and set(value['arms']) == {'left', 'right'}
    arming.write(root, True)
    code, value = call(base, '/api/disarm-all', {})
    assert code == 200 and not arming.read(root)['armed']
    assert 'left_arm' in value['hold'] and 'right_hand' in value['hold']
    assert calls == [(op, root, saved['backend_options']['arms']) for op in ('status', 'stop')]
