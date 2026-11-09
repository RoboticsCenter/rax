"""Read-only G1 status projection for the existing dashboard response shape."""
import json
import time


def snapshot(root):
    try:
        value = json.loads((root / 'operator' / 'g1-state.json').read_text())
        age = time.time() - value['recorded_unix']
        if not 0 <= age <= .5:
            raise ValueError('G1 snapshot is stale')
    except (OSError, ValueError, KeyError) as exc:
        value = {'fault': str(exc), 'body': {}, 'hands': {}}
    body = value.get('body', {})
    # The existing UI only renders one calibrated TCP and a scalar gripper width.
    # Use its unavailable-field path rather than inventing those measurements.
    body_text = value.get('fault') or body.get('fault') or (
        f"G1 FSM {body.get('fsm_id', 'unknown')}; "
        f"feedback age {body.get('feedback_age_s', 'unknown')} s; "
        'arm TCP unavailable in this phase')
    hand_text = '; '.join(f'{side}: {state.get("mode", "unknown")}'
                          for side, state in value.get('hands', {}).items()) or 'Wuji profiles not configured'
    return {'time': time.time(), 'robot': 'g1', 'g1': value,
            'arm': {'ok': False, 'error': body_text},
            'gripper': {'ok': False, 'error': hand_text + '; scalar gripper width is not applicable'}}
