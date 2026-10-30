"""Operator-requested recovery of retained thermal or cleared stall faults."""
import json
import math
import time
from pathlib import Path

from services.json_io import write_private_json
from services.xarm import arming

from .client import operation, request, socket_path
from .config import DEFAULTS, JOINT_NAMES


class PreparationBlocked(RuntimeError):
    pass


def preparation_action(state, cfg):
    """Preflight without enabling, clearing faults, or issuing a robot command."""
    if state.get('fresh') is not True:
        raise PreparationBlocked('Hand feedback is stale or unavailable. Check hand connectivity before preparing.')
    faults = state.get('faults') or []
    if not faults:
        return 'prepare'
    kinds = {_recoverable_fault(f) for f in faults}
    if None in kinds:
        details = '; '.join(f"{f.get('joint', 'hand')}: {f.get('name') or f.get('reason') or f.get('code')}"
                            if isinstance(f, dict) else str(f) for f in faults)
        raise PreparationBlocked(f'Hand fault requires inspection; it will not be cleared automatically: {details}')
    kind = 'stall' if 'stall' in kinds else 'thermal'
    _require_disabled_feedback(state, cfg, kind)
    return 'reconnect-stall' if kind == 'stall' else 'reconnect'


def _recoverable_fault(fault):
    if not isinstance(fault, dict) or fault.get('joint') not in JOINT_NAMES:
        return None
    if fault.get('reason') == 'temperature limit exceeded' and fault.get('code') is None:
        return 'thermal'
    if (type(fault.get('code')) is int and fault['code'] == 7 and fault.get('name') == 'Stall'
            and fault.get('severity') == 'Warning' and fault.get('clear_policy') == 'AutoClear'):
        return 'stall'
    return None


def _require_disabled_feedback(state, cfg, kind):
    """Require the device itself to be healthy before discarding retained evidence."""
    label = f'{kind.capitalize()} recovery'
    if state.get('fresh') is not True:
        raise PreparationBlocked(f'{label} requires fresh hand feedback.')
    if (state.get('enabled') is not False or state.get('owns_control') is not False
            or state.get('mode') not in ('disabled', 'faulted')):
        raise PreparationBlocked(f'{label} requires a confirmed disabled hand. Stop it before preparing.')
    diag = state.get('diagnostics', {})
    temperatures = diag.get('mcu_temp_c_fb')
    velocities = state.get('velocity_rad_s')
    names = diag.get('state_names', [])
    codes = diag.get('error_code_current', [])
    if (not all(isinstance(v, list) and len(v) == 20
                and all(type(x) in (int, float) and math.isfinite(x) for x in v)
                for v in (temperatures, velocities))
            or len(names) != 20 or not all(str(n).lower() in ('ready', 'stopped', 'disabled') for n in names)
            or len(codes) != 20 or any(type(c) is not int or c != 0 for c in codes)):
        raise PreparationBlocked(f'{label} needs complete disabled feedback and zero native error codes. '
                                 'A current device fault must clear before recovery.')
    if max(map(abs, velocities)) > cfg.get('stopped_velocity_rad_s', DEFAULTS['stopped_velocity_rad_s']):
        raise PreparationBlocked(f'Hand is still moving; {kind} recovery requires measured standstill.')
    limit = cfg.get('max_temperature_c', DEFAULTS['max_temperature_c'])
    active_limit = state.get('diagnostics_policy', {}).get('max_temperature_c', limit)
    if not all(type(x) in (int, float) and math.isfinite(x) and 20 <= x <= 90
               for x in (limit, active_limit)):
        raise PreparationBlocked(f'{label} requires a valid configured temperature limit.')
    limit = min(limit, active_limit)
    hottest = max(range(20), key=lambda i: temperatures[i])
    if temperatures[hottest] >= limit:
        raise PreparationBlocked(
            f'Hand cooling: {JOINT_NAMES[hottest]} is {temperatures[hottest]:.1f} C. '
            f'The earlier fault is retained. Wait until all joints are below {limit:g} C, '
            f'then click Prepare again; it will reconnect once before preparing. The {limit:g} C limit is unchanged.')


def recover_cooled_thermal(root):
    return _recover_retained_fault(root, 'thermal', 'reconnect')


def recover_cleared_stall(root):
    """Clear the software latch only after the device's AutoClear stall has cleared."""
    return _recover_retained_fault(root, 'stall', 'reconnect-stall')


def _recover_retained_fault(root, kind, expected_action):
    """One explicit operator reconnect, never an automatic motion retry."""
    root = Path(root)
    cfg = json.loads((root/'operator-input.json').read_text())['backend_options']['end_effector']
    path = socket_path(root)

    def ready_sample():
        if arming.read(root)['armed']:
            raise PreparationBlocked(f'Disarm the session before {kind} reconnection.')
        state = request(path, 'status')
        if preparation_action(state, cfg) != expected_action:
            raise PreparationBlocked(f'No recoverable {kind} fault is pending; reconnection was not performed.')
        return state

    before = ready_sample()
    deadline = time.monotonic()+2.
    while time.monotonic() < deadline:
        time.sleep(.1)
        ready_sample()
    evidence = root/'operator'/f'{kind}-recovery-{time.time_ns()}.json'
    record = {'before': before, 'source': 'explicit_operator_prepare', 'time': time.time()}
    write_private_json(evidence, record)
    try:
        ready_sample()  # Check arming/health again immediately before the one reconnect.
        result = operation(root, 'connect')
        deadline = time.monotonic()+3.
        while True:
            after = request(path, 'status')
            if after.get('fresh'):
                break
            if time.monotonic() >= deadline:
                raise PreparationBlocked('Fresh diagnostics did not return after reconnection; hand remains disabled.')
            time.sleep(.05)
        record.update(after=after, reconnect=result)
        if (after.get('enabled') or after.get('mode') != 'disabled' or after.get('fault')
                or after.get('faults') or arming.read(root)['armed']):
            raise PreparationBlocked(f'{kind.capitalize()} recovery did not establish a fault-free, disarmed state. '
                                     'Do not retry motion.')
        _require_disabled_feedback(after, cfg, kind)
        return {'outcome': f'{kind}_reconnected', 'enabled': False,
                'max_temperature_c': max(after['diagnostics']['mcu_temp_c_fb']), 'evidence': str(evidence)}
    except Exception as exc:
        record['error'] = str(exc)
        raise
    finally:
        write_private_json(evidence, record)
