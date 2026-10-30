"""Controller health checks, independent of motion progress and deadlines."""
import numbers

import numpy as np


class ControllerFault(RuntimeError):
    operator_required = True
    control_certain = False

    def __init__(self, reason, *, state=None, category='controller'):
        super().__init__(reason)
        self.robot_state = dict(state or {})
        for name in ('joints', 'tcp_position', 'tcp_quaternion', 'tcp_rpy_rad'):
            if name in self.robot_state:
                try:
                    value = np.asarray(self.robot_state[name], dtype=float)
                    self.robot_state[name] = value.tolist() if np.isfinite(value).all() else None
                except (TypeError, ValueError):
                    self.robot_state[name] = None
        self.fault_category = category


def error_title(code):
    try:
        from xarm.core.config.x_code import ControllerErrorCodeMap
        return ControllerErrorCodeMap.get(code, {}).get('en', {}).get('title', 'Unknown controller error')
    except ImportError:
        return 'Controller error'


def check_controller(state):
    """State 0/1/2 in position mode is usable; warnings alone are not faults."""
    for name in ('error_code', 'warn_code', 'state', 'mode'):
        if isinstance(state.get(name), bool) or not isinstance(state.get(name), numbers.Integral):
            raise ControllerFault(f'xArm {name} feedback is unavailable', state=state, category='feedback')
    code = state['error_code']
    if code != 0:
        raise ControllerFault(f'xArm controller error {code}: {error_title(code)}', state=state)
    if state.get('connected') is False:
        raise ControllerFault('xArm controller disconnected', state=state, category='feedback')
    if state['state'] not in (0, 1, 2):
        raise ControllerFault(f'xArm controller is not running: state {state["state"]}',
                              state=state, category='state')
    if state['mode'] != 0:
        raise ControllerFault(f'xArm controller left position mode: mode {state["mode"]}',
                              state=state, category='mode')
    for name, size in (('joints', 6), ('tcp_position', 3), ('tcp_quaternion', 4)):
        try:
            value = np.asarray(state.get(name), dtype=float)
            valid = value.shape == (size,) and np.isfinite(value).all()
            if name == 'tcp_quaternion':
                valid = valid and np.linalg.norm(value) > .5
        except (TypeError, ValueError):
            valid = False
        if not valid:
            raise ControllerFault(f'xArm {name} feedback is invalid', state=state, category='feedback')
    return state
