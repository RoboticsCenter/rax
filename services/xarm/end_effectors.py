"""End-effector selection, independent of xArm motion and camera code."""
from services.wuji.config import TOOL_NAMES, validate_config


def kind(options):
    value = options.get('end_effector')
    if value is None:
        return 'g2' if options.get('gripper_type') == 'g2' else 'xarm_gripper'
    if not isinstance(value, dict) or value.get('type') not in ('g2', 'xarm_gripper', 'wuji_hand2', 'brainco_revo2'):
        raise ValueError('end_effector.type must be g2, xarm_gripper, wuji_hand2 or brainco_revo2')
    if value['type'] == 'wuji_hand2':
        validate_config(value)
    elif value['type'] == 'brainco_revo2':
        from services.revo2.config import validate_config as validate_revo
        validate_revo(value)
    elif set(value) != {'type'}:
        raise ValueError('gripper settings remain in the existing gripper_* options')
    return value['type']


def tool_names(options):
    if kind(options) == 'brainco_revo2':
        from services.revo2.config import tools, validate_config as validate_revo
        return tuple(t['name'] for t in tools(validate_revo(options['end_effector'])))
    return TOOL_NAMES if kind(options) == 'wuji_hand2' else ('xarm_gripper',)


class Gripper:
    """Preserve the working native G2/standard-gripper adapter."""
    def __init__(self, arm):
        self.arm = arm

    def status(self):
        return self.arm.gripper_state()

    def move(self, width, **kwargs):
        return self.arm.gripper_move(width, **kwargs)

    def close(self):
        pass  # Connection remains owned by XArmDriver.


def make(session, arm, *, on_fault=None):
    if kind(session.backend_options) == 'brainco_revo2':
        from services.revo2.client import RevoHand
        return RevoHand(session, on_fault)
    if kind(session.backend_options) == 'wuji_hand2':
        from services.wuji.client import WujiClient
        return WujiClient(session, on_fault)
    return Gripper(arm)
