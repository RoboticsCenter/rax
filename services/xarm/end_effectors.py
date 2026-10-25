"""End-effector selection, independent of xArm motion and camera code."""
from services.wuji.config import TOOL_NAMES, validate_config


def kind(options):
    value = options.get('end_effector')
    if value is None:
        return 'g2' if options.get('gripper_type') == 'g2' else 'xarm_gripper'
    if not isinstance(value, dict) or value.get('type') not in ('g2', 'xarm_gripper', 'wuji_hand2'):
        raise ValueError('end_effector.type must be g2, xarm_gripper or wuji_hand2')
    if value['type'] == 'wuji_hand2':
        validate_config(value)
    elif set(value) != {'type'}:
        raise ValueError('gripper settings remain in the existing gripper_* options')
    return value['type']


def tool_names(options):
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
    if kind(session.backend_options) == 'wuji_hand2':
        from services.wuji.client import WujiClient
        return WujiClient(session, on_fault)
    return Gripper(arm)
