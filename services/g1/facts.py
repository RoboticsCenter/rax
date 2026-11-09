"""Embodiment metadata for the existing workspace template registry."""
ROBOT_FACTS = {
    'name': 'Unitree G1 EDU 29-DoF with two Wuji hands',
    'status': 'humanoid_status', 'observe': 'humanoid_observe',
    'move': 'humanoid_move_base', 'joints': 'wuji_move_joints',
    'gripper': 'wuji_pose', 'stream': 'humanoid_stream', 'pixel': 'unavailable',
    'tools': ('humanoid_status', 'humanoid_observe', 'humanoid_move_base',
              'humanoid_stop', 'humanoid_stream', 'wuji_status', 'wuji_move_joints',
              'wuji_pose', 'wuji_hold'),
    'overview_camera': 'overview', 'overview': 'operator-configured RGB camera',
    'cameras': ('overview',),
}
