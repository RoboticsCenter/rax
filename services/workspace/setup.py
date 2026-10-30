"""The facts a workspace is generated from; roles live in recipes, not here."""
from dataclasses import dataclass, field

# Words shared sources use for the robot instead of branching on it. Tool names are the
# backend's MCP tools (services/backends.py); keep them in sync with the backend.
ROBOT_FACTS = {
    'franka': {'name': 'Franka FR3', 'status': 'franka_status', 'observe': 'franka_observe',
               'move': 'franka_move_pose', 'joints': 'franka_move_joints', 'gripper': 'franka_gripper',
               'pixel': 'franka_pixel_to_position', 'stream': 'franka_stream',
               'tools': ('franka_status', 'franka_observe', 'franka_move_pose', 'franka_move_joints',
                         'franka_gripper', 'franka_pixel_to_position', 'franka_stream', 'franka_hold'),
               'overview_camera': 'left', 'overview': 'left exterior camera',
               'cameras': ('left', 'right', 'wrist')},
    'xarm': {'name': 'UFactory xArm 6', 'status': 'xarm_status', 'observe': 'xarm_observe',
             'move': 'xarm_move_pose', 'joints': 'xarm_move_joints', 'gripper': 'xarm_gripper',
             'pixel': 'xarm_pixel_to_position', 'stream': 'xarm_stream',
             'tools': ('xarm_status', 'xarm_observe', 'xarm_move_pose', 'xarm_move_joints',
                       'xarm_gripper', 'xarm_pixel_to_position', 'xarm_stream', 'xarm_hold'),
             'overview_camera': 'left', 'overview': 'left sideview camera',
             'cameras': ('left', 'right', 'wrist')},
    'mock': {'name': 'mock robot', 'status': 'status', 'observe': 'mock_observe', 'move': 'mock_move', 'joints': 'mock_move',
             'gripper': 'mock_move', 'pixel': 'mock_observe', 'stream': 'mock_observe', 'tools': ('mock_observe', 'mock_move'),
             'overview_camera': 'none', 'overview': 'no camera', 'cameras': ()},
}
CLIS = ('claude',)


@dataclass(frozen=True)
class Setup:
    robot: str = 'xarm'
    task: str | None = None
    cli: str = 'claude'
    observation_profile: str = 'rgbd'
    subagents: int = 0
    gemini: bool = True
    backend_options: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.robot not in ROBOT_FACTS:
            raise ValueError(f'Unknown robot {self.robot!r}')
        if self.cli not in CLIS:
            raise ValueError('Unknown agent CLI')
        if self.observation_profile not in ('rgbd', 'rgb-only'):
            raise ValueError('Unknown observation profile')
        if self.robot == 'xarm':
            from services.xarm.end_effectors import kind
            kind(self.backend_options)

    @classmethod
    def from_config(cls, config, *, gemini=True):
        return cls(config.robot, config.task, 'claude', config.observation_profile, config.subagents, gemini,
                   config.backend_options)

    @property
    def end_effector(self):
        if self.robot == 'xarm':
            from services.xarm.end_effectors import kind
            return kind(self.backend_options)
        return 'gripper' if self.robot == 'franka' else 'none'

    @property
    def robot_facts(self):
        facts = dict(ROBOT_FACTS[self.robot])
        cameras = self.backend_options.get('cameras')
        if cameras:
            facts['cameras'] = tuple(camera['name'] for camera in cameras)
            facts['fixed_cameras'] = tuple(camera['name'] for camera in cameras if camera.get('mount') == 'fixed')
            facts['wrist_cameras'] = tuple(camera['name'] for camera in cameras if camera.get('mount') == 'tcp')
            facts['overview_camera'] = (facts['fixed_cameras'] or facts['cameras'])[0]
            facts['overview'] = f"{facts['overview_camera']} camera"
        else:
            facts['fixed_cameras'] = tuple(name for name in facts['cameras'] if name != 'wrist')
            facts['wrist_cameras'] = tuple(name for name in facts['cameras'] if name == 'wrist')
        facts.update(has_hand=self.end_effector == 'wuji_hand2', end_effector=self.end_effector,
                     effector_name='gripper', effector_tool=facts['gripper'])
        if self.robot == 'xarm':
            from services.xarm.end_effectors import tool_names
            facts['tools'] = tuple(t for t in facts['tools'] if t != 'xarm_gripper') + tool_names(self.backend_options)
        if facts['has_hand']:
            side = self.backend_options['end_effector'].get('handedness', 'right')
            facts.update(effector_name=f'{side} Wuji Hand 2', effector_tool='wuji_move_joints',
                         hand_status='wuji_status', hand_pose='wuji_pose', hand_home='wuji_home',
                         hand_hold='wuji_hold')
            facts.pop('gripper')  # Accidental gripper-template references fail rendering.
        return facts

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @property
    def tools(self):
        """Every MCP tool the session exposes, sorted."""
        return tuple(sorted({'status', 'request_operator', *self.robot_facts['tools'],
                             *(('gemini_generate',) if self.gemini else ())}))

    def axes(self):
        """Which skill layers a workspace takes: the robot it controls and how it perceives."""
        return {'robot': self.robot, 'perception': 'depth' if self.camera_geometry else 'rgb-only',
                'end_effector': self.end_effector}

    def context(self):
        """Template variables: facts only, named for readability in the sources."""
        return {
            'agent_cli': self.cli,
            'task': self.task,
            'observation_profile': self.observation_profile,
            'camera_geometry': self.camera_geometry,
            'subagent_cap': self.subagents,
            'gemini': self.gemini,
            'robot': self.robot_facts,
        }
