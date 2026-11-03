"""MCP_CONTRACT.json and MCP_SESSION.md: the session's tool list in the workspace."""
import json
from pathlib import Path


def describe(setup) -> str:
    if setup.robot == 'xarm_bimanual':
        calibrated = bool(setup.backend_options.get('calibration_manifest'))
        geometry = ('Four camera views and both assemblies’ state are supplied. '
                    + ('Verified RGB models, camera poses in both bases and the inter-base transform are loaded. '
                       'xarm_observe arm selects the observation/projection base (default left); project marks '
                       'points in that base. Native image coordinates and preview_scale are reported. '
                       if calibrated else 'Camera-to-arm and inter-arm calibration are pending. ')
                    + 'Depth metric validation is pending; no depth pixel-to-base tool is exposed. '
                    'Arm tools require arm=left|right, hand tools require hand=left|right. '
                    'Each arm motion target still uses its own base frame.')
    elif setup.camera_geometry:
        geometry = 'RGB images, depth and camera calibration are supplied.'
    else:
        geometry = 'Only RGB images and measured robot state are supplied.'
    listing = ('Tools: ' + ', '.join(f'`{name}`' for name in setup.tools)
               + '. The live tool schemas give arguments and limits. `status` reports the session budgets '
               'and the live robot; `request_operator` asks the human operator for help or a scene reset.')
    return (f'# Session tools\n\nRobot: {setup.context()["robot"]["name"]} (real hardware). '
            f'Observation profile: {setup.observation_profile}.\n\n{geometry}\n\n{listing}\n')


def manifest(setup):
    return {'robot': setup.robot, 'end_effector': setup.end_effector, 'observation_profile': setup.observation_profile,
            'subagents': setup.subagents, 'tools': list(setup.tools)}


def write_contract(workspace, setup):
    workspace = Path(workspace)
    (workspace / 'MCP_CONTRACT.json').write_text(json.dumps(manifest(setup), indent=2) + '\n')
    (workspace / 'MCP_SESSION.md').write_text(describe(setup))
