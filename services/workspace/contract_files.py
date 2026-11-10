"""MCP_CONTRACT.json and MCP_SESSION.md: the session's tool list in the workspace."""
import json
from pathlib import Path


def describe(setup) -> str:
    if setup.robot == 'xarm_bimanual':
        calibrated = bool(setup.backend_options.get('calibration_manifest'))
        geometry = ('Configured camera views and both assemblies’ state are supplied. '
                    + ('RGB models, verified camera poses in both bases and the inter-base transform are loaded. '
                       'Views marked extrinsics_unverified remain visual-only; read per-camera status. '
                       'xarm_observe arm selects the observation/projection base (default left); project marks '
                       'points in that base. Native image coordinates and preview_scale are reported. '
                       if calibrated else 'Camera-to-arm and inter-arm calibration are pending. ')
                    + ('RGB and aligned uint16 millimetre depth PNGs are saved for every camera. '
                       if setup.observation_profile == 'rgbd' else 'Depth capture is disabled. ')
                    + 'Depth metric validation is pending; no depth pixel-to-base tool is exposed. '
                    'Single-arm tools require arm=left|right, single-hand tools require hand=left|right. '
                    'xarm_move_combined accepts any subset of left_arm, right_arm, left_hand and right_hand '
                    'for concurrent motion within one MCP call. Each arm motion target still uses its own base frame.')
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
