"""The MuJoCo twin of the bimanual xArm 6 + Wuji Hand 2 rig (services/sim_xarm_wuji).

Hardware-free: the twin needs MuJoCo and the pinned vendor models
(`python -m services.sim_xarm_wuji.assets fetch`); tests skip without them. Camera
checks also need an offscreen OpenGL context (EGL).
"""
from functools import lru_cache
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
from services.sim_xarm_wuji import assets  # noqa: E402

pytestmark = pytest.mark.skipif(not assets.available(), reason='simulation assets not fetched '
                                '(python -m services.sim_xarm_wuji.assets fetch)')
PROJECT = Path(__file__).resolve().parents[2]
EXAMPLE = json.loads((PROJECT/'examples'/'xarm-wuji-bimanual-sim.json').read_text())


def world(scene='empty', seed=0, randomize=False, **hand):
    from services.sim_xarm_wuji.world import SimWorld
    options = json.loads(json.dumps(EXAMPLE))
    if hand:
        options['simulation']['hand'] = {**options['simulation']['hand'], **hand}
    return SimWorld(options, scene=scene, seed=seed, randomize=randomize)


def enable(w, side='right', kp=8.0, kd=0.15, cap=0.8):
    fw = w.firmware[side]
    fw.set_effort_limit(cap)
    fw.set_mit_params([(kp, kd)] * 20)
    q, _, _ = w.hand_measurement(side)
    fw.command(w.time, q, np.zeros(20), np.zeros(20))
    fw.enable()
    w.step(200)
    return fw, w.hand_measurement(side)[0]


@lru_cache(maxsize=1)
def egl_available():
    code = ('import os; os.environ["MUJOCO_GL"]="egl"; import mujoco; '
            'm=mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom size=\'1\'/></worldbody></mujoco>"); '
            'r=mujoco.Renderer(m, 8, 8); r.render(); print("ok", flush=True); os._exit(0)')
    try:
        out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=60)
        return 'ok' in out.stdout
    except (OSError, subprocess.TimeoutExpired):
        return False


# ---- assets and geometry --------------------------------------------------------------------------------------------

def test_assets_are_the_pinned_commits_and_unchanged():
    assert assets.verify() == {name: [] for name in assets.SOURCES}
    provenance = assets.provenance()
    assert provenance['wuji-description']['license'] == 'MIT'
    assert provenance['xarm_ros2']['license'] == 'BSD-3-Clause'


def test_rig_loads_with_xarm6_kinematics_and_calibrated_bases():
    w = world()
    m, d = w.model, w.data
    for side in ('left', 'right'):
        d.qpos[w.ix.arm_qpos[side]] = 0.0
    mujoco.mj_forward(m, d)
    # xArm 6 at zero joints: flange at (207, 0, 112) mm, pointing down (UFACTORY's kinematics).
    assert np.allclose(d.site_xpos[w.ix.flange_site['left']], [0.207, 0.0, 0.112], atol=1e-4)
    assert np.allclose(d.site_xpos[w.ix.tcp_site['left']], [0.207, 0.0, 0.112 - 0.172], atol=1e-4)
    right_base = np.asarray(w.rig.manifest['T_left_base_right_base'])
    assert np.allclose(d.xpos[w.ix.base_body['right']], right_base[:3, 3], atol=1e-6)
    assert m.nu == 2 * (6 + 20)
    assert sorted(w.rig.cameras) == ['front', 'side_right', 'wrist_left', 'wrist_right']


def test_cameras_render_where_rax_projects():
    if not egl_available():
        pytest.skip('no EGL rendering context')
    code = f'''
import itertools, json, os
os.environ["MUJOCO_GL"] = "egl"
import cv2 as cv, mujoco, numpy as np
from services.sim_xarm_wuji.world import SimWorld
from services.xarm.calibration import project
w = SimWorld(json.load(open({str(PROJECT/'examples'/'xarm-wuji-bimanual-sim.json')!r})), scene="cube", randomize=False)
m, d, cube = w.model, w.data, w.scene.objects[0]
R = d.xmat[cube.body].reshape(3, 3)
corners = [d.xpos[cube.body] + R @ (0.025 * np.array(s)) for s in itertools.product((-1, 1), repeat=3)]
out = {{}}
for name in ("front", "wrist_right"):
    spec = w.rig.cameras[name]
    cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, w.rig.model_camera(name))
    K = w.rig.manifest["camera_models"][name]["K"]
    if spec["mount"] == "fixed":
        T = np.linalg.inv(np.asarray(w.rig.manifest["fixed"][name]["T_cam_left_base"]))
    else:
        s = w.ix.tcp_site[spec["arm"]]
        T = np.eye(4); T[:3, :3] = d.site_xmat[s].reshape(3, 3); T[:3, 3] = d.site_xpos[s]
        T = T @ np.linalg.inv(np.asarray(w.rig.manifest["wrists"][spec["arm"]]["T_cam_tcp"]))
    uv = np.array([project(c, T, K) for c in corners])
    r = mujoco.Renderer(m, int(m.cam_resolution[cam][1]), int(m.cam_resolution[cam][0]))
    r.update_scene(d, cam)
    hsv = cv.cvtColor(cv.cvtColor(r.render(), cv.COLOR_RGB2BGR), cv.COLOR_BGR2HSV)
    r.close()
    ys, xs = np.nonzero(((hsv[..., 0] < 8) | (hsv[..., 0] > 172)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 40))
    out[name] = [float(uv[:, 0].min() - xs.min()), float(uv[:, 1].min() - ys.min()),
                 float(uv[:, 0].max() - xs.max()), float(uv[:, 1].max() - ys.max())]
print(json.dumps(out), flush=True); os._exit(0)
'''
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=120,
                            cwd=PROJECT, env={**os.environ, 'PYTHONPATH': str(PROJECT)})
    offsets = json.loads(result.stdout.strip().splitlines()[-1])
    for name, values in offsets.items():
        assert max(map(abs, values)) < 2.5, (name, values)  # the cube's silhouette within ~2 px


# ---- hand firmware ---------------------------------------------------------------------------------------------------

def test_hand_moves_to_target_under_the_mit_law():
    w = world()
    fw, q0 = enable(w)
    target = q0.copy()
    target[4] += 0.4   # index S1
    target[9] += 0.2   # middle S2
    fw.command(w.time, target, np.zeros(20), np.zeros(20))
    w.step(1500)
    q, _, current = w.hand_measurement('right')
    assert np.max(np.abs(q - target)) < 0.03
    assert np.all(np.abs(current) <= 0.8 + 1e-9)


def test_current_follows_the_effective_gain_cap_and_feedforward():
    w = world(static_friction_a=0.0, coulomb_friction_a=0.0)
    fw, q0 = enable(w, kp=8.0, kd=0.0, cap=2.0)
    target = q0.copy()
    target[4] += 0.05
    fw.command(w.time, target, np.zeros(20), np.zeros(20))
    w.step(1)
    assert abs(w.hand_measurement('right')[2][4]) < 0.05        # 2 ms command latency: not yet applied
    w.step(2)
    expected = 1.98 * 8.0 * (target[4] - w.hand_measurement('right')[0][4])
    assert w.hand_measurement('right')[2][4] == pytest.approx(expected, rel=0.1)
    fw.set_effort_limit(0.3)
    target[4] += 0.2
    fw.command(w.time, target, np.zeros(20), np.zeros(20))
    w.step(4)
    assert w.hand_measurement('right')[2][4] == pytest.approx(0.3, abs=1e-6)  # clamped by the cap
    # Feed-forward current (MIT effort, amperes) passes through the same 1.98 gain.
    fw.set_mit_params([(0.0, 0.0)] * 20)
    fw.set_effort_limit(2.0)
    i_ff = np.zeros(20)
    i_ff[4] = 0.1
    fw.command(w.time, target, np.zeros(20), i_ff)
    w.step(3)
    assert w.hand_measurement('right')[2][4] == pytest.approx(0.198, abs=1e-6)


def _ramp(w, joint=4, speed=0.02, seconds=2.0):
    fw, start = enable(w)
    vel = []
    for k in range(int(seconds * 200)):
        target = start.copy()
        target[joint] += speed * k / 200.0
        dq = np.zeros(20)
        dq[joint] = speed
        fw.command(w.time, target, dq, np.zeros(20))
        for _ in range(5):
            w.step(1)
            vel.append(w.hand_measurement('right')[1][joint])
    return np.abs(np.asarray(vel[100:]))


def test_stick_slip_appears_with_friction_and_not_without():
    sticky = _ramp(world())
    smooth = _ramp(world(static_friction_a=0.0, coulomb_friction_a=0.0))
    assert np.mean(sticky < 0.002) > 0.5            # mostly stuck ...
    assert sticky.max() > 8 * 0.02                  # ... then slipping much faster than the reference
    assert np.mean(smooth < 0.002) < 0.1
    assert smooth.max() < 3 * 0.02


def test_partial_left_hand_reports_the_rigs_isolated_faults():
    w = world()
    diag = w.firmware['left'].diagnostics()
    from services.wuji.config import JOINT_NAMES
    codes = {JOINT_NAMES[i]: int(c) for i, c in enumerate(diag['error_code_current']) if c}
    assert codes == {'thumb_S3': 0x2108, 'middle_S1': 0x3208}
    w.firmware['left'].enable()
    states = w.firmware['left'].diagnostics()['states']
    assert states[JOINT_NAMES.index('thumb_S3')] == 1 and states[0] == 2


# ---- threads ---------------------------------------------------------------------------------------------------------

def _hold_still(w, body):
    w.model.body_mass[body] = 50.0
    w.model.body_inertia[body] = w.model.body_inertia[body] * 5000.0


def test_cap_thread_advances_under_rotation_releases_and_reseats():
    w = world('bottle', seed=2)
    cap = w.scene.objects[0]
    _hold_still(w, cap.body)
    d = w.data
    assert cap.thread_state(w.model, d)['seated']
    d.qfrc_applied[cap.turn_v] = 0.5 * cap.breakaway_nm
    w.step(300)
    assert abs(d.qpos[cap.turn_q]) < 0.02                      # below breakaway: held
    d.qfrc_applied[cap.turn_v] = 1.5 * cap.breakaway_nm
    w.step(60)
    turn = float(d.qpos[cap.turn_q])
    assert turn > 0.2
    assert d.qpos[cap.lift_q] == pytest.approx(cap.lead * turn / (2 * math.pi), abs=5e-5)  # thread coupling
    for _ in range(100):
        if not cap.threaded:
            break
        w.step(20)
    assert not cap.threaded and [e['event'] for e in cap.events][-1] == 'removed'
    d.qfrc_applied[cap.turn_v] = 0.0
    w.step(800)                                                # the free cap settles on its thread start
    origin, R = cap._thread_frame(d)
    local = R.T @ (d.xpos[cap.free] - origin)
    assert local[2] == pytest.approx(cap.lift_release, abs=0.003) and np.hypot(*local[:2]) < 0.003
    for _ in range(400):                                       # turn it clockwise: the thread catches
        d.qvel[cap.free_v + 3:cap.free_v + 6] = cap and (d.xmat[cap.free].reshape(3, 3).T @ (R[:, 2] * -3.0))
        w.step(1)
        if cap.threaded:
            break
    assert cap.threaded and cap.events[-1]['event'] == 'engaged'
    d.qfrc_applied[cap.turn_v] = -0.2                          # screw it down
    w.step(2000)
    state = cap.thread_state(w.model, d)
    assert state['seated'] and state['events'][-1]['event'] == 'seated'
    assert state['friction_nm'] == pytest.approx(cap.breakaway_nm)  # a seated cap needs breakaway again


def test_bulb_engages_screws_in_and_lights():
    w = world('bulb', seed=1)
    bulb = w.scene.objects[0]
    d = w.data
    assert not bulb.threaded and not bulb.lit
    origin, R = bulb._thread_frame(d)
    d.qpos[bulb.free_q:bulb.free_q + 3] = origin + R[:, 2] * (bulb.lift_release + 0.001)
    d.qpos[bulb.free_q + 3:bulb.free_q + 7] = [1, 0, 0, 0]
    d.qvel[bulb.free_v:bulb.free_v + 6] = 0.0
    mujoco.mj_forward(w.model, d)
    w.step(100)
    for _ in range(400):
        d.qvel[bulb.free_v + 5] = -3.0
        w.step(1)
        if bulb.threaded:
            break
    assert bulb.threaded
    d.qfrc_applied[bulb.turn_v] = -0.15
    w.step(3000)
    state = w.scene.state(w.model, d)['objects']['bulb']
    assert state['bulb']['seated'] and state['lit']


def test_ground_truth_is_marked_privileged():
    truth = world('household').ground_truth()
    assert truth['privileged'] is True and truth['frame'] == 'left_base'
    assert set(truth['objects']) == {'mug', 'marker', 'small_box'}


# ---- configuration and SDK surface -----------------------------------------------------------------------------------

def test_simulated_options_refuse_real_device_addresses():
    from services.sim_xarm_wuji.config import complete, validate_simulation
    options = complete({})
    validate_simulation(options)
    for path, value in ((('arms', 'left', 'xarm_ip'), '192.168.2.205'),
                        (('arms', 'right', 'end_effector', 'serial'), 'WH2KA01260823017'),
                        (('arms', 'right', 'end_effector', 'address'), '192.168.1.111:50001')):
        bad = json.loads(json.dumps(options))
        target = bad
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        with pytest.raises(ValueError):
            validate_simulation(bad)


def test_backend_selection():
    from services.backends import BACKENDS, selected_backend
    from services.controller.config import Configuration
    assert 'xarm_wuji_bimanual_sim' in BACKENDS
    config = Configuration(root=Path(os.environ['RAX_ARTIFACT_ROOT'])/'s', task='t', robot='xarm_bimanual',
                           backend_options=EXAMPLE)
    assert selected_backend(config) == 'xarm_wuji_bimanual_sim'
    assert Configuration(root=Path(os.environ['RAX_ARTIFACT_ROOT'])/'s2', task='t',
                         robot='xarm_wuji_bimanual_sim', backend_options={}).robot == 'xarm_wuji_bimanual_sim'


def test_wuji_sdk_mimic_covers_the_vendor_surface():
    import ast
    import importlib.util
    from services.sim_xarm_wuji import wuji_sdk
    used = ('handedness', 'fingertip_online', 'joint_states', 'joint_diagnostics', 'joint_command', 'effort_limit',
            'mit_params', 'enable', 'disable', 'clear_fault', 'emergency_stop', 'set_constant', 'set_position_table',
            'joint', 'joints', 'fingers', 'nid_to_joint_index', 'joint_index_to_nid', 'describe_error', 'disconnect')
    assert all(callable(getattr(wuji_sdk.WujiHand2, name)) for name in used)
    assert all(wuji_sdk.WujiHand2.joint_index_to_nid(i) == n for i, n in enumerate(
        (1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24)))
    assert isinstance(wuji_sdk.StatusWord(2).ext_state_name, str)  # a property, as the stubs declare
    spec = importlib.util.find_spec('wuji_sdk')
    if spec is None or not spec.submodule_search_locations:
        pytest.skip('the vendor wuji-sdk is not installed: no stubs to compare')
    stubs = Path(list(spec.submodule_search_locations)[0])/'__init__.pyi'
    tree = ast.parse(stubs.read_text())
    vendor = {item.name for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'WujiHand2'
              for item in node.body if isinstance(item, ast.FunctionDef)}
    assert set(used) <= vendor
    connect = next(item for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'SdkManager'
                   for item in node.body if isinstance(item, ast.FunctionDef) and item.name == 'connect')
    assert {'sn', 'address', 'device_name', 'options'} <= {a.arg for a in connect.args.kwonlyargs}


# ---- the robot service on the twin ---------------------------------------------------------------------------------

class _Ctx:
    """The skills' ctx contract (auto_research_agent/api/runtime.py) on an in-process Frontend."""

    def __init__(self, frontend):
        self.frontend, self.sequence = frontend, 0

    def value(self, tool, **arguments):
        self.sequence += 1
        response = self.frontend.handle({'jsonrpc': '2.0', 'id': self.sequence, 'method': 'tools/call',
                                         'params': {'name': tool, 'arguments': arguments}})
        result = response['result']
        text = next(b['text'] for b in result['content'] if b.get('type') == 'text')
        if result.get('isError'):
            raise RuntimeError(text)
        return json.loads(text)


def test_robot_service_runs_unchanged_on_the_sim_backend(tmp_path):
    from services.backends import make_backend
    from services.controller.config import Configuration
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    from services.dex_eval.trace import validate_trial_dir
    cameras = egl_available()
    config = Configuration(root=tmp_path/'session', task='simtest', robot='xarm_wuji_bimanual_sim',
                           backend_options={'simulation': {'scene': 'cube', 'randomize': False, 'cameras': cameras,
                                                           'privileged_tools': True, 'trace_tools': True}})
    backend = make_backend(config)
    frontend = Frontend(Session(backend.config), backend)
    ctx = _Ctx(frontend)
    try:
        right = ctx.value('wuji_status', hand='right')
        left = ctx.value('wuji_status', hand='left')
        assert right['mode'] == 'holding' and right['fresh'] and right['serial'] == 'SIM-WH2KA-RIGHT'
        assert left['mode'] == 'holding' and left['partial_hand']
        assert {f['joint'] for f in left['isolated_faults']} == {'thumb_S3', 'middle_S1'}
        status = ctx.value('xarm_status')
        assert status['simulation']['backend'] == 'sim' and status['calibration_status'] == 'rgb_extrinsics_verified'
        # The robot service's own trace (services/wuji) and the firmware-level one, side by side.
        workspace = tmp_path/'session'/'agent-workspace'
        started = ctx.value('trace_start', directory=str(workspace/'trial'), sides=['right'])
        ctx.value('sim_trace_start', directory=str(tmp_path/'firmware'))
        target = list(right['target_positions'])
        target[4] += 0.3
        moved = ctx.value('wuji_move_joints', hand='right', positions=target)
        assert moved['outcome'] == 'reached' and moved['max_joint_error_rad'] < 0.1
        arm = ctx.value('xarm_move_pose', arm='right', position=[0.0, 0.0, -0.02], relative=True)
        assert arm['reached'] and arm['position_error_mm'] < 2.0
        firmware = ctx.value('sim_trace_stop')
        assert firmware['samples'] > 1000
        assert validate_trial_dir(tmp_path/'firmware')[0] == []
        assert json.loads((tmp_path/'firmware'/'meta.json').read_text())['backend'] == 'sim'
        stopped = ctx.value('trace_stop', success=True)
        trial = Path(stopped['directory'])
        assert trial == Path(started['directory'])
        assert validate_trial_dir(trial)[0] == []
        assert json.loads((trial/'meta.json').read_text())['backend'] == 'sim'  # RAX_BACKEND_KIND on the mimic
        truth = ctx.value('sim_ground_truth')
        assert truth['privileged'] and 'cube' in truth['objects']
        if cameras:
            cube = truth['objects']['cube']['position']
            observed = ctx.value('xarm_observe', cameras=['front'], arm='left', project=[cube])
            u, v = observed['cameras']['front']['projected_pixels'][0]
            import cv2 as cv
            image = cv.imread(str(tmp_path/'session'/'agent-workspace'/observed['directory']/'front.jpg'))
            b, g, r = image[int(round(v)), int(round(u))].astype(int)
            assert r > 120 and r > g + 50 and r > b + 50   # the projected cube centre is on the red cube
    finally:
        frontend.close()


# ---- the MPC model ---------------------------------------------------------------------------------------------------

def test_mpc_hand_model_rolls_out_in_parallel_and_tracks():
    from services.sim_xarm_wuji.mpc import HandModel, effort_cost, object_pose_cost
    model = HandModel('right', object='cube', nthread=4, wrist_pose=((0.0, 0.0, 0.25), (0.0, 0.0, 0.0, 1.0)))
    try:
        q0 = np.zeros(20)
        model.set_state(q0, object_pose=((0.05, 0.0, 0.025), (0, 0, 0, 1)))
        target = q0.copy()
        target[4] = 0.4
        sequences = np.stack([np.tile(q0, (60, 1)), np.tile(target, (60, 1))])
        result = model.rollout(sequences, control_dt=0.01)
        assert result.q.shape == (2, 60, 20) and result.fingertips.shape == (2, 60, 5, 3)
        assert abs(result.q[1, -1, 4] - 0.4) < 0.03 and abs(result.q[0, -1, 4]) < 0.03
        assert np.abs(result.current).max() <= 0.8 + 1e-6
        costs = object_pose_cost(result, target_position=(0.05, 0.0, 0.025))
        assert costs.shape == (2,) and np.all(np.isfinite(costs))
        assert effort_cost(result)[1] > effort_cost(result)[0]
    finally:
        model.close()
