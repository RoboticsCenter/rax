"""RAX's bimanual xArm 6 + Wuji Hand 2 backend on the MuJoCo twin.

The robot service is the real one (services/xarm_bimanual/backend.py
XArmBimanualBackend, unchanged): same MCP tools, same admission, guards, settling and
stop logic, same calibration loader and observation code. Only what sits below the
vendor SDKs changes:
- arms: RAX's XArmDriver on SimXArmAPI (xarm_api.py) instead of xarm.wrapper.XArmAPI;
- hands: RAX's hand service process (services/wuji/controller.py serve, with its
  WujiDriver) started with the wuji_sdk mimic (hand_service.py) instead of wuji_sdk;
- cameras: a camera server with the real one's socket protocol that renders MuJoCo
  views (camera_server.py) instead of reading RealSense devices;
- the devices themselves: one sim server process (server.py) owning the physics.

Simulation-only additions, both off unless the session's `simulation` block enables
them: sim_trace_start / sim_trace_stop (1 kHz firmware-level traces in the shared trace
format, next to the robot service's own trace_start / trace_stop) and the PRIVILEGED
sim_* tools (ground truth, reset, object placement).
"""
from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from services.json_io import write_private_json
from services.wuji.client import WujiClient, side_suffix, PROJECT as HAND_PROJECT
from services.xarm.backend import CameraService, DEFAULTS
from services.xarm_bimanual.backend import XArmBimanualBackend, _NoCameras
from services.xarm_bimanual.config import SIDES, arm_options, validate_options

from . import shm
from .config import complete, validate_simulation
from .rpc import Client, socket_path
from .spec import options as sim_options
from .xarm_api import SimXArmDriver

PROJECT = Path(__file__).resolve().parents[2]


class SimServerProcess:
    """The sim server child process for one session."""

    def __init__(self, root, options, *, timeout=60.0):
        self.root = Path(root)
        self.socket = socket_path(root)
        self.shm_dir = shm.private_directory(shm.token_for(root))
        options_path = self.root/'operator'/'sim-options.json'
        write_private_json(options_path, options)
        self.socket.unlink(missing_ok=True)
        self.log = open(self.root/'sim-server.log', 'ab')
        self.process = subprocess.Popen(
            [sys.executable, '-m', 'services.sim_xarm_wuji.server', '--options', str(options_path),
             '--socket', str(self.socket), '--shm-dir', str(self.shm_dir), '--parent-pid', str(os.getpid())],
            cwd=PROJECT, env={**os.environ, 'PYTHONPATH': str(PROJECT)}, stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.client = Client(self.socket)
        deadline = time.monotonic() + timeout
        while True:
            if self.process.poll() is not None:
                self.close()
                raise RuntimeError(f'sim server exited ({self.process.returncode}); see {self.root/"sim-server.log"}')
            if self.socket.exists():
                try:
                    self.hello = self.client.call('sim.hello')
                    break
                except OSError:
                    pass
            if time.monotonic() > deadline:
                self.close()
                raise RuntimeError('sim server did not start')
            time.sleep(0.05)

    def call(self, op, **args):
        return self.client.call(op, **args)

    def close(self):
        self.client.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.log.close()


class SimWujiClient(WujiClient):
    """WujiClient (services/wuji/client.py) whose hand service runs on the sim SDK.

    Identical to WujiClient.__init__ except the child module (hand_service instead of
    services.wuji.controller) and its --sim-socket argument."""

    def __init__(self, session, on_fault=None, *, side=None, sim_socket):
        from services.wuji.client import socket_path as hand_socket
        from services.wuji.config import validate_config
        from services.wuji.sdk import private_directory
        self.cfg = validate_config(session.backend_options['end_effector'])
        self.side = side
        self.root, self.path = session.root, hand_socket(session.root, side)
        self.on_fault = on_fault
        self.stop_event = threading.Event()
        self.cached = {'connected': False, 'fresh': False, 'enabled': False, 'mode': 'starting'}
        self.received = time.monotonic()
        self.process = self.thread = None
        private_directory()
        config_path = self.root/'operator'/f'wuji{side_suffix(side)}-configuration.json'
        config_path.parent.mkdir(parents=True, exist_ok=True)
        write_private_json(config_path, session.to_dict())
        if self.path.exists():
            raise RuntimeError('hand service socket already exists; stop its owner before restarting this session')
        with open(self.root/f'wuji{side_suffix(side)}-controller.log', 'ab') as log:
            self.process = subprocess.Popen(
                [sys.executable, '-m', 'services.sim_xarm_wuji.hand_service', '--config', str(config_path),
                 '--socket', str(self.path), '--parent-pid', str(os.getpid()),
                 *(['--side', side] if side else []), '--sim-socket', str(sim_socket)], cwd=HAND_PROJECT,
                env={**os.environ, 'PYTHONPATH': str(HAND_PROJECT), 'WUJI_TELEMETRY_DISABLE': '1'},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+10
        while not self.path.exists():
            if self.process.poll() is not None or time.monotonic() > deadline:
                self.close()
                raise RuntimeError('simulated hand service did not start; see wuji-controller.log')
            time.sleep(0.02)
        self.thread = threading.Thread(target=self._monitor, name='wuji-supervisor', daemon=True)
        self.thread.start()


class SimCameraService(CameraService):
    """CameraService (services/xarm/backend.py) whose server renders the sim world."""

    def __init__(self, options, frames_root, log_path, depth_mode, stream=None, *, sim_socket):
        super().__init__(options, frames_root, log_path, depth_mode, stream=stream)
        self.sim_socket = sim_socket
        directory = Path(f'/tmp/rax-xarm-cam-{os.getuid()}')
        self.socket_path = directory/('sim-' + hashlib.sha256(str(frames_root).encode()).hexdigest()[:16] + '.sock')

    def _start(self):
        specs = [{k: v for k, v in spec.items() if k not in ('extrinsics', 'intrinsics')}
                 for spec in self.options['cameras']]
        command = [sys.executable, '-m', 'services.sim_xarm_wuji.camera_server', '--socket', str(self.socket_path),
                   '--sim-socket', str(self.sim_socket), '--cameras', json.dumps(specs),
                   '--depth-mode', self.depth_mode, '--frames-root', str(self.frames_root),
                   '--fps', str(self.options['stream_fps'])]
        if self.stream is not None:
            command += ['--stream', str(self.stream), '--window-seconds', str(self.options['stream_window_s']),
                        '--depth-hz', str(self.options['stream_depth_hz']),
                        '--stream-quality', str(self.options['stream_jpeg_quality'])]
        log = open(self.log_path, 'ab')
        self.socket_path.unlink(missing_ok=True)
        self.process = subprocess.Popen(command, cwd=PROJECT, env={**os.environ, 'PYTHONPATH': str(PROJECT),
                                        'MUJOCO_GL': os.environ.get('MUJOCO_GL', 'egl')},
                                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 60
        while not self.socket_path.exists():
            if self.process.poll() is not None:
                raise RuntimeError(f'sim camera server exited ({self.process.returncode}); see camera-server.log')
            if time.monotonic() > deadline:
                raise RuntimeError('sim camera server did not start')
            time.sleep(0.05)


def _schema(properties=None, required=()):
    return {'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}


class SimXArmBimanualBackend(XArmBimanualBackend):
    def __init__(self, config):
        options = complete(config.backend_options)
        validate_options(options)
        validate_simulation(options)
        config = replace(config, backend_options=options)
        self.sim = sim_options(options['simulation'])
        self.sim_server = SimServerProcess(config.root, options)
        arms, hands, cameras = {}, {}, None
        try:
            for side in SIDES:
                arms[side] = SimXArmDriver({**DEFAULTS, **arm_options(options, side)}, self.sim_server.socket)
                child = replace(config, robot='xarm', backend_options=arm_options(options, side))
                hands[side] = SimWujiClient(child, lambda side=side: self._hand_fault(side), side=side,
                                            sim_socket=self.sim_server.socket)
            if self.sim['cameras']:
                camera_options = {**DEFAULTS, **options}
                stream = config.root/'stream' if camera_options['stream_enabled'] else None
                cameras = SimCameraService(camera_options, config.root/'agent-workspace'/'runtime'/'frames',
                                           config.root/'camera-server.log',
                                           camera_options['camera_depth_mode']
                                           if config.observation_profile == 'rgbd' else 'NONE',
                                           stream=stream, sim_socket=self.sim_server.socket)
            else:
                cameras = _NoCameras()
            super().__init__(config, arms=arms, hands=hands, cameras=cameras)
        except BaseException:
            for hand in hands.values():
                try:
                    hand.close()
                except Exception:
                    pass
            for arm in arms.values():
                try:
                    arm.close()
                except Exception:
                    pass
            self.sim_server.close()
            raise
        try:
            if self.stream is not None and isinstance(self.cameras, SimCameraService):
                self.stream.mkdir(parents=True, exist_ok=True)
                self.cameras.start()
            if self.sim['auto_prepare']:
                self.prepare_simulation()
        except BaseException:
            self.close()
            raise

    # ---- operator role in simulation ------------------------------------------------------------------------

    def prepare_simulation(self, minutes=None):
        """What the operator does on the rig: ready, arm, prepare (simulation only)."""
        from .operator import prepare
        states = prepare(self.config.root, self.config.backend_options, minutes=minutes)
        # wuji_status reads each hand client's 20 Hz heartbeat cache: return once the
        # backend itself sees both holds, so the first tool call reports them.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not all(
                self.arms[side].end_effector.cached.get('mode') == 'holding' for side in SIDES):
            time.sleep(0.01)
        return states

    # ---- simulation-only tools ----------------------------------------------------------------------------

    def tools(self):
        definitions = super().tools()
        if self.sim['trace_tools']:
            definitions += [
                {'name': 'sim_trace_start', 'description':
                    'SIMULATION: start a 1 kHz firmware-level trace of both hands and arms (q_ref, q_meas, dq_meas, '
                    'current, i_ff, kp, kd, temperature_c) in the shared trace format, as the simulated firmware '
                    'sees it (trace_start records the robot service\'s own). directory: absolute path '
                    '(default: the session sim-traces folder); label names the default folder.',
                 'inputSchema': _schema({'directory': {'type': 'string'}, 'label': {'type': 'string'}})},
                {'name': 'sim_trace_stop', 'description': 'SIMULATION: stop the trace and write hand_<side>.npz, '
                    'arm_<side>.npz and meta.json (backend "sim").', 'inputSchema': _schema()}]
        if self.sim['privileged_tools']:
            definitions += [
                {'name': 'sim_ground_truth', 'description':
                    'PRIVILEGED, SIMULATION ONLY: true object poses (left base frame), thread/cap/bulb state, '
                    'fingertip positions and hand contacts. Never available on the real robot: use only for sim '
                    'checks or where an equivalent perception skill exists.', 'inputSchema': _schema()},
                {'name': 'sim_reset', 'description':
                    'PRIVILEGED, SIMULATION ONLY: redraw the scene objects (seed), or load another scene '
                    '(re-preparing the simulated rig).',
                 'inputSchema': _schema({'seed': {'type': 'integer'}, 'scene': {'type': 'string'}})},
                {'name': 'sim_set_object_pose', 'description':
                    'PRIVILEGED, SIMULATION ONLY: place a free scene object (left base frame, metres, [x,y,z,w]).',
                 'inputSchema': _schema({'name': {'type': 'string'},
                                         'position': {'type': 'array', 'items': {'type': 'number'},
                                                      'minItems': 3, 'maxItems': 3},
                                         'quaternion': {'type': 'array', 'items': {'type': 'number'},
                                                        'minItems': 4, 'maxItems': 4}}, ('name', 'position'))}]
        return definitions

    def _sim_trace_start(self, directory=None, label=None):
        if directory is None:
            stamp = label or time.strftime('%Y%m%dT%H%M%S')
            if not all(c.isalnum() or c in '-_.' for c in stamp):
                raise ValueError('label must be a path-safe name')
            directory = self.config.root/'sim-traces'/stamp
        directory = Path(directory)
        if not directory.is_absolute():
            raise ValueError('directory must be an absolute path')
        return self.sim_server.call('sim.trace_start', directory=str(directory)), []

    def _sim_trace_stop(self):
        return self.sim_server.call('sim.trace_stop'), []

    def _sim_ground_truth(self):
        return self.sim_server.call('sim.ground_truth'), []

    def _sim_reset(self, seed=None, scene=None):
        if scene is None:
            return self.sim_server.call('sim.reset', seed=seed), []
        # A new scene is a new world: both devices power on again, as after a rig restart.
        # Stop and disarm first, so the hand services see an expected stop, not a fault.
        stopped = self._stop_assembly(verify_arm=False, require_healthy=False)
        value = self.sim_server.call('sim.load_scene', scene=scene, seed=seed)
        value['stopped'] = {name: result.get('confirmed') for name, result in stopped.items()}
        self.sim = {**self.sim, 'scene': scene, 'seed': value.get('seed', seed)}
        if self.sim['auto_prepare']:
            for side in SIDES:  # a powered-off hand must be reconnected before it can prepare
                self.arms[side].end_effector._operation('connect', timeout=10.0)
            value['prepared'] = {side: s.get('mode') for side, s in self.prepare_simulation().items()}
        return value, []

    def _sim_set_object_pose(self, name, position, quaternion=None):
        return self.sim_server.call('sim.set_object_pose', name=name, position=position,
                                    quaternion_xyzw=quaternion), []

    def status(self):
        value = super().status()
        value['simulation'] = {'backend': 'sim', 'scene': self.sim['scene'], 'seed': self.sim['seed'],
                               'privileged_tools': self.sim['privileged_tools'],
                               'trace_tools': self.sim['trace_tools']}
        return value

    def close(self):
        try:
            super().close()
        finally:
            server = getattr(self, 'sim_server', None)
            if server is not None:
                server.close()
