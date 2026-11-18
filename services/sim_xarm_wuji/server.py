"""The simulated rig's hardware: one process that steps the world in real time and
serves the two xArm controller boxes and the two Wuji hands to their SDK mimics.

    python -m services.sim_xarm_wuji.server --options OPTIONS.json --socket PATH \
        --shm-dir DIR [--parent-pid PID]

- physics thread: SimWorld.step at 1 kHz, paced to wall-clock time (realtime_factor);
  each step reads the hands' latest commands from shared memory and publishes their
  joint_states (state_rate_hz) and joint_diagnostics (diagnostics_rate_hz) frames,
  stamped with the host's CLOCK_MONOTONIC (as the real SDK's frames are host-received);
- RPC (rpc.py): `wuji.*` (the hand's parameter and state services), `xarm.*` (the
  controller box), `sim.*` (scene control and PRIVILEGED ground truth, simulation only).
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socketserver
import sys
import threading
import time

import numpy as np

from . import shm
from .firmware import FirmwareError, describe_error, mask_indices
from .rpc import encode
from .spec import SIDES
from .world import SimWorld


class SimServer:
    def __init__(self, options, socket_path, shm_dir, *, parent_pid=None):
        self.options = options
        self.socket_path, self.shm_dir = Path(socket_path), Path(shm_dir)
        self.parent_pid = parent_pid
        self.stop = threading.Event()
        self.world = SimWorld(options)
        self.model_version = 1
        self.blocks = {}
        self.serial_side = {}
        self.ip_side = {side: options['arms'][side]['xarm_ip'] for side in SIDES}
        for side in SIDES:
            serial = self.world.hand_cfg[side]['serial']
            self.serial_side[serial] = side
            paths = shm.hand_paths(self.shm_dir, serial)
            self.blocks[side] = {'state': shm.Block(paths['state'], shm.STATE_FIELDS, create=True),
                                 'diag': shm.Block(paths['diag'], shm.DIAG_FIELDS, create=True),
                                 'command': shm.Block(paths['command'], shm.COMMAND_FIELDS, create=True)}
        self.last_command = {side: 0 for side in SIDES}
        self.stats = {'steps': 0, 'overruns': 0, 'started_wall': time.monotonic(), 'max_lag_s': 0.0}
        self.trace = None
        self._publish(force=True)

    # ---- physics -------------------------------------------------------------------------------------------------

    def _rates(self):
        hand = self.world.sim['hand']
        dt = self.world.dt
        return (max(1, round(1.0 / (dt * float(hand['state_rate_hz'])))),
                max(1, round(1.0 / (dt * float(hand['diagnostics_rate_hz'])))))

    def _read_commands(self):
        w = self.world
        for side in SIDES:
            block = self.blocks[side]['command']
            count = block.count()
            if count != self.last_command[side]:
                self.last_command[side] = count
                _, _, value = block.read()
                try:
                    w.firmware[side].command(w.time, value['q'], value['dq'], value['effort'])
                except FirmwareError as exc:
                    print(f'{side} hand rejected a command: {exc}', flush=True)

    def _publish(self, force=False):
        w = self.world
        state_every, diag_every = self._rates()
        now = time.monotonic_ns()
        for side in SIDES:
            if force or w.steps % state_every == 0:
                q, dq, current = w.hand_measurement(side)
                self.blocks[side]['state'].write(now, {'q': q, 'dq': dq, 'effort': current})
            if force or w.steps % diag_every == 0:
                diag = w.firmware[side].diagnostics()
                self.blocks[side]['diag'].write(now, {
                    'states': diag['states'], 'error': diag['error_code_current'], 'current': diag['current'],
                    'vbus': diag['vbus_v_fb'], 'temperature': diag['mcu_temp_c_fb'],
                    'current_limited': diag['current_limit_active']})

    def physics(self):
        w = self.world
        next_wall = time.monotonic()
        while not self.stop.is_set():
            rtf = float(w.sim['realtime_factor'])
            with w.lock:
                self._read_commands()
                w.step(1)
                self._publish()
                if self.trace is not None:
                    self._trace_sample()
            self.stats['steps'] += 1
            if rtf > 0:
                next_wall += w.dt / rtf
                lag = time.monotonic() - next_wall
                if lag < -0.0003:
                    time.sleep(-lag - 0.0002)
                elif lag > 0.05:
                    self.stats['overruns'] += 1
                    self.stats['max_lag_s'] = max(self.stats['max_lag_s'], lag)
                    next_wall = time.monotonic()
            if self.stats['steps'] % 1000 == 0 and self.parent_pid is not None and os.getppid() != self.parent_pid:
                self.stop.set()

    # ---- firmware-level traces (PRIVILEGED) ------------------------------------------------------------------------

    def _trace_sample(self):
        tr, w = self.trace, self.world
        if w.steps % tr['every']:
            return
        tr['t'].append(w.time)
        for side in SIDES:
            fw = w.firmware[side]
            q, dq, current = w.hand_measurement(side)
            q_ref = fw.q_ref if fw.q_ref is not None else q
            row = tr['hands'][side]
            for key, value in (('q_ref', q_ref), ('q_meas', q), ('dq_meas', dq), ('current', current),
                               ('i_ff', fw.i_ff), ('kp', fw.kp), ('kd', fw.kd), ('dq_ref', fw.dq_ref),
                               ('current_limit', fw.limit), ('temperature', fw.temperature)):
                row[key].append(np.asarray(value, dtype=float).copy())
            arm, ix = tr['arms'][side], w.ix
            arm['q_ref'].append(w.arm[side].q_ref.copy())
            arm['q_meas'].append(w.data.qpos[ix.arm_qpos[side]].copy())
            arm['torque'].append(w.data.actuator_force[ix.arm_act[side]].copy())

    def trace_start(self, directory, rate_hz=1000.0):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        keys = ('q_ref', 'q_meas', 'dq_meas', 'current', 'i_ff', 'kp', 'kd', 'dq_ref', 'current_limit', 'temperature')
        with self.world.lock:
            self.trace = {'directory': directory, 'every': max(1, round(1.0 / (self.world.dt * float(rate_hz)))),
                          't': [], 'hands': {s: {k: [] for k in keys} for s in SIDES},
                          'arms': {s: {'q_ref': [], 'q_meas': [], 'torque': []} for s in SIDES},
                          'started': time.time()}
        return {'directory': str(directory)}

    def trace_stop(self):
        with self.world.lock:
            tr, self.trace = self.trace, None
        if tr is None:
            raise ValueError('no trace is recording')
        from services.dex_eval.trace import default_meta, save_arm, save_hand, write_meta
        from services.wuji.partial import disabled_indices
        t = np.asarray(tr['t'])
        w = self.world
        for side in SIDES:
            arrays = {k: np.asarray(v, dtype=float).reshape(len(t), 20) for k, v in tr['hands'][side].items()}
            # Shared trace contract: disabled joints are NaN in the measured arrays.
            for i in disabled_indices(w.hand_cfg[side]):
                for key in ('q_meas', 'dq_meas', 'current'):
                    arrays[key][:, i] = np.nan
            # The firmware sees only the latched command, not a move's goal: q_target = q_ref
            # here (the robot service's own trace records requested goals).
            save_hand(tr['directory']/f'hand_{side}.npz', t=t, q_ref=arrays['q_ref'], q_target=arrays['q_ref'],
                      q_meas=arrays['q_meas'], dq_meas=arrays['dq_meas'], current=arrays['current'],
                      i_ff=arrays['i_ff'], kp=arrays['kp'], kd=arrays['kd'], dq_ref=arrays['dq_ref'],
                      current_limit_a=arrays['current_limit'], temperature_c=arrays['temperature'])
            save_arm(tr['directory']/f'arm_{side}.npz', t=t,
                     q_ref=np.asarray(tr['arms'][side]['q_ref']).reshape(len(t), 6),
                     q_meas=np.asarray(tr['arms'][side]['q_meas']).reshape(len(t), 6),
                     torque_nm=np.asarray(tr['arms'][side]['torque']).reshape(len(t), 6))
        meta = default_meta(
            sides=list(SIDES), nominal_rate_hz=1.0 / (w.dt * tr['every']), backend='sim',
            controller='sim firmware (MIT law, gain factor, current caps; services/sim_xarm_wuji)',
            started_at=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(tr['started'])),
            notes='firmware-level trace from the MuJoCo twin; q_target equals the latched command q_ref')
        meta.update(trace_source='service:sim_firmware', time_base='simulation seconds', privileged=True,
                    scene=w.sim['scene'], seed=w.sim['seed'], samples=int(len(t)),
                    hand_model={k: w.sim['hand'][k] for k in ('gain_factor', 'static_friction_a',
                                'coulomb_friction_a', 'stribeck_velocity_rad_s', 'command_latency_s')},
                    torque_per_amp={s: w.kt[s].tolist() for s in SIDES},
                    disabled_joints={s: list(w.hand_cfg[s]['disabled_joints']) for s in SIDES})
        write_meta(tr['directory']/'meta.json', meta)
        return {'directory': str(tr['directory']), 'samples': int(len(t))}

    # ---- RPC ------------------------------------------------------------------------------------------------

    def dispatch(self, op, args):
        domain, _, name = op.partition('.')
        handler = getattr(self, f'_{domain}_{name}', None)
        if domain not in ('wuji', 'xarm', 'sim') or handler is None:
            raise ValueError(f'unknown op {op!r}')
        return handler(**args)

    # hands
    def _hand(self, serial):
        if serial not in self.serial_side:
            raise ValueError(f'no simulated hand with serial {serial}; the simulator has {sorted(self.serial_side)}')
        side = self.serial_side[serial]
        return side, self.world.firmware[side]

    def _wuji_connect(self, serial=None, address=None):
        if serial is None:
            raise ValueError('the simulated hands are reached by serial number')
        side, fw = self._hand(serial)
        paths = shm.hand_paths(self.shm_dir, serial)
        hand = self.world.sim['hand']
        return {**fw.info(), 'side': side, 'fingertip_online': [False] * 5,
                'shm': {k: str(v) for k, v in paths.items()},
                'state_rate_hz': float(hand['state_rate_hz']), 'diagnostics_rate_hz': float(hand['diagnostics_rate_hz'])}

    def _wuji_call(self, serial, method, **kwargs):
        side, fw = self._hand(serial)
        with self.world.lock:
            return getattr(fw, method)(**kwargs)

    def _wuji_set_effort_limit(self, serial, value):
        self._wuji_call(serial, 'set_effort_limit', value=value)

    def _wuji_get_effort_limit(self, serial):
        return self._hand(serial)[1].limit.tolist()

    def _wuji_set_mit_params(self, serial, value):
        self._wuji_call(serial, 'set_mit_params', pairs=value)

    def _wuji_get_mit_params(self, serial):
        return self._hand(serial)[1].mit_params()

    def _wuji_enable(self, serial, joints=None):
        self._wuji_call(serial, 'enable', joints=joints)

    def _wuji_disable(self, serial, joints=None):
        self._wuji_call(serial, 'disable', joints=joints)

    def _wuji_clear_fault(self, serial, joints=None):
        self._wuji_call(serial, 'clear_fault', joints=joints)

    def _wuji_emergency_stop(self, serial):
        self._wuji_call(serial, 'emergency_stop')

    def _wuji_set_constant(self, serial, mode, joints=None):
        self._wuji_call(serial, 'set_constant', mode=mode, joints=joints)

    def _wuji_set_position_table(self, serial, on, joints=None):
        self._wuji_call(serial, 'set_position_table', on=on, joints=joints)

    def _wuji_joint(self, serial, index, what, value=None):
        side, fw = self._hand(serial)
        index = int(index)
        if not 0 <= index < 20:
            raise ValueError('joint index is 0..19')
        mask = [int(i == index) for i in range(20)]
        with self.world.lock:
            if what == 'error_code':
                return int(fw.error[index])
            if what == 'status_word':
                diag = fw.diagnostics()
                return {'ext_state': int(diag['states'][index]),
                        'current_limit_active': bool(diag['current_limit_active'][index])}
            if what == 'get_effort_limit':
                return float(fw.limit[index])
            if what == 'set_effort_limit':
                values = fw.limit.copy()
                values[index] = float(value)
                fw.set_effort_limit(values)
                return None
            if what in ('enable', 'disable', 'clear_fault'):
                getattr(fw, what)(mask)
                return None
            if what == 'constant':
                fw.set_constant(value, mask)
                return None
            if what == 'position_table':
                fw.set_position_table(value, mask)
                return None
        raise ValueError(f'unknown joint resource {what!r}')

    def _wuji_describe_error(self, code):
        return describe_error(code)

    def _wuji_command(self, serial, q, dq, effort):
        side, fw = self._hand(serial)
        with self.world.lock:
            fw.command(self.world.time, q, dq, effort)

    def _wuji_mask(self, joints):
        return mask_indices(joints)

    # arms
    def _arm(self, ip):
        for side, address in self.ip_side.items():
            if address == ip:
                return side, self.world.arm[side]
        raise ValueError(f'no simulated xArm at {ip!r}; the simulator has {sorted(self.ip_side.values())}')

    def _xarm_connect(self, ip):
        side, arm = self._arm(ip)
        return {'side': side, 'serial': arm.serial, 'tcp_offset': arm.tcp_offset}

    def _xarm_report(self, ip):
        return self._arm(ip)[1].report()

    def _xarm_set_position(self, ip, pose, speed, mvacc):
        return self._arm(ip)[1].set_position(pose, speed, mvacc)

    def _xarm_set_servo_angle(self, ip, angle, speed, mvacc):
        return self._arm(ip)[1].set_servo_angle(angle, speed, mvacc)

    def _xarm_set_state(self, ip, state):
        return self._arm(ip)[1].set_state(state)

    def _xarm_set_mode(self, ip, mode):
        return self._arm(ip)[1].set_mode(mode)

    def _xarm_motion_enable(self, ip, enable=True):
        return self._arm(ip)[1].motion_enable(enable)

    def _xarm_clean_error(self, ip):
        return self._arm(ip)[1].clean_error()

    def _xarm_clean_warn(self, ip):
        return self._arm(ip)[1].clean_warn()

    def _xarm_emergency_stop(self, ip):
        return self._arm(ip)[1].emergency_stop()

    def _xarm_fk(self, ip, angles):
        return self._arm(ip)[1].forward_kinematics(angles)

    def _xarm_ik(self, ip, pose, reference):
        side, arm = self._arm(ip)
        from scipy.spatial.transform import Rotation
        pose = np.asarray(pose, dtype=float)
        q, ok = arm.kin.ik(pose[:3] / 1000.0, Rotation.from_euler('xyz', pose[3:6]).as_matrix(), reference)
        return {'angles': q.tolist(), 'ok': ok}

    def _xarm_joint_limited(self, ip, angles):
        return self._arm(ip)[1].joint_limited(angles)

    # simulation control and PRIVILEGED state
    def _sim_hello(self):
        w = self.world
        return {'pid': os.getpid(), 'time': w.time, 'scene': w.sim['scene'], 'seed': w.sim['seed'],
                'hands': {side: w.hand_cfg[side]['serial'] for side in SIDES}, 'arms': self.ip_side,
                'shm_dir': str(self.shm_dir), 'model_version': self.model_version}

    def _sim_options(self):
        return {'options': self.options, 'simulation': self.world.sim, 'model_version': self.model_version}

    def _sim_model_xml(self):
        with self.world.lock:
            return {'xml': self.world.spec.to_xml(), 'model_version': self.model_version}

    def _sim_render_state(self):
        return {**self.world.render_state(), 'model_version': self.model_version}

    def _sim_ground_truth(self):
        return self.world.ground_truth()

    def _sim_reset(self, seed=None, power_cycle=False):
        self.world.reset(seed=seed, power_cycle=bool(power_cycle))
        with self.world.lock:
            self._publish(force=True)
        return self.world.scene.draws()

    def _sim_load_scene(self, scene, seed=None, randomize=None):
        with self.world.lock:
            self.world.rebuild(scene=scene, seed=seed, randomize=randomize)
            self.model_version += 1
            self._publish(force=True)
        return {'scene': scene, 'model_version': self.model_version, 'draws': self.world.scene.draws()}

    def _sim_set_object_pose(self, name, position, quaternion_xyzw=None):
        self.world.set_object_pose(name, position, quaternion_xyzw)

    def _sim_set_hand_model(self, side=None, **values):
        """Change hand-model constants live (friction, gain factor, latency, noise)."""
        allowed = {'gain_factor', 'static_friction_a', 'coulomb_friction_a', 'stribeck_velocity_rad_s',
                   'command_latency_s', 'position_noise_rad', 'velocity_noise_rad_s', 'firmware_feedforward_effective'}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f'cannot change {sorted(unknown)} live')
        with self.world.lock:
            self.world.sim['hand'].update(values)
            for s in SIDES:
                fw = self.world.firmware[s]
                fw.gain = float(self.world.sim['hand']['gain_factor'])
                fw.latency = float(self.world.sim['hand']['command_latency_s'])
                fw.opts = self.world.sim['hand']
                fw.dirty = True
        return {k: self.world.sim['hand'][k] for k in allowed}

    def _sim_stats(self):
        elapsed = time.monotonic() - self.stats['started_wall']
        return {**self.stats, 'sim_time_s': self.world.time, 'wall_s': elapsed,
                'realtime_factor_achieved': self.world.time / elapsed if elapsed > 0 else None}

    def _sim_trace_start(self, directory, rate_hz=1000.0):
        return self.trace_start(directory, rate_hz)

    def _sim_trace_stop(self):
        return self.trace_stop()

    # ---- serving ---------------------------------------------------------------------------------------------

    def serve(self):
        server = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                for line in self.rfile:
                    try:
                        request = json.loads(line)
                        value = server.dispatch(request['op'], request.get('args') or {})
                        reply = {'ok': True, 'value': value}
                    except Exception as exc:  # noqa: BLE001 - every failure goes back to the caller
                        reply = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
                    try:
                        self.wfile.write(encode(reply))
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        return

        class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
            daemon_threads = True
            block_on_close = False

        self.socket_path.unlink(missing_ok=True)
        physics = threading.Thread(target=self.physics, name='sim-physics', daemon=True)
        with Server(str(self.socket_path), Handler) as listener:
            os.chmod(self.socket_path, 0o600)
            listener.timeout = 0.2
            physics.start()
            print(f'sim server on {self.socket_path} (scene {self.world.sim["scene"]})', flush=True)
            try:
                while not self.stop.is_set():
                    listener.handle_request()
            finally:
                self.stop.set()
                physics.join(timeout=2)
                self.socket_path.unlink(missing_ok=True)
                for blocks in self.blocks.values():
                    for block in blocks.values():
                        block.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--options', type=Path, required=True, help='bimanual backend options with `simulation`')
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--shm-dir', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int)
    args = parser.parse_args(argv)
    options = json.loads(args.options.read_text())
    # The physics thread shares the interpreter with RPC handlers (IK planning, rendering
    # state): switch threads often so a long handler cannot delay a 1 ms step for 5 ms.
    sys.setswitchinterval(0.0002)
    server = SimServer(options, args.socket, args.shm_dir, parent_pid=args.parent_pid)

    def stop(*_):
        server.stop.set()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    server.serve()


if __name__ == '__main__':
    main()
