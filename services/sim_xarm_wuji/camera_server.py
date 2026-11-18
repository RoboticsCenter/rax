"""The rig's RealSense cameras, rendered by MuJoCo on demand, behind the same socket
protocol as services/xarm/camera_server.py (`grab`, `ping`), so the backend's
CameraService client and `xarm_observe` / `xarm_stream` run unchanged.

Each configured camera is drawn from the sim server's current state at its calibrated
pose with its calibrated K (rig.py, model.py): RGB at the colour resolution (JPEG, as
the real server writes) and metric z-depth aligned to colour (16-bit PNG, mm). Images
are undistorted pinhole renders (distortion_model 'none'). Optional continuous
recording (`--stream`) renders every camera at `--fps` into the same sliding-window
layout the real server writes.

Rendering needs an OpenGL context: MUJOCO_GL=egl (default here) or osmesa.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import threading
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

import cv2 as cv  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from .rpc import Client  # noqa: E402

cv.setNumThreads(1)


def minute_name(t_s):
    return time.strftime('%Y%m%dT%H%M', time.gmtime(t_s))


def write_atomic(path, data):
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_bytes(data)
    os.replace(temporary, path)


def depth_png(metres):
    millimetres = np.where(np.isfinite(metres) & (metres > 0), np.round(metres * 1000), 0)
    ok, png = cv.imencode('.png', np.clip(millimetres, 0, 65535).astype(np.uint16))
    return png.tobytes()


class Renderer:
    """The sim world's model, kept in step with the server, drawn from named cameras."""

    def __init__(self, sim_socket, specs, *, max_depth_m=6.0):
        self.client = Client(sim_socket)
        self.specs = {s['name']: s for s in specs}
        self.max_depth = max_depth_m
        self.lock = threading.Lock()
        self.version = None
        self.renderers = {}
        self._load()

    def _load(self):
        reply = self.client.call('sim.model_xml')
        for renderer in self.renderers.values():
            renderer.close()
        self.model = mujoco.MjModel.from_xml_string(reply['xml'])
        self.data = mujoco.MjData(self.model)
        self.version = reply['model_version']
        self.renderers = {}

    def camera_name(self, name):
        spec = self.specs[name]
        return f"{spec['arm']}_wrist_camera" if spec.get('mount') == 'tcp' else name

    def _sync(self):
        state = self.client.call('sim.render_state')
        if state['model_version'] != self.version:
            self._load()
            state = self.client.call('sim.render_state')
        m, d = self.model, self.data
        d.qpos[:] = state['qpos']
        if m.nmocap:
            d.mocap_pos[:] = state['mocap_pos']
            d.mocap_quat[:] = state['mocap_quat']
        m.geom_rgba[:] = state['geom_rgba']
        m.body_quat[:] = state['body_quat']
        if m.nmat:
            m.mat_emission[:] = state['mat_emission']
            m.mat_rgba[:] = state['mat_rgba']
        mujoco.mj_forward(m, d)
        return state['time']

    def _renderer(self, width, height):
        key = (width, height)
        if key not in self.renderers:
            self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
            self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
            self.renderers[key] = mujoco.Renderer(self.model, height, width)
        return self.renderers[key]

    def grab(self, names, depth):
        """{name: (bgr uint8, metres float32 or None, K, sim_time)} from one consistent state."""
        with self.lock:
            sim_time = self._sync()
            out = {}
            for name in names:
                camera = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name(name))
                if camera < 0:
                    raise ValueError(f'camera {name!r} has no simulated pose (calibration missing)')
                width, height = (int(v) for v in self.model.cam_resolution[camera])
                renderer = self._renderer(width, height)
                renderer.disable_depth_rendering()
                renderer.update_scene(self.data, camera)
                rgb = renderer.render()
                metres = None
                if depth:
                    renderer.enable_depth_rendering()
                    renderer.update_scene(self.data, camera)
                    metres = renderer.render().astype(np.float32)
                    metres[(metres <= 0.1) | (metres > self.max_depth)] = 0.0  # D435 min/max range
                    renderer.disable_depth_rendering()
                out[name] = (cv.cvtColor(rgb, cv.COLOR_RGB2BGR), metres, self._K(camera, width, height), sim_time)
            return out

    def _K(self, camera, width, height):
        m = self.model
        focal = m.cam_intrinsic[camera][:2] / m.cam_sensorsize[camera] * m.cam_resolution[camera]
        principal = m.cam_intrinsic[camera][2:] / m.cam_sensorsize[camera] * m.cam_resolution[camera]
        return [[float(focal[0]), 0.0, float(width / 2 - principal[0])],
                [0.0, float(focal[1]), float(height / 2 - principal[1])], [0.0, 0.0, 1.0]]


class Cameras:
    def __init__(self, renderer, specs, *, frames_root, fps, depth_mode, stream=None, window_s=600, quality=80,
                 depth_hz=1.0):
        self.renderer, self.specs = renderer, {s['name']: s for s in specs}
        self.frames_root = Path(frames_root).resolve()
        self.fps, self.depth_enabled = fps, depth_mode != 'NONE'
        self.stream, self.window_s, self.quality, self.depth_hz = stream, window_s, quality, depth_hz
        self.frames = {name: 0 for name in self.specs}
        self.running = True
        if stream is not None:
            Path(stream).mkdir(parents=True, exist_ok=True)
            threading.Thread(target=self._record, name='sim-stream', daemon=True).start()

    def grab(self, names, depth, out_dir):
        out_dir = Path(out_dir).resolve()
        if out_dir.parent != self.frames_root:
            raise ValueError('out_dir must be a direct child of the frames root')
        for name in names:
            if name not in self.specs:
                raise ValueError(f'unknown camera {name!r}; configured: {sorted(self.specs)}')
        out_dir.mkdir(parents=True, exist_ok=True)
        images = self.renderer.grab(names, depth and self.depth_enabled)
        stamp = time.time_ns()
        result = {}
        for name in names:
            bgr, metres, K, sim_time = images[name]
            self.frames[name] += 1
            rgb_file = out_dir/f'{name}.jpg'
            cv.imwrite(str(rgb_file), bgr, [cv.IMWRITE_JPEG_QUALITY, 95])
            entry = {'rgb': str(rgb_file), 'depth': None, 'K': K, 'width': bgr.shape[1], 'height': bgr.shape[0],
                     'timestamp_ns': stamp, 'distortion_model': 'none', 'distortion_coeffs': [0.0] * 5,
                     'intrinsics_source': 'simulation (calibrated K, undistorted render)',
                     'serial': str(self.specs[name]['serial']), 'fps': self.fps, 'depth_scale_m': 0.001,
                     'depth_alignment': 'color' if self.depth_enabled else None, 'depth_scale_factor': 1.0,
                     'depth_registration_source': 'simulation' if self.depth_enabled else None,
                     'sim_time_s': sim_time}
            if metres is not None:
                depth_file = out_dir/f'{name}_depth.png'
                write_atomic(depth_file, depth_png(metres))
                entry['depth'] = str(depth_file)
            result[name] = entry
        return result

    def _record(self):
        last_depth = 0.0
        period = 1.0 / max(float(self.fps), 0.1)
        while self.running:
            started = time.time()
            want_depth = self.depth_enabled and started - last_depth >= 1.0 / max(self.depth_hz, 1e-3)
            try:
                images = self.renderer.grab(list(self.specs), want_depth)
                for name, (bgr, metres, _, _) in images.items():
                    folder = Path(self.stream)/name/minute_name(started)
                    folder.mkdir(parents=True, exist_ok=True)
                    ms = int(started * 1000)
                    ok, jpeg = cv.imencode('.jpg', bgr, [cv.IMWRITE_JPEG_QUALITY, self.quality])
                    write_atomic(folder/f'{ms}.jpg', jpeg.tobytes())
                    if metres is not None:
                        write_atomic(folder/f'{ms}_depth.png', depth_png(metres))
                if want_depth:
                    last_depth = started
                self._prune(started)
            except Exception as exc:  # noqa: BLE001 - a missed frame must not stop the recorder
                print(f'stream: {type(exc).__name__}: {exc}', flush=True)
            time.sleep(max(0.0, period - (time.time() - started)))

    def _prune(self, now):
        cutoff = minute_name(now - self.window_s - 60)
        for camera in Path(self.stream).iterdir():
            if camera.is_dir():
                for minute in camera.iterdir():
                    if minute.is_dir() and minute.name < cutoff:
                        shutil.rmtree(minute, ignore_errors=True)


def serve(listener, cameras):
    while True:
        connection, _ = listener.accept()
        with connection, connection.makefile('r') as reader, connection.makefile('w') as writer:
            for line in reader:
                try:
                    request = json.loads(line)
                    op = request.get('op')
                    if op == 'grab':
                        value = {'cameras': cameras.grab(request['cameras'], bool(request.get('depth', True)),
                                                         request['out_dir'])}
                    elif op == 'ping':
                        value = {'open': sorted(cameras.specs), 'pid': os.getpid(), 'simulated': True,
                                 'frames': dict(cameras.frames), 'dropped': {n: 0 for n in cameras.specs}}
                    else:
                        raise ValueError(f'unknown op {op!r}')
                    response = {'ok': True, **value}
                except Exception as exc:  # noqa: BLE001
                    response = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
                writer.write(json.dumps(response) + '\n')
                writer.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--socket', required=True)
    parser.add_argument('--sim-socket', required=True)
    parser.add_argument('--cameras', required=True, help='JSON list of camera specs')
    parser.add_argument('--depth-mode', default='Z16')
    parser.add_argument('--fps', type=int, default=6)
    parser.add_argument('--frames-root', required=True)
    parser.add_argument('--stream')
    parser.add_argument('--window-seconds', type=float, default=600.0)
    parser.add_argument('--stream-quality', type=int, default=80)
    parser.add_argument('--depth-hz', type=float, default=1.0)
    args = parser.parse_args()

    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)
    specs = json.loads(args.cameras)
    renderer = Renderer(args.sim_socket, specs)
    cameras = Cameras(renderer, specs, frames_root=args.frames_root, fps=args.fps, depth_mode=args.depth_mode,
                      stream=args.stream, window_s=args.window_seconds, quality=args.stream_quality,
                      depth_hz=args.depth_hz)
    path = Path(args.socket)
    path.unlink(missing_ok=True)
    try:
        with socket.socket(socket.AF_UNIX) as listener:
            listener.bind(str(path))
            os.chmod(path, 0o600)
            listener.listen(4)
            print(f'sim camera server on {path}', flush=True)
            serve(listener, cameras)
    finally:
        cameras.running = False
        path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
