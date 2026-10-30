"""Intel RealSense capture service for xArm sessions.

Each D435i streams continuously into a sliding window the agent can read but not write:

    <stream>/<camera>/<YYYYmmddTHHMM>/<epoch_ms>.jpg
    <stream>/<camera>/<YYYYmmddTHHMM>/<epoch_ms>_depth.png
    <stream>/index.json

Protocol: one JSON object per line over a private Unix socket.
"""
import argparse
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import socket
import threading
import time

import cv2 as cv
import numpy as np

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


class CameraWorker:
    def __init__(self, rs, spec, *, fps, depth_mode, stream, stream_quality, depth_hz):
        self.rs, self.spec, self.name = rs, spec, spec['name']
        self.stream, self.quality, self.depth_hz = stream, stream_quality, depth_hz
        self.depth_enabled = depth_mode != 'NONE'
        self.calibration_native = bool(spec.get('calibration_native', False))
        self.native_frames = {}
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(str(spec['serial']))
        config.enable_stream(rs.stream.color, int(spec.get('width', 1280)), int(spec.get('height', 720)),
                             rs.format.bgr8, fps)
        if self.depth_enabled or self.calibration_native:
            config.enable_stream(rs.stream.depth, 640 if self.calibration_native else int(spec.get('width', 1280)),
                                 480 if self.calibration_native else int(spec.get('height', 720)),
                                 rs.format.z16, fps)
        if self.calibration_native:
            config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, fps)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color) if self.depth_enabled else None
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        if self.calibration_native:
            native_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            ni = native_profile.get_intrinsics()
            ext = native_profile.get_extrinsics_to(color_profile)
            self.native_metadata = {
                'K': [[ni.fx, 0, ni.ppx], [0, ni.fy, ni.ppy], [0, 0, 1]],
                'distortion_coeffs': list(ni.coeffs), 'width': ni.width, 'height': ni.height,
                'depth_to_color_rotation_column_major': list(ext.rotation),
                'depth_to_color_translation_m': list(ext.translation)}
        intr = color_profile.get_intrinsics()
        self.K = [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]]
        self.distortion_model = str(intr.model).rsplit('.', 1)[-1]
        self.distortion_coeffs = list(intr.coeffs)
        from .calibration import (validate_intrinsics, ColorRectification, DepthRegistration,
                                  depth_to_color_transform, calibrated_depth_scale, aligned_depth_scale)
        self.depth_registration = None
        self.depth_scale_factor = 1.0
        self.depth_registration_source = 'realsense_factory' if self.depth_enabled else None
        try:
            live = {'K': self.K, 'width': intr.width, 'height': intr.height,
                    'fps': color_profile.fps(), 'distortion_model': self.distortion_model,
                    'distortion_coeffs': self.distortion_coeffs}
            reference = validate_intrinsics(spec, live)
            if self.depth_enabled:
                self.depth_scale_factor = aligned_depth_scale(
                    reference, profile.get_device().first_depth_sensor().get_depth_scale(), live)
            # New calibration collections need raw pixels even after a fitted
            # model has been installed. Still validate the factory reference.
            self.rectification = ColorRectification(
                None if spec.get('calibration_raw_color') else reference, live)
            self.K = self.rectification.K
            if self.rectification.maps is not None:
                self.distortion_model, self.distortion_coeffs = 'none', [0.]*5
                if self.depth_enabled:
                    native = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                    di = native.get_intrinsics()
                    ext = native.get_extrinsics_to(color_profile)
                    depth_K = [[di.fx, 0, di.ppx], [0, di.fy, di.ppy], [0, 0, 1]]
                    rotation, translation, self.depth_registration_source = depth_to_color_transform(
                        reference, depth_K, (di.height, di.width), di.coeffs,
                        np.asarray(ext.rotation).reshape(3, 3, order='F'), ext.translation)
                    self.depth_scale_factor = calibrated_depth_scale(
                        reference, profile.get_device().first_depth_sensor().get_depth_scale())
                    self.depth_registration = DepthRegistration(
                        depth_K, (di.height, di.width), di.coeffs, self.K,
                        rotation, translation, (intr.height, intr.width),
                        depth_scale_factor=self.depth_scale_factor)
                    self.align = None
        except BaseException:
            self.pipeline.stop()
            raise
        self.depth_scale = None
        if self.depth_enabled or self.calibration_native:
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.lock = threading.Condition()
        self.latest = None
        self.latest_depth = None
        self.depth_wanted = 0
        self.frames = self.dropped = self.errors = 0
        self.last_depth_s = 0.0
        self.running = True
        self.records = queue.Queue(maxsize=int(fps * 2))
        threading.Thread(target=self._capture, name=f'cap-{self.name}', daemon=True).start()
        if stream is not None:
            threading.Thread(target=self._record, name=f'rec-{self.name}', daemon=True).start()

    def _capture(self):
        while self.running:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=2000)
                stamp = time.time_ns()
                native = None
                if self.calibration_native:
                    ir, raw_depth = frames.get_infrared_frame(1), frames.get_depth_frame()
                    if not ir or not raw_depth:
                        continue
                    native = (np.asanyarray(ir.get_data()).copy(),
                              np.asanyarray(raw_depth.get_data()).copy())
                if self.align is not None:
                    frames = self.align.process(frames)
                color = frames.get_color_frame()
                if not color:
                    self.errors += 1
                    continue
                bgr = np.asanyarray(color.get_data()).copy()
                depth = None
                now = time.time()
                want_depth = self.depth_enabled and (
                    self.depth_wanted > 0 or (self.depth_hz > 0 and now - self.last_depth_s >= 1.0 / self.depth_hz))
                if want_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * float(self.depth_scale)
                        if self.depth_registration is not None:
                            depth = self.depth_registration.apply(depth)
                        else:
                            depth *= self.depth_scale_factor
                        depth_min = float(self.spec.get('depth_min_m', 0.0))
                        depth_max = float(self.spec.get('depth_max_m', 10.0))
                        depth[(depth < depth_min) | (depth > depth_max)] = 0.0
                        self.last_depth_s = now
                bgr = self.rectification.apply(bgr)
                with self.lock:
                    if native is not None:
                        self.native_frames[stamp] = native
                        while len(self.native_frames) > 30:
                            del self.native_frames[next(iter(self.native_frames))]
                    self.latest = (stamp, bgr)
                    if depth is not None:
                        self.latest_depth = (stamp, depth, bgr)
                    self.frames += 1
                    self.lock.notify_all()
                if self.stream is not None:
                    try:
                        self.records.put_nowait((stamp, bgr, depth))
                    except queue.Full:
                        self.dropped += 1
            except Exception:
                self.errors += 1
                time.sleep(0.05)

    def _record(self):
        root = self.stream/self.name
        while True:
            stamp, bgr, depth = self.records.get()
            folder = root/minute_name(stamp / 1e9)
            folder.mkdir(parents=True, exist_ok=True)
            ms = stamp // 1_000_000
            ok, jpeg = cv.imencode('.jpg', bgr, [cv.IMWRITE_JPEG_QUALITY, self.quality])
            write_atomic(folder/f'{ms}.jpg', jpeg.tobytes())
            if depth is not None:
                write_atomic(folder/f'{ms}_depth.png', depth_png(depth))

    def newest(self, depth, after_ns, timeout=3.0):
        with self.lock:
            if depth and self.depth_enabled:
                self.depth_wanted += 1
                try:
                    if not self.lock.wait_for(lambda: self.latest_depth and self.latest_depth[0] > after_ns, timeout):
                        raise RuntimeError(f'camera {self.name}: no depth frame within {timeout}s')
                    stamp, metres, bgr = self.latest_depth
                    return stamp, bgr, metres
                finally:
                    self.depth_wanted -= 1
            if not self.lock.wait_for(lambda: self.latest and self.latest[0] > after_ns, timeout):
                raise RuntimeError(f'camera {self.name}: no frame within {timeout}s')
            stamp, bgr = self.latest
            return stamp, bgr, None

    def close(self):
        self.running = False
        time.sleep(0.1)
        self.pipeline.stop()
        deadline = time.monotonic() + 10
        while not self.records.empty() and time.monotonic() < deadline:
            time.sleep(0.05)


class Cameras:
    def __init__(self, specs, *, depth_mode, fps, frames_root, stream, window_s, stream_quality, depth_hz,
                 archive=None):
        import pyrealsense2 as rs
        self.frames_root = Path(frames_root).resolve()
        self.stream = Path(stream).resolve() if stream else None
        self.window_s, self.fps, self.depth_hz = window_s, fps, depth_hz
        self.workers = {}
        self.archiver = None
        try:
            for spec in specs:
                self.workers[spec['name']] = CameraWorker(rs, spec, fps=fps, depth_mode=depth_mode,
                                                          stream=self.stream, stream_quality=stream_quality,
                                                          depth_hz=depth_hz)
        except BaseException:
            self.close()
            raise
        if self.stream is not None and archive:
            from .archive import Archiver
            self.archiver = Archiver(self.stream, archive, list(self.workers), fps=fps,
                                     log=lambda message: print(message, flush=True))
        if self.stream is not None:
            threading.Thread(target=self._maintain, name='stream-maintenance', daemon=True).start()

    def prune(self, now):
        oldest_kept = minute_name(now - self.window_s)
        last_resort = minute_name(now - self.window_s - 300)
        archive = getattr(self, 'archiver', None)
        for entry in self.stream.iterdir():
            if not entry.is_dir():
                continue
            for minute in entry.iterdir():
                name = minute.stem if minute.suffix == '.jsonl' else minute.name
                if name >= oldest_kept or not (minute.is_dir() or minute.suffix == '.jsonl'):
                    continue
                kept = archive is not None and name >= last_resort and not (
                    archive.archived(entry.name, name) if minute.is_dir()
                    else (archive.archive/'robot'/minute.name).exists())
                if kept:
                    continue
                if minute.is_dir():
                    shutil.rmtree(minute, ignore_errors=True)
                else:
                    minute.unlink(missing_ok=True)

    def _maintain(self):
        previous = {}
        while True:
            now = time.time()
            try:
                self.prune(now)
                cameras = {}
                for name, worker in self.workers.items():
                    frames = worker.frames
                    rate = frames - previous.get(name, frames)
                    previous[name] = frames
                    folder = self.stream/name
                    minutes = sorted(p.name for p in folder.iterdir() if p.is_dir()) if folder.is_dir() else []
                    latest = worker.latest[0] if worker.latest else None
                    cameras[name] = {'latest_ms': latest // 1_000_000 if latest else None,
                                     'oldest_minute': minutes[0] if minutes else None, 'fps_measured': rate,
                                     'dropped_frames': worker.dropped, 'grab_errors': worker.errors}
                index = {'updated_ms': int(now * 1000), 'window_s': self.window_s, 'fps': self.fps,
                         'depth_hz': self.depth_hz,
                         'layout': '<camera>/<UTC YYYYmmddTHHMM>/<epoch_ms>.jpg and <epoch_ms>_depth.png '
                                   '(uint16 mm, 0 = invalid); robot/<UTC YYYYmmddTHHMM>.jsonl',
                         'cameras': cameras,
                         'archive': self.archiver.status() if self.archiver else None}
                write_atomic(self.stream/'index.json', json.dumps(index, indent=1).encode())
            except OSError as exc:
                print(f'stream maintenance: {exc}', flush=True)
            time.sleep(max(0.0, 1.0 - (time.time() - now)))

    def grab(self, names, depth, out_dir):
        out_dir = Path(out_dir).resolve()
        if out_dir.parent != self.frames_root:
            raise ValueError('out_dir must be a direct child of the frames root')
        for name in names:
            if name not in self.workers:
                raise ValueError(f'unknown camera {name!r}; configured: {sorted(self.workers)}')
        out_dir.mkdir(parents=True, exist_ok=True)
        after = time.time_ns() - 50_000_000
        result = {}
        for name in names:
            worker = self.workers[name]
            stamp, bgr, metres = worker.newest(depth, after)
            rgb_file = out_dir/f'{name}.jpg'
            cv.imwrite(str(rgb_file), bgr, [cv.IMWRITE_JPEG_QUALITY, 95])
            entry = {'rgb': str(rgb_file), 'depth': None, 'K': worker.K, 'width': bgr.shape[1],
                     'height': bgr.shape[0], 'timestamp_ns': stamp,
                     'distortion_model': worker.distortion_model,
                     'distortion_coeffs': worker.distortion_coeffs,
                     'intrinsics_source': worker.rectification.source,
                     'serial': str(worker.spec['serial']), 'fps': self.fps,
                     'depth_scale_m': worker.depth_scale, 'depth_alignment': 'color' if worker.depth_enabled else None,
                     'depth_scale_factor': getattr(worker, 'depth_scale_factor', 1.0),
                     'depth_registration_source': worker.depth_registration_source}
            if metres is not None:
                depth_file = out_dir/f'{name}_depth.png'
                write_atomic(depth_file, depth_png(metres))
                entry['depth'] = str(depth_file)
            if worker.calibration_native:
                with worker.lock:
                    infrared, native_depth = worker.native_frames[stamp]
                ir_file, native_file = out_dir/f'{name}_infrared.png', out_dir/f'{name}_native_depth.png'
                cv.imwrite(str(ir_file), infrared)
                cv.imwrite(str(native_file), native_depth)
                entry['native_calibration'] = {
                    **worker.native_metadata, 'infrared': str(ir_file), 'depth': str(native_file),
                    'depth_scale_m': worker.depth_scale, 'timestamp_ns': stamp}
            result[name] = entry
        return result

    def close(self):
        for worker in self.workers.values():
            worker.close()
        if self.archiver is not None:
            self.archiver.flush()


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
                        value = {'open': sorted(cameras.workers), 'pid': os.getpid(),
                                 'affinity': sorted(os.sched_getaffinity(0)),
                                 'frames': {n: w.frames for n, w in cameras.workers.items()},
                                 'dropped': {n: w.dropped for n, w in cameras.workers.items()}}
                    else:
                        raise ValueError(f'unknown op {op!r}')
                    response = {'ok': True, **value}
                except Exception as exc:
                    response = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
                writer.write(json.dumps(response) + '\n')
                writer.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', required=True)
    parser.add_argument('--cameras', required=True, help='JSON list of camera specs')
    parser.add_argument('--depth-mode', default='Z16')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--frames-root', required=True)
    parser.add_argument('--stream', help='sliding-window recording directory (omit to disable)')
    parser.add_argument('--window-seconds', type=float, default=600.0)
    parser.add_argument('--stream-quality', type=int, default=80)
    parser.add_argument('--depth-hz', type=float, default=1.0)
    parser.add_argument('--archive', help='whole-session MP4 archive directory (omit to disable)')
    args = parser.parse_args()

    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)
    import ctypes
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)
    if os.getppid() == 1:
        raise SystemExit('parent exited before the camera server started')
    if args.stream:
        Path(args.stream).mkdir(parents=True, exist_ok=True)
    cameras = Cameras(json.loads(args.cameras), depth_mode=args.depth_mode, fps=args.fps,
                      frames_root=args.frames_root, stream=args.stream, window_s=args.window_seconds,
                      stream_quality=args.stream_quality, depth_hz=args.depth_hz, archive=args.archive)
    path = Path(args.socket)
    path.unlink(missing_ok=True)
    try:
        with socket.socket(socket.AF_UNIX) as listener:
            listener.bind(str(path))
            os.chmod(path, 0o600)
            listener.listen(4)
            print(f'camera server on {path}', flush=True)
            serve(listener, cameras)
    finally:
        cameras.close()
        path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
