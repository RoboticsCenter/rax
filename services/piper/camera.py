"""RGB-only observations from the existing Spyglass owner; no camera SDK connection."""
import base64
import json
from pathlib import Path
import threading
import time
import urllib.request

import cv2 as cv
import numpy as np


def image_content(data):
    return {'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(data).decode()}


class Camera:
    def __init__(self, settings, session, robot_state):
        self.settings, self.session, self.robot_state = settings, Path(session), robot_state
        self.spec = settings['cameras'][0]
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.response = None
        self.thread = threading.Thread(target=self._record, name='piper-rgb', daemon=True)
        self.thread.start()

    def _record(self):
        while not self.stop.is_set():
            try:
                with urllib.request.urlopen(self.spec['url'], timeout=2) as response:
                    self.response = response
                    buffer = b''
                    previous = 0.
                    while not self.stop.is_set():
                        block = response.read(4096)
                        if not block:
                            raise RuntimeError('RGB stream ended')
                        buffer += block
                        if len(buffer) > 4 * 1024 * 1024:
                            raise RuntimeError('Oversized RGB stream frame')
                        while True:
                            start = buffer.find(b'\xff\xd8')
                            end = buffer.find(b'\xff\xd9', max(0, start + 2))
                            if start < 0 or end < 0:
                                break
                            data, buffer = buffer[start:end + 2], buffer[end + 2:]
                            if cv.imdecode(np.frombuffer(data, np.uint8), cv.IMREAD_COLOR) is None:
                                raise RuntimeError('Invalid JPEG from camera owner')
                            now = time.time()
                            with self.lock:
                                self.latest = (now, data)
                                self.error = None
                            if self.settings['stream_enabled'] and now - previous >= 1 / self.settings['stream_fps']:
                                previous = now
                                self._save(now, data)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                self.stop.wait(.5)
            finally:
                self.response = None

    def _save(self, timestamp, data):
        minute = time.strftime('%Y%m%dT%H%M', time.gmtime(timestamp))
        root = self.session / 'stream'
        directory = root / 'overview' / minute
        directory.mkdir(parents=True, exist_ok=True)
        (directory / f'{int(timestamp * 1000)}.jpg').write_bytes(data)
        records = root / 'robot'
        records.mkdir(parents=True, exist_ok=True)
        try:
            record = {'t_ms': int(timestamp * 1000), **self.robot_state()}
        except Exception as exc:
            record = {'t_ms': int(timestamp * 1000), 'error': str(exc)}
        with (records / f'{minute}.jsonl').open('a') as output:
            output.write(json.dumps(record) + '\n')
        cutoff = int((timestamp - self.settings['stream_window_s']) * 1000)
        for folder in (root / 'overview').iterdir():
            for frame in folder.glob('*.jpg'):
                if int(frame.stem) < cutoff:
                    frame.unlink(missing_ok=True)
            if not any(folder.iterdir()):
                folder.rmdir()
        earliest = time.strftime('%Y%m%dT%H%M', time.gmtime(cutoff / 1000))
        for path in records.glob('*.jsonl'):
            if path.stem < earliest:
                path.unlink(missing_ok=True)

    def grab(self, timeout=3.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                latest, error = self.latest, self.error
            if latest and time.time() - latest[0] <= 1. and error is None:
                return latest
            time.sleep(.02)
        raise RuntimeError(f'No fresh overview RGB frame: {error or "camera starting"}')

    def close(self):
        self.stop.set()
        self.thread.join(timeout=4)
