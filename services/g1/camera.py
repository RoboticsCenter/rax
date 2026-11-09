"""RGB snapshots from operator-owned camera bridges; no inferred extrinsics."""
import base64
from collections import deque
import time
import urllib.request


class Cameras:
    def __init__(self, specs):
        self.specs = specs
        self.history = deque(maxlen=32)

    def observe(self):
        frames = []
        for spec in self.specs:
            started = time.time()
            with urllib.request.urlopen(spec['url'], timeout=2.) as response:
                data = response.read(8 * 1024 * 1024 + 1)
            if len(data) > 8 * 1024 * 1024 or not data.startswith(b'\xff\xd8'):
                raise ValueError('Camera bridge must return a bounded JPEG snapshot')
            import cv2
            import numpy as np
            decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if decoded is None:
                raise ValueError('Invalid JPEG snapshot')
            frames.append({'camera': spec['name'], 'received_unix': time.time(),
                           'request_unix': started, 'capture_time_verified': False,
                           'data': data})
        self.history.append((time.time(), frames))
        return frames

    def stream(self, seconds_ago):
        if not self.history:
            raise ValueError('No observations recorded yet')
        target = time.time() - seconds_ago
        if target < self.history[0][0] - .1:
            raise ValueError('Requested frame predates retained observations')
        return min(self.history, key=lambda entry: abs(entry[0] - target))[1]

    @staticmethod
    def images(frames):
        return [{'type': 'image', 'mimeType': 'image/jpeg',
                 'data': base64.b64encode(frame['data']).decode()} for frame in frames]

    def close(self):
        self.history.clear()
