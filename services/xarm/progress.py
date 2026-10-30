"""Detect an unfinished command that has stopped making measurable movement."""
import numpy as np
from scipy.spatial.transform import Rotation


class MotionProgress:
    IDLE_SECONDS = 2.0
    STATIONARY_SECONDS = 5.0

    def __init__(self):
        self.anchor = None
        self.stationary_since = self.idle_since = None

    def check(self, state, now):
        """Called only outside target tolerance, with validated controller feedback."""
        p = np.asarray(state['tcp_position'], dtype=float)
        q = np.asarray(state['joints'], dtype=float)
        rotation = Rotation.from_quat(state['tcp_quaternion'])
        moved = (self.anchor is None
                 or np.linalg.norm(p-self.anchor[0]) > .0005
                 or np.max(np.abs(q-self.anchor[1])) > .001
                 or (rotation * self.anchor[2].inv()).magnitude() > np.radians(.1))
        if moved:
            self.anchor = (p.copy(), q.copy(), rotation)
            self.stationary_since = now
            self.idle_since = None
        if state['state'] == 2:
            if self.idle_since is None:
                self.idle_since = now
        else:
            self.idle_since = None
        if self.idle_since is not None and now-self.idle_since >= self.IDLE_SECONDS:
            return f'controller idle and stationary outside target tolerance for {self.IDLE_SECONDS:g}s'
        if now-self.stationary_since >= self.STATIONARY_SECONDS:
            return f'no measurable arm movement for {self.STATIONARY_SECONDS:g}s outside target tolerance'
        return None
