"""Feedback-paced straight joint paths; no SDK or hardware dependencies."""
import math
from collections import deque


def blend(u):
    return u**3 * (10 + u*(-15 + 6*u))


def blend_integral(u):
    return 2.5*u**4 - 3*u**5 + u**6


def smooth_minimum_duration(distance, cfg):
    """Conservative chain-rule bounds, including smooth changes of path speed.

    The path clock runs at a scale in [0,1]. Scale transitions are quintics of
    length tau, joined at zero first/second derivative. Worst-case bounds cover
    a full 0↔1 transition, so feedback cannot invalidate the reference limits.
    """
    tau = cfg['feedback_transition_s']
    def fits(t):
        velocity = 1.875*distance/t
        acceleration = distance*(5.773503/t**2 + 1.875**2/(tau*t))
        jerk = distance*(60/t**3 + 3*5.773503*1.875/(tau*t**2)
                         + 1.875*5.773503/(tau**2*t))
        return (velocity <= cfg['max_velocity_rad_s'] and
                acceleration <= cfg['max_acceleration_rad_s2'] and
                jerk <= cfg['max_jerk_rad_s3'])
    lo, hi = .1, .1
    while not fits(hi):
        hi *= 2
    for _ in range(50):
        mid = (lo+hi)/2
        if fits(mid):
            hi = mid
        else:
            lo = mid
    return hi


class FeedbackPath:
    """Retime one existing path without changing its geometry or overshooting.

    Every scale transition completes before another begins. Feedback changes
    pacing, never the path endpoint, stiffness, current cap or a target bias.
    """
    def __init__(self, trajectory, cfg, now):
        self.path, self.cfg = trajectory, cfg
        self.clock_s = 0.
        self.last = self.transition_started = now
        self.scale_from = self.scale_to = 1.
        self.scale = self.minimum_scale = 1.
        self.positions = list(trajectory.start)
        self.velocity = [0.]*20
        self.max_reference_error_rad = 0.

    @property
    def complete(self):
        return self.clock_s >= self.path.duration

    def advance(self, now, measured):
        if now < self.last:
            raise ValueError('feedback path clock regressed')
        tau = self.cfg['feedback_transition_s']
        cursor = self.last
        while cursor < now:
            boundary = self.transition_started+tau
            end = min(now, boundary)
            a = (cursor-self.transition_started)/tau
            b = (end-self.transition_started)/tau
            integral = ((end-cursor)*self.scale_from + tau*(self.scale_to-self.scale_from)
                        * (blend_integral(b)-blend_integral(a)))
            self.clock_s = min(self.path.duration, self.clock_s+max(0., integral))
            cursor = end
            if cursor >= boundary:
                reference = self.path.sample(self.clock_s)
                error = max(abs(q-m) for q, m in zip(reference, measured))
                slow, pause = self.cfg['feedback_slow_error_rad'], self.cfg['feedback_pause_error_rad']
                self.scale_from = self.scale_to
                self.scale_to = min(1., max(0., (pause-error)/(pause-slow)))
                self.transition_started = boundary
        self.last = now
        u = min(1., max(0., (now-self.transition_started)/tau))
        self.scale = self.scale_from+(self.scale_to-self.scale_from)*blend(u)
        self.minimum_scale = min(self.minimum_scale, self.scale)
        self.positions = self.path.sample(self.clock_s)
        phase = min(1., self.clock_s/self.path.duration)
        rate = 30*phase**2*(1-phase)**2/self.path.duration*self.scale
        self.velocity = [d*rate for d in self.path.delta]
        self.max_reference_error_rad = max(self.max_reference_error_rad,
                                          max(abs(q-m) for q, m in zip(self.positions, measured)))
        return list(self.positions), list(self.velocity)

    def status(self):
        return {'phase': 'settling' if self.complete else ('catching_up' if self.scale < .999 else 'approaching'),
                'progress': self.clock_s/self.path.duration, 'speed_scale': self.scale,
                'minimum_speed_scale': self.minimum_scale,
                'nominal_duration_s': self.path.duration,
                'max_reference_error_rad': self.max_reference_error_rad,
                'requested_positions': list(self.path.goal)}


class SettlingWindow:
    """Require fresh, in-tolerance positions with low motion over a full window."""
    def __init__(self, goal, tolerance, span, position_range):
        self.goal, self.tolerance = tuple(goal), tolerance
        self.span, self.position_range = span, position_range
        self.rows = deque()
        self.last = -math.inf

    def update(self, received, positions):
        if received <= self.last:
            return False
        self.last = received
        if max(abs(a-b) for a, b in zip(self.goal, positions)) > self.tolerance:
            self.rows.clear()
            return False
        self.rows.append((received, tuple(positions)))
        while len(self.rows) > 1 and received-self.rows[1][0] >= self.span:
            self.rows.popleft()
        if len(self.rows) < 2 or received-self.rows[0][0] < self.span:
            return False
        return all(max(q[j] for _, q in self.rows)-min(q[j] for _, q in self.rows)
                   <= self.position_range for j in range(20))
