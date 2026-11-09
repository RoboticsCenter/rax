"""Bounded transient tolerance for the measured velocity under current bias."""
import math
from copy import deepcopy

from .config import JOINT_NAMES


class EffortVelocityGuard:
    """Keep the speed bound; optionally defer a small, brief excursion.

    The operator can allow at most 20 ms above the normal bound. Twice that
    speed still trips immediately, as does displacement exceeding normal speed
    times the grace period. Repeated reads of one frame cannot extend the
    deadline. This is not a filter on the velocity sent to the motors.
    """
    def __init__(self, cfg):
        self.limit = cfg['max_velocity_rad_s']
        self.grace = cfg['effort_velocity_grace_s']
        self.hard_limit = self.limit * (2 if self.grace else 1)
        self.excursion_limit = self.limit * self.grace
        self.active = None
        self.last_event = None
        self.last_trip = None
        self.tolerated_events = 0

    def reset(self):
        # Historical evidence survives preparation/stop; no active deadline does.
        self.active = None

    def status(self):
        return {'limit_rad_s': self.limit, 'grace_s': self.grace,
                'hard_limit_rad_s': self.hard_limit,
                'max_grace_excursion_rad': self.excursion_limit,
                'active': deepcopy(self.active), 'last_event': deepcopy(self.last_event),
                'last_trip': deepcopy(self.last_trip), 'tolerated_events': self.tolerated_events}

    def check(self, state, now):
        speeds, positions = state['velocity_rad_s'], state['positions']
        if (len(speeds) != 20 or len(positions) != 20
                or not all(math.isfinite(x) for x in [*speeds, *positions])):
            raise RuntimeError('invalid measured feedback for effort velocity guard')
        joint = max(range(20), key=lambda i: abs(speeds[i]))
        speed = abs(speeds[joint])
        if speed <= self.limit and self.active is None:
            return
        if self.active is None:
            self.active = {'started_monotonic': now, 'start_positions': list(positions),
                           'peak_rad_s': speed, 'peak_joint': JOINT_NAMES[joint]}
        event = self.active
        if speed > event['peak_rad_s']:
            event.update(peak_rad_s=speed, peak_joint=JOINT_NAMES[joint])
        elapsed = max(0., now-event['started_monotonic'])
        excursion = max(abs(a-b) for a, b in zip(positions, event['start_positions']))
        # Check deadline/displacement before accepting a return below the bound;
        # a late low-speed sample cannot erase an unobserved over-limit interval.
        reason = ('immediate' if not self.grace else
                  'hard_limit' if speed >= self.hard_limit else
                  'displacement' if excursion >= self.excursion_limit else
                  'duration' if elapsed >= self.grace-1e-12 else None)
        if reason is not None or speed <= self.limit:
            evidence = {**deepcopy(event), 'outcome': 'tripped' if reason else 'tolerated',
                        'reason': reason, 'elapsed_s': elapsed, 'excursion_rad': excursion,
                        'joint': JOINT_NAMES[joint], 'velocity_rad_s': speeds[joint],
                        'velocities_rad_s': list(speeds), 'positions': list(positions),
                        'source_headers': deepcopy(state.get('source_headers')),
                        'received_monotonic': deepcopy(state.get('received_monotonic')),
                        'limit_rad_s': self.limit, 'hard_limit_rad_s': self.hard_limit,
                        'grace_s': self.grace, 'max_grace_excursion_rad': self.excursion_limit}
            self.last_event = evidence
            self.active = None
            if reason:
                self.last_trip = evidence
                raise RuntimeError('effort-driven measured joint velocity exceeded the configured bound: '
                                   f'{reason}; {JOINT_NAMES[joint]}={speeds[joint]:.6g} rad/s, '
                                   f'peak={event["peak_rad_s"]:.6g} rad/s ({event["peak_joint"]}), '
                                   f'elapsed={elapsed:.6g} s, excursion={excursion:.6g} rad')
            self.tolerated_events += 1
