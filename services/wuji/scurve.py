"""Local jerk-limited reference generation; no SDK, sockets, or hardware access."""
import math

from .config import number, target, vector

RUCKIG_VERSION = '0.19.4'


def require_ruckig():
    # Keep workspace/configuration generation independent of native packages.
    try:
        import ruckig
    except ImportError as exc:
        raise RuntimeError(f'Install ruckig=={RUCKIG_VERSION} in the robot environment') from exc
    if ruckig.__version__ != RUCKIG_VERSION:
        raise RuntimeError(f'Install ruckig=={RUCKIG_VERSION}; other versions need qualification')
    return ruckig


class SCurvePath:
    """Advance once per published control tick, without wall-time catch-up jumps.

    The initial state is the preceding commanded q/v/a. Normal moves end at rest;
    a healthy hold brakes to rest with a velocity-interface trajectory. Only the
    local state-to-state API is used, never intermediate/cloud waypoints.
    """
    def __init__(self, start, goal, cfg, duration=None, *, velocity=None, acceleration=None, brake=False):
        ruckig = require_ruckig()
        self.cfg, self.dt, self.brake = cfg, 1/cfg['command_hz'], brake
        self.start = target(start, cfg)
        self.goal = target(goal, cfg)
        self.positions = list(self.start)
        self.velocity = vector(velocity if velocity is not None else [0.]*20, 'reference velocity')
        self.acceleration = vector(acceleration if acceleration is not None else [0.]*20, 'reference acceleration')
        if not brake and max(abs(b-a) for a, b in zip(self.start, self.goal)) > cfg['max_step_rad']:
            raise ValueError(f'hand move exceeds max_step_rad={cfg["max_step_rad"]}')
        inp = ruckig.InputParameter(20)
        inp.current_position, inp.current_velocity, inp.current_acceleration = self.start, self.velocity, self.acceleration
        inp.target_position, inp.target_velocity, inp.target_acceleration = self.goal, [0.]*20, [0.]*20
        inp.max_velocity = [cfg['max_velocity_rad_s']]*20
        inp.max_acceleration = [cfg['max_acceleration_rad_s2']]*20
        inp.max_jerk = [cfg['max_jerk_rad_s3']]*20
        inp.synchronization = ruckig.Synchronization.No if brake else ruckig.Synchronization.Phase
        inp.duration_discretization = ruckig.DurationDiscretization.Discrete
        if brake:
            inp.control_interface = ruckig.ControlInterface.Velocity
        planner = ruckig.Ruckig(20, self.dt)
        self.trajectory = ruckig.Trajectory(20)
        planner.validate_input(inp, True, True)

        def calculate():
            result = planner.calculate(inp, self.trajectory)
            if result not in (ruckig.Result.Working, ruckig.Result.Finished):
                raise RuntimeError(f'S-curve planning failed: {result}')

        calculate()
        minimum = max(0. if brake else .1, self.trajectory.duration)
        if duration is not None:
            duration = number(duration, 'duration_s', .1, cfg['max_duration_s'])
            if duration < minimum-1e-9:
                raise ValueError(f'duration_s must be at least {minimum:.3f} for the configured reference limits')
        requested = minimum if duration is None else duration
        if requested > self.trajectory.duration:
            inp.minimum_duration = requested
            calculate()
        self.duration = self.trajectory.duration
        if self.duration > (cfg['stop_timeout_s'] if brake else cfg['max_duration_s'])+1e-9:
            raise ValueError('S-curve exceeds the configured duration limit')
        for bound, low, high in zip(self.trajectory.position_extrema, cfg['lower_rad'], cfg['upper_rad']):
            if (not math.isfinite(bound.min) or not math.isfinite(bound.max)
                    or bound.min < low-1e-10 or bound.max > high+1e-10):
                raise ValueError('S-curve braking path would cross configured joint limits')
        if brake:
            self.goal = list(self.trajectory.at_time(self.duration)[0])
        self.clock_s = 0.
        self.steps = 0
        self.peak_velocity = self.peak_acceleration = self.peak_jerk = 0.

    @property
    def complete(self):
        return self.clock_s >= self.duration

    def sample_state(self, seconds):
        if seconds >= self.duration:
            return list(self.goal), [0.]*20, [0.]*20
        return tuple(map(list, self.trajectory.at_time(max(0., seconds))))

    def advance(self):
        self.steps += 1
        self.clock_s = min(self.duration, self.steps*self.dt)
        q, v, a = self.sample_state(self.clock_s)
        jerk = [(b-c)/self.dt for b, c in zip(a, self.acceleration)]
        for values, limit in ((v, self.cfg['max_velocity_rad_s']), (a, self.cfg['max_acceleration_rad_s2']),
                              (jerk, self.cfg['max_jerk_rad_s3'])):
            if any(not math.isfinite(x) or abs(x) > limit+1e-7 for x in values):
                raise RuntimeError('S-curve reference derivative exceeded its configured limit')
        target(q, self.cfg)
        self.positions, self.velocity, self.acceleration = q, v, a
        self.peak_velocity = max(self.peak_velocity, *map(abs, v))
        self.peak_acceleration = max(self.peak_acceleration, *map(abs, a))
        self.peak_jerk = max(self.peak_jerk, *map(abs, jerk))
        return list(q), list(v), list(a)

    def status(self):
        return {'phase': 'settling' if self.complete else ('braking' if self.brake else 'approaching'),
                'progress': 1. if self.duration == 0 else self.clock_s/self.duration,
                'nominal_duration_s': self.duration, 'reference_time_s': self.clock_s,
                'requested_positions': list(self.goal), 'generator': 'Ruckig', 'ruckig_version': RUCKIG_VERSION,
                'peak_reference_velocity_rad_s': self.peak_velocity,
                'peak_reference_acceleration_rad_s2': self.peak_acceleration,
                'peak_nominal_reference_jerk_rad_s3': self.peak_jerk}


def next_deadline(previous, now, period):
    """Skip overdue slots rather than publishing a catch-up burst."""
    due = previous+period
    if now >= due:
        return now+period, math.floor((now-due)/period)+1
    return due, 0
