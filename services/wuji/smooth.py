"""Smooth streaming hand references. Pure host logic: no SDK, sockets or hardware.

Pipeline, evaluated once per writer tick while the hand is holding:

    caller targets ──► goal g(t) ──► optional first-order filter ──► online Ruckig tracker
    (stream/trajectory)  (ramp, min-jerk   (filter_tau_s)               (v/a/j limits, rest target,
                          or PCHIP spline)                               finger/hand/joint sync)
                    ──► joint-limit clamp ──► q_ref, dq_ref ──► + hold-deadband offset (position)
                                                             ──► + friction feed-forward (current)

The tracker always plans toward the *current* goal with zero target velocity. Because the
goal itself moves continuously (interpolated between targets), the reference never stops
between targets, and if targets stop arriving it comes to rest at the last target without
overshooting it. All limits are per joint and never exceed the operator profile.
"""
import math

from .config import JOINT_NAMES, number, vector
from .partial import active_indices, disabled_indices

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
SYNCHRONIZATION = ('finger', 'hand', 'joint')
SHAPES = ('linear', 'minjerk')

# Impedance presets in *effective* units, i.e. what the firmware applies. Telemetry fits
# (docs/wuji-parameter-sweep-2026-10-27.md) show the hand applies about gain_scale (1.98)
# times the documented MIT law, so nominal gains sent = effective / gain_scale, then
# clamped per joint to the operator profile ceilings (cfg kp/kd). 'profile' is the
# operator profile itself.
PRESETS = {
    'soft': {'stiffness_a_per_rad': 6.0, 'damping_a_s_per_rad': .12, 'hold_deadband_rad': 0.},
    'default': {'stiffness_a_per_rad': 10.0, 'damping_a_s_per_rad': .20, 'hold_deadband_rad': 0.},
    'firm': {'stiffness_a_per_rad': 16.0, 'damping_a_s_per_rad': .30, 'hold_deadband_rad': 0.},
    'hold': {'stiffness_a_per_rad': 8.0, 'damping_a_s_per_rad': .12, 'hold_deadband_rad': .004},
}
PROFILE_NAMES = ('profile', *PRESETS)

# Runtime stream settings and their defaults (None = derived from the operator profile).
SETTINGS = {
    'interpolation_s': None,        # None: auto (observed target period, 10..100 ms); 0: none
    'filter_tau_s': 0.,             # first-order low-pass on the goal; 0: off
    'synchronization': 'finger',    # Ruckig time-synchronization groups
    'max_velocity_rad_s': None,     # per-joint reference limits, <= profile ceilings
    'max_acceleration_rad_s2': None,
    'max_jerk_rad_s3': None,
    'friction': False,              # tanh friction feed-forward on/off
    'friction_current_a': None,     # per-joint magnitude (A, commanded); None: profile default
    'friction_velocity_rad_s': None,  # tanh velocity scale; None: profile default
    'hold_deadband_rad': 0.,        # host hold deadband; 0: off
    'hold_delay_s': .2,             # rest time before the deadband engages
    'lookahead': True,              # aim along the known goal by half the braking time
    'command_hz': None,             # writer rate, <= profile command_hz; None: profile value
}
AUTO_INTERPOLATION = (.01, .15, .05)  # min, max, one-off target ramp (s)
# Auto ramps span 1.5x the observed target period: the next target always arrives before
# the ramp ends, so the goal keeps a constant velocity (no pause) under arrival jitter.
AUTO_PERIOD_FACTOR = 1.5
STREAM_GAP_S = .25                    # targets further apart than this are one-off
HOLD_BLEND_S, RELEASE_BLEND_S = .2, .05
MEASURED_FILTER_S = .02               # position filter used only by the hold deadband
MAX_WAYPOINTS = 256


def stream_ceilings(cfg):
    """Operator maxima for the stream tracker (per joint lists)."""
    acc = cfg.get('stream_max_acceleration_rad_s2') or cfg['max_acceleration_rad_s2']
    jerk = cfg.get('stream_max_jerk_rad_s3') or cfg['max_jerk_rad_s3']
    return {'max_velocity_rad_s': [cfg['max_velocity_rad_s']]*20,
            'max_acceleration_rad_s2': [float(acc)]*20, 'max_jerk_rad_s3': [float(jerk)]*20}


def per_joint(value, name, low, high):
    values = value if isinstance(value, (list, tuple)) else [value]*20
    return [number(v, name, low, high) for v in vector(values, name)]


def resolve_settings(update, cfg, current=None):
    """Validated, fully-resolved runtime settings. Unknown keys and values beyond the
    operator profile are rejected rather than clipped."""
    if not isinstance(update, dict):
        raise ValueError('stream settings must be an object')
    unknown = set(update) - set(SETTINGS)
    if unknown:
        raise ValueError(f'unknown stream settings {sorted(unknown)}')
    base = dict(current) if current is not None else {}
    if current is None:
        ceilings = stream_ceilings(cfg)
        base.update(SETTINGS)
        base.update({k: list(v) for k, v in ceilings.items()})
        base['friction_current_a'] = list(cfg['friction_current_a'])
        base['friction_velocity_rad_s'] = cfg['friction_velocity_rad_s']
        base['command_hz'] = cfg['command_hz']
    ceilings = stream_ceilings(cfg)
    result = dict(base)
    for key, value in update.items():
        if key == 'interpolation_s':
            result[key] = None if value is None else number(value, key, 0., 1.)
        elif key == 'filter_tau_s':
            result[key] = number(value, key, 0., 2.)
        elif key == 'synchronization':
            if value not in SYNCHRONIZATION:
                raise ValueError(f'synchronization must be one of {SYNCHRONIZATION}')
            result[key] = value
        elif key in ceilings:
            if value is None:
                result[key] = list(ceilings[key])
                continue
            values = per_joint(value, key, 1e-3, math.inf)
            for name, v, limit in zip(JOINT_NAMES, values, ceilings[key]):
                if v > limit + 1e-12:
                    raise ValueError(f'{name} {key} {v:g} exceeds the operator profile ceiling {limit:g}')
            result[key] = values
        elif key in ('friction', 'lookahead'):
            if type(value) is not bool:
                raise ValueError(f'{key} must be true or false')
            result[key] = value
        elif key == 'friction_current_a':
            result[key] = list(cfg['friction_current_a']) if value is None else per_joint(value, key, 0., .3)
        elif key == 'friction_velocity_rad_s':
            result[key] = cfg['friction_velocity_rad_s'] if value is None else number(value, key, .001, 1.)
        elif key == 'hold_deadband_rad':
            result[key] = number(value, key, 0., .02)
        elif key == 'hold_delay_s':
            result[key] = number(value, key, 0., 5.)
        elif key == 'command_hz':
            if value is None:
                result[key] = cfg['command_hz']
                continue
            hz = number(value, key, 20., cfg['command_hz'])
            if cfg['max_scheduler_gap_s'] <= 1/hz:
                raise ValueError('command_hz period must stay below max_scheduler_gap_s')
            result[key] = hz
    return result


def public_settings(settings):
    """Compact settings for status (scalars where all joints agree)."""
    def compact(values):
        return values[0] if isinstance(values, list) and len(set(values)) == 1 else values
    return {key: compact(value) for key, value in settings.items()}


# ---------------------------------------------------------------- target parsing

def parse_targets(args, cfg, base, *, allow_duration=True):
    """Merge positions/joints/fingers (optionally relative) into a full 20-joint target.

    Returns (indices, merged): the joints this command sets and the merged absolute target.
    Disabled joints must stay at their base value; every set joint stays within limits.
    """
    if not isinstance(args, dict):
        raise ValueError('target must be an object')
    given = [key for key in ('positions', 'joints', 'fingers') if key in args]
    if len(given) != 1:
        raise ValueError('give exactly one of positions (20 rad), joints {name: rad} or fingers {finger: [4 rad]}')
    relative = args.get('relative', False)
    if type(relative) is not bool:
        raise ValueError('relative must be true or false')
    values = {}
    if 'positions' in args:
        values = dict(enumerate(vector(args['positions'])))
    elif 'joints' in args:
        joints = args['joints']
        if not isinstance(joints, dict) or not joints:
            raise ValueError('joints must map joint names (e.g. index_S2) to radians')
        for name, value in joints.items():
            if name not in JOINT_NAMES:
                raise ValueError(f'unknown joint {name!r}; names are {FINGERS} with _S1.._S4')
            values[JOINT_NAMES.index(name)] = number(value, name, -100., 100.)
    else:
        fingers = args['fingers']
        if not isinstance(fingers, dict) or not fingers:
            raise ValueError('fingers must map finger names to four radians each')
        for finger, joints in fingers.items():
            if finger not in FINGERS:
                raise ValueError(f'unknown finger {finger!r}; use {FINGERS}')
            if not isinstance(joints, (list, tuple)) or len(joints) != 4:
                raise ValueError(f'{finger} needs four values (S1..S4)')
            for k, value in enumerate(joints):
                values[FINGERS.index(finger)*4+k] = number(value, f'{finger}_S{k+1}', -100., 100.)
    merged = list(base)
    excluded = set(disabled_indices(cfg))
    for i, value in values.items():
        q = base[i]+value if relative else value
        if i in excluded:
            if abs(q-base[i]) > 1e-9:
                raise ValueError(f'{JOINT_NAMES[i]} is disabled; keep its value unchanged or omit it')
            continue
        if not cfg['lower_rad'][i] <= q <= cfg['upper_rad'][i]:
            raise ValueError(f'{JOINT_NAMES[i]} target {q:g} outside configured limits '
                             f'{cfg["lower_rad"][i]:g}..{cfg["upper_rad"][i]:g} rad')
        merged[i] = q
    indices = tuple(sorted(i for i in values if i not in excluded))
    if max((abs(merged[i]-base[i]) for i in indices), default=0.) > cfg['max_step_rad']:
        raise ValueError(f'target moves more than max_step_rad={cfg["max_step_rad"]} from the current target')
    return indices, merged


def parse_stream(args, cfg, base):
    """Validate a wuji_stream payload; returns a command for StreamEngine.apply()."""
    allowed = {'positions', 'joints', 'fingers', 'relative', 'duration_s', 'shape', 'dry_run'}
    if not isinstance(args, dict) or set(args) - allowed:
        raise ValueError(f'unknown stream arguments {sorted(set(args or {}) - allowed)}')
    indices, merged = parse_targets(args, cfg, base)
    duration = args.get('duration_s')
    if duration is not None:
        duration = number(duration, 'duration_s', 0., cfg['max_duration_s'])
    shape = args.get('shape', 'minjerk' if duration else 'linear')
    if shape not in SHAPES:
        raise ValueError(f'shape must be one of {SHAPES}')
    return {'kind': 'target', 'indices': indices, 'target': merged, 'duration_s': duration, 'shape': shape}


def parse_trajectory(args, cfg, base):
    """Validate timed waypoints; times are seconds from the call (strictly increasing)."""
    allowed = {'waypoints', 'wait', 'timeout_s', 'record', 'dry_run'}
    if not isinstance(args, dict) or set(args) - allowed:
        raise ValueError(f'unknown trajectory arguments {sorted(set(args or {}) - allowed)}')
    waypoints = args.get('waypoints')
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= MAX_WAYPOINTS:
        raise ValueError(f'waypoints must be a list of 1..{MAX_WAYPOINTS} objects')
    times, rows, union = [], [], set()
    previous = list(base)
    for k, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict) or 't' not in waypoint:
            raise ValueError(f'waypoint {k} needs t (seconds from now) and positions/joints/fingers')
        if set(waypoint) - {'t', 'positions', 'joints', 'fingers'}:
            raise ValueError(f'waypoint {k} has unknown keys {sorted(set(waypoint) - {"t", "positions", "joints", "fingers"})}')
        t = number(waypoint['t'], 't', 1e-3, cfg['max_duration_s'])
        if times and t <= times[-1]+1e-6:
            raise ValueError('waypoint times must strictly increase')
        indices, merged = parse_targets({key: value for key, value in waypoint.items() if key != 't'}, cfg, previous)
        union.update(indices)
        times.append(t)
        rows.append(merged)
        previous = merged
    if not union:
        raise ValueError('trajectory moves no active joint')
    return {'kind': 'trajectory', 'times': times, 'rows': rows, 'indices': tuple(sorted(union)),
            'target': list(rows[-1])}


RECORD_FIELDS = {'controller', 'skill', 'variant', 'params', 'notes', 'max_seconds'}


def check_record_meta(value, cfg, *, label='record'):
    if value is True:
        return {}
    if not isinstance(value, dict) or set(value) - RECORD_FIELDS:
        raise ValueError(f'{label} must be an object with {sorted(RECORD_FIELDS)}')
    result = {}
    for key in ('controller', 'skill', 'variant'):
        if value.get(key) is not None:
            if not isinstance(value[key], str) or len(value[key]) > 120:
                raise ValueError(f'{key} must be text of at most 120 characters')
            result[key] = value[key]
    if value.get('notes') is not None:
        if not isinstance(value['notes'], str) or len(value['notes']) > 4000:
            raise ValueError('notes must be text of at most 4000 characters')
        result['notes'] = value['notes']
    if value.get('params') is not None:
        if not isinstance(value['params'], dict):
            raise ValueError('params must be an object')
        import json
        try:
            text = json.dumps(value['params'], allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'params must be finite JSON: {exc}') from None
        if len(text) > 65536:
            raise ValueError('params exceed 64 KiB')
        result['params'] = json.loads(text)
    if value.get('max_seconds') is not None:
        result['max_seconds'] = number(value['max_seconds'], 'max_seconds', 1., cfg['max_record_s'])
    return result


def check_tool_args(name, args, cfg):
    """Structural validation of the smooth-layer MCP tools (no live state needed)."""
    args = dict(args)
    if name == 'wuji_stream':
        # Disabled-joint retention, relative sums and max_step need the live target: owner checks.
        midpoint = [(lo+hi)/2 for lo, hi in zip(cfg['lower_rad'], cfg['upper_rad'])]
        loose = {**cfg, 'max_step_rad': math.inf, 'disabled_joints': []}
        if args.get('relative') is True:
            loose.update(lower_rad=[-100.]*20, upper_rad=[100.]*20)
        if 'dry_run' in args:
            raise ValueError('dry_run is internal')
        parse_stream(args, loose, midpoint)
        return args
    if name == 'wuji_trajectory':
        midpoint = [(lo+hi)/2 for lo, hi in zip(cfg['lower_rad'], cfg['upper_rad'])]
        loose = {**cfg, 'max_step_rad': math.inf, 'disabled_joints': []}
        parse_trajectory({'waypoints': args.get('waypoints')}, loose, midpoint)
        if set(args) - {'waypoints', 'wait', 'timeout_s', 'record'}:
            raise ValueError(f'unknown trajectory arguments {sorted(set(args) - {"waypoints", "wait", "timeout_s", "record"})}')
        if 'wait' in args and type(args['wait']) is not bool:
            raise ValueError('wait must be true or false')
        if 'timeout_s' in args:
            args['timeout_s'] = number(args['timeout_s'], 'timeout_s', .1, 300.)
        if 'record' in args:
            args['record'] = check_record_meta(args['record'], cfg)
        return args
    if name == 'wuji_set_profile':
        profile_parameters(args, cfg, {'current_limit_a': list(cfg['current_limit_a'])})
        return args
    if name == 'wuji_stream_config':
        update = {k: v for k, v in args.items() if k != 'reset'}
        if 'reset' in args and type(args['reset']) is not bool:
            raise ValueError('reset must be true or false')
        resolve_settings(update, cfg)
        return args
    if name == 'wuji_wait':
        if set(args) - {'timeout_s'}:
            raise ValueError('wuji_wait accepts timeout_s')
        if 'timeout_s' in args:
            args['timeout_s'] = number(args['timeout_s'], 'timeout_s', 0., 60.)
        return args
    raise ValueError(f'unknown smooth-control tool {name}')


# ---------------------------------------------------------------- goal segments

def minjerk(u):
    u = min(1., max(0., u))
    return u**3*(10+u*(-15+6*u)), 30*u*u*(1-u)**2


class Ramp:
    """Linear or minimum-jerk goal from the current goal to a target over a duration."""
    def __init__(self, now, duration, start, end, indices, shape):
        self.t0, self.duration, self.indices, self.shape = now, max(duration, 0.), indices, shape
        self.start, self.delta = [start[i] for i in indices], [end[i]-start[i] for i in indices]
        self.end_time = now+self.duration

    def blend(self, now):
        if self.duration <= 0 or now >= self.end_time:
            return 1., 0.
        u = (now-self.t0)/self.duration
        if self.shape == 'linear':
            return min(1., max(0., u)), 1./self.duration
        s, ds = minjerk(u)
        return s, ds/self.duration

    def write(self, now, goal, owner):
        s, _ = self.blend(now)
        for k, i in enumerate(self.indices):
            if owner[i] is self:
                goal[i] = self.start[k]+s*self.delta[k]

    def value(self, t, i):
        s, _ = self.blend(t)
        k = self.indices.index(i)
        return self.start[k]+s*self.delta[k]

    def velocity(self, now, i):
        _, ds = self.blend(now)
        return ds*self.delta[self.indices.index(i)]


def pchip_slopes(times, values, start_slope=0.):
    """Fritsch-Carlson monotone slopes: no overshoot between waypoints; zero end slope."""
    n = len(times)
    h = [times[k+1]-times[k] for k in range(n-1)]
    delta = [(values[k+1]-values[k])/h[k] for k in range(n-1)]
    slopes = [0.]*n
    for k in range(1, n-1):
        if delta[k-1]*delta[k] <= 0:
            continue
        w1, w2 = 2*h[k]+h[k-1], h[k]+2*h[k-1]
        slopes[k] = (w1+w2)/(w1/delta[k-1]+w2/delta[k])
    if n > 1 and delta[0] != 0 and start_slope*delta[0] > 0:
        slopes[0] = math.copysign(min(abs(start_slope), 3*abs(delta[0])), delta[0])
    return slopes


def hermite(y0, y1, m0, m1, h, s):
    s2, s3 = s*s, s*s*s
    return ((2*s3-3*s2+1)*y0 + (s3-2*s2+s)*h*m0 + (-2*s3+3*s2)*y1 + (s3-s2)*h*m1)


def hermite_peaks(y0, y1, m0, m1, h):
    """Exact peak |velocity| and |acceleration| of one cubic Hermite segment."""
    # p(s) = a s^3 + b s^2 + c s + d in s in [0, 1]; t = s*h.
    a = 2*y0+h*m0-2*y1+h*m1
    b = -3*y0-2*h*m0+3*y1-h*m1
    c = h*m0
    candidates = [0., 1.]
    if abs(a) > 1e-15:
        vertex = -b/(3*a)
        if 0 < vertex < 1:
            candidates.append(vertex)
    velocity = max(abs(3*a*s*s+2*b*s+c) for s in candidates)/h
    acceleration = max(abs(6*a*s+2*b) for s in (0., 1.))/(h*h)
    return velocity, acceleration


class Spline:
    """PCHIP goal through timed waypoints for a set of joints."""
    def __init__(self, now, times, columns, indices, start_slopes):
        self.indices, self.t0 = indices, now
        self.knots = [now+t for t in times]
        self.columns = columns          # per joint: values at knots
        self.slopes = [pchip_slopes(self.knots, col, m0) for col, m0 in zip(columns, start_slopes)]
        self.end_time = self.knots[-1]
        self.cursor = 0

    @staticmethod
    def peaks(times, columns, start_slopes):
        velocity = acceleration = 0.
        for col, m0 in zip(columns, start_slopes):
            slopes = pchip_slopes(times, col, m0)
            for k in range(len(times)-1):
                v, a = hermite_peaks(col[k], col[k+1], slopes[k], slopes[k+1], times[k+1]-times[k])
                velocity, acceleration = max(velocity, v), max(acceleration, a)
        return velocity, acceleration

    def _segment(self, now):
        k = min(self.cursor, len(self.knots)-2)
        while k > 0 and now < self.knots[k]:
            k -= 1
        while k < len(self.knots)-2 and now >= self.knots[k+1]:
            k += 1
        self.cursor = k
        h = self.knots[k+1]-self.knots[k]
        return k, h, min(1., max(0., (now-self.knots[k])/h))

    def write(self, now, goal, owner):
        if now >= self.end_time:
            for c, i in enumerate(self.indices):
                if owner[i] is self:
                    goal[i] = self.columns[c][-1]
            return
        k, h, s = self._segment(now)
        for c, i in enumerate(self.indices):
            if owner[i] is self:
                col, m = self.columns[c], self.slopes[c]
                goal[i] = hermite(col[k], col[k+1], m[k], m[k+1], h, s)

    def value(self, t, i):
        c = self.indices.index(i)
        if t >= self.end_time:
            return self.columns[c][-1]
        k, h, s = self._segment(t)
        col, m = self.columns[c], self.slopes[c]
        return hermite(col[k], col[k+1], m[k], m[k+1], h, s)

    def velocity(self, now, i):
        if now >= self.end_time:
            return 0.
        c = self.indices.index(i)
        k, h, s = self._segment(now)
        col, m = self.columns[c], self.slopes[c]
        y0, y1, m0, m1 = col[k], col[k+1], m[k], m[k+1]
        return ((6*s*s-6*s)*y0 + (3*s*s-4*s+1)*h*m0 + (-6*s*s+6*s)*y1 + (3*s*s-2*s)*h*m1)/h


# ---------------------------------------------------------------- tracker

def braking_time(speed, acceleration, jerk):
    """Time for a jerk-limited stop from (speed, 0 acceleration); the braking distance is
    speed * braking_time / 2, so a lookahead of half this time cancels the steady lag of
    rest-target tracking without ever aiming beyond the commanded goal."""
    if speed <= acceleration*acceleration/jerk:
        return 2*math.sqrt(speed/jerk)
    return speed/acceleration+acceleration/jerk


def require_ruckig():
    from .scurve import require_ruckig as require
    return require()


class Tracker:
    """Online Ruckig toward a moving goal with zero target velocity, per sync group."""
    def __init__(self, cfg, settings, dt, q, v=None, a=None):
        ruckig = require_ruckig()
        self.ruckig, self.dt = ruckig, dt
        active = active_indices(cfg)
        mode = settings['synchronization']
        if mode == 'finger':
            groups = [tuple(i for i in active if i//4 == f) for f in range(5)]
        else:
            groups = [tuple(active)]
        sync = ruckig.Synchronization.No if mode == 'joint' else ruckig.Synchronization.Time
        self.groups = []
        for indices in (g for g in groups if g):
            n = len(indices)
            otg, inp, out = ruckig.Ruckig(n, dt), ruckig.InputParameter(n), ruckig.OutputParameter(n)
            inp.max_velocity = [settings['max_velocity_rad_s'][i] for i in indices]
            inp.max_acceleration = [settings['max_acceleration_rad_s2'][i] for i in indices]
            inp.max_jerk = [settings['max_jerk_rad_s3'][i] for i in indices]
            inp.synchronization = sync
            inp.duration_discretization = ruckig.DurationDiscretization.Continuous
            self.groups.append({'indices': indices, 'otg': otg, 'inp': inp, 'out': out, 'finished': True})
        self.q, self.v, self.a = list(q), list(v or [0.]*20), list(a or [0.]*20)
        self.braking = False
        self.reset(self.q, self.v, self.a)

    def reset(self, q, v=None, a=None):
        self.q, self.v, self.a = list(q), list(v or [0.]*20), list(a or [0.]*20)
        for group in self.groups:
            idx, inp = group['indices'], group['inp']
            inp.current_position = [self.q[i] for i in idx]
            inp.current_velocity = [self.v[i] for i in idx]
            inp.current_acceleration = [self.a[i] for i in idx]
            inp.target_position = [self.q[i] for i in idx]
            inp.target_velocity = [0.]*len(idx)
            inp.target_acceleration = [0.]*len(idx)
            group['finished'] = not any(self.v[i] or self.a[i] for i in idx)

    @property
    def finished(self):
        return all(group['finished'] for group in self.groups)

    def brake(self):
        """Switch to a velocity-interface brake to rest from the current state."""
        self.braking = True
        for group in self.groups:
            inp = group['inp']
            inp.control_interface = self.ruckig.ControlInterface.Velocity
            inp.target_velocity = [0.]*len(group['indices'])
            inp.target_acceleration = [0.]*len(group['indices'])
            group['finished'] = False

    def end_brake(self):
        self.braking = False
        for group in self.groups:
            group['inp'].control_interface = self.ruckig.ControlInterface.Position
        self.reset(self.q)

    def step(self, goal, steps=1):
        ruckig = self.ruckig
        for group in self.groups:
            idx, otg, inp, out = group['indices'], group['otg'], group['inp'], group['out']
            if not self.braking:
                target = [goal[i] for i in idx]
                if group['finished'] and target == list(inp.target_position):
                    continue
                inp.target_position = target
            for _ in range(steps):
                result = otg.update(inp, out)
                if result not in (ruckig.Result.Working, ruckig.Result.Finished):
                    raise RuntimeError(f'stream tracker failed: {result}')
                out.pass_to_input(inp)
                if result == ruckig.Result.Finished:
                    break
            group['finished'] = result == ruckig.Result.Finished
            for k, i in enumerate(idx):
                self.q[i], self.v[i], self.a[i] = out.new_position[k], out.new_velocity[k], out.new_acceleration[k]
            if group['finished']:
                for i in idx:
                    self.v[i] = self.a[i] = 0.
        return self.q, self.v, self.a


# ---------------------------------------------------------------- feed-forward and hold

def friction_feedforward(velocity, measured_velocity, settings, cfg, bias, caps):
    """I_ff = F tanh(v_ref/v0), only on active joints; counts toward the current cap.

    Bias plus friction never exceeds max_feedforward_fraction * cap (an existing bias
    above that is left unchanged, and friction may then only reduce it). Friction is
    withheld on a joint whose measured speed exceeds 1.5x the reference ceiling.
    """
    result = [0.]*20
    if not settings['friction']:
        return result
    v0, fraction = settings['friction_velocity_rad_s'], cfg['max_feedforward_fraction']
    runaway = 1.5*cfg['max_velocity_rad_s']
    for i in active_indices(cfg):
        if not velocity[i]:
            continue
        if measured_velocity is not None and abs(measured_velocity[i]) > runaway:
            continue
        f = settings['friction_current_a'][i]*math.tanh(velocity[i]/v0)
        limit = max(fraction*caps[i], abs(bias[i]))
        total = max(-limit, min(limit, bias[i]+f))
        result[i] = total-bias[i]
    return result


class SettleMonitor:
    """Measured positions within tolerance of the goal, moving less than a range, for a span."""
    def __init__(self, indices, tolerance, span, position_range):
        self.indices, self.tolerance, self.span, self.range = indices, tolerance, span, position_range
        self.reset()

    def reset(self):
        self.start = None
        self.settled = False

    def update(self, now, measured, goal):
        if self.settled:
            return True
        q = measured
        if any(abs(goal[i]-q[i]) > self.tolerance for i in self.indices):
            self.start = None
            return False
        if self.start is None:
            self.start, self.lo, self.hi = now, [q[i] for i in self.indices], [q[i] for i in self.indices]
            return False
        for k, i in enumerate(self.indices):
            if q[i] < self.lo[k]:
                self.lo[k] = q[i]
            elif q[i] > self.hi[k]:
                self.hi[k] = q[i]
            if self.hi[k]-self.lo[k] > self.range:
                self.start, self.lo, self.hi = now, [q[j] for j in self.indices], [q[j] for j in self.indices]
                return False
        self.settled = now-self.start >= self.span
        return self.settled


# ---------------------------------------------------------------- engine

class StreamEngine:
    """Owned by the hand controller's writer thread; commands arrive through apply()."""
    def __init__(self, cfg):
        self.cfg = cfg
        self.active = active_indices(cfg)
        self.settings = resolve_settings({}, cfg)
        self.tracker = None
        self.reset([0.]*20)

    # -- lifecycle
    def reset(self, q, *, keep_settings=False):
        if not keep_settings:
            self.settings = resolve_settings({}, self.cfg)
        self.goal, self.filtered = list(q), list(q)
        self.owner = [None]*20
        self.latest_target = list(q)
        self.trajectory = None
        self.last_step = self.last_target_time = None
        self.arrival_period = None
        self.offset, self.weight, self.idle_since = [0.]*20, 0., None
        self.measured_filtered = None
        self.friction = [0.]*20
        self.limit_clamps = 0
        self.sequence = getattr(self, 'sequence', 0)
        self.braking = False
        self.watch_settle = False
        self.settle = SettleMonitor(self.active, self.cfg['tolerance_rad'], self.cfg['settle_s'],
                                    self.cfg['settle_position_range_rad'])
        self.tracker = None
        self._q, self._v, self._a = list(q), [0.]*20, [0.]*20

    def sync(self, q, v=None, a=None):
        """Adopt a reference produced elsewhere (preparation, S-curve move, hold).

        Settings persist; goals, trajectories and the tracker restart from (q, v, a).
        """
        self.reset(q, keep_settings=True)
        self._v, self._a = list(v or [0.]*20), list(a or [0.]*20)

    def same_reference(self, q):
        return q == self._q

    @property
    def period(self):
        return 1/self.settings['command_hz']

    def _tracker(self):
        if self.tracker is None:
            self.tracker = Tracker(self.cfg, self.settings, self.period, self._q, self._v, self._a)
        return self.tracker

    # -- state
    @property
    def goal_static(self):
        return all(owner is None for owner in self.owner) and (
            self.settings['filter_tau_s'] <= 0 or self.filtered == self.goal)

    @property
    def moving(self):
        return any(self._v) or any(self._a) or (self.tracker is not None and not self.tracker.finished)

    @property
    def converged(self):
        return self.goal_static and not self.moving

    @property
    def engaged(self):
        """Whether the writer must step the engine this tick."""
        return (not self.converged or self.braking or self.weight > 0
                or self.settings['hold_deadband_rad'] > 0
                or (self.watch_settle and not self.settle.settled))

    # -- commands (writer thread)
    def apply(self, command, now):
        kind = command['kind']
        if kind == 'settings':
            if self.moving or not self.goal_static:
                raise ValueError('stream settings change only while the reference is at rest')
            self.settings = command['settings']
            self.tracker = None
            return
        if kind not in ('target', 'trajectory'):
            raise ValueError(f'unknown stream command {kind}')
        self._sample_goal(now)
        spline = self.plan_trajectory(command, now) if kind == 'trajectory' else None  # May reject first.
        self.sequence += 1
        self.settle.reset()
        self.watch_settle = True
        self.latest_target = list(command['target'])
        if kind == 'target':
            duration = command['duration_s']
            if duration is None:
                duration = self.settings['interpolation_s']
            if duration is None:
                low, high, single = AUTO_INTERPOLATION
                recent = self.last_target_time is not None and now-self.last_target_time <= STREAM_GAP_S
                if recent:
                    gap = now-self.last_target_time
                    self.arrival_period = gap if self.arrival_period is None else .7*self.arrival_period+.3*gap
                    duration = min(high, max(low, AUTO_PERIOD_FACTOR*self.arrival_period))
                else:
                    self.arrival_period = None
                    duration = single
            self.last_target_time = now
            ramp = Ramp(now, duration, self.goal, command['target'], command['indices'], command['shape'])
            for i in command['indices']:
                self.owner[i] = ramp
            if ramp.duration <= 0:
                ramp.write(now, self.goal, self.owner)
                for i in command['indices']:
                    self.owner[i] = None
        else:
            for i in spline.indices:
                self.owner[i] = spline
            self.trajectory = {'sequence': self.sequence, 'start': now, 'end': spline.end_time,
                               'time_scale': command.get('time_scale', 1.), 'waypoints': len(command['times']),
                               'done': False}

    def goal_velocity(self, now, i):
        owner = self.owner[i]
        return 0. if owner is None else owner.velocity(now, i)

    def plan_trajectory(self, command, now):
        """Build the PCHIP goal, stretching time uniformly to respect v/a limits."""
        indices = command['indices']
        starts = [self.goal_velocity(now, i) for i in indices]
        times = [0., *command['times']]
        columns = [[self.goal[i], *[row[i] for row in command['rows']]] for i in indices]
        scale = 1.
        for _ in range(6):
            scaled = [t*scale for t in times]
            needed = self._limit_ratio(scaled, columns, indices, starts)
            if needed <= 1+1e-9:
                break
            scale *= needed*1.02
        scaled = [t*scale for t in times]
        if scaled[-1] > self.cfg['max_duration_s']:
            raise ValueError(f'trajectory needs {scaled[-1]:.2f} s at the configured limits, '
                             f'above max_duration_s={self.cfg["max_duration_s"]}')
        command['time_scale'] = scale
        return Spline(now, scaled, columns, indices, starts)

    def _limit_ratio(self, times, columns, indices, starts):
        """Largest ratio of spline peak velocity (or sqrt acceleration) to its limit."""
        velocity_peak = acceleration_peak = 0.
        for c, i in enumerate(indices):
            v, a = Spline.peaks(times, [columns[c]], [starts[c]])
            velocity_peak = max(velocity_peak, v/self.settings['max_velocity_rad_s'][i])
            acceleration_peak = max(acceleration_peak, a/self.settings['max_acceleration_rad_s2'][i])
        return max(velocity_peak, math.sqrt(acceleration_peak))

    def brake(self):
        self.braking = True
        self._tracker().brake()

    # -- per tick
    def _sample_goal(self, now):
        seen = []
        for i in range(20):
            owner = self.owner[i]
            if owner is not None and owner not in seen:
                seen.append(owner)
        for owner in seen:
            owner.write(now, self.goal, self.owner)
            if now >= owner.end_time:
                for i in owner.indices:
                    if self.owner[i] is owner:
                        self.owner[i] = None
        if self.trajectory is not None and now >= self.trajectory['end']:
            self.trajectory['done'] = True

    def step(self, now, measured, bias, caps):
        """Advance one writer tick; returns (q_ref, v_ref, a_ref, offset, friction)."""
        if self.last_step is None:
            steps, dt = 1, self.period
        else:
            elapsed = max(0., now-self.last_step)
            steps = max(1, min(5, round(elapsed/self.period)))
            dt = elapsed
        self.last_step = now
        tracker = self._tracker()
        if self.braking:
            q, v, a = tracker.step(None, steps)
            if tracker.finished:
                self.braking = False
                tracker.end_brake()
                self.goal, self.filtered = list(tracker.q), list(tracker.q)
                self.owner = [None]*20
                self.latest_target = list(tracker.q)
                self.trajectory = None
        else:
            self._sample_goal(now)
            aim = self._lookahead(now) if self.settings['lookahead'] else self.goal
            tau = self.settings['filter_tau_s']
            if tau > 0:
                alpha = 1-math.exp(-dt/tau)
                for i in self.active:
                    f = self.filtered[i]+alpha*(aim[i]-self.filtered[i])
                    self.filtered[i] = aim[i] if abs(aim[i]-f) < 1e-9 else f
                target = self.filtered
            else:
                self.filtered = list(self.goal)
                target = aim
            q, v, a = tracker.step(target, steps)
        q, v, a = list(q), list(v), list(a)
        lower, upper = self.cfg['lower_rad'], self.cfg['upper_rad']
        for i in self.active:
            if q[i] < lower[i] or q[i] > upper[i]:
                q[i] = min(upper[i], max(lower[i], q[i]))
                v[i] = a[i] = 0.
                self.limit_clamps += 1
        self._q, self._v, self._a = q, v, a
        positions = measured.get('positions')
        velocities = measured.get('velocity_rad_s')
        self.friction = friction_feedforward(v, velocities, self.settings, self.cfg, bias, caps)
        self._hold(now, q, positions, dt)
        if not self.converged:
            self.settle.reset()
        elif self.watch_settle and positions is not None:
            self.settle.update(now, positions, self.goal)
        return q, v, a, self.offset, self.friction

    def _lookahead(self, now):
        aim = self.goal
        for i in self.active:
            owner = self.owner[i]
            if owner is None:
                continue
            speed = abs(owner.velocity(now, i))
            if speed <= 1e-9:
                continue
            if aim is self.goal:
                aim = list(self.goal)
            lead = .5*braking_time(speed, self.settings['max_acceleration_rad_s2'][i],
                                   self.settings['max_jerk_rad_s3'][i])
            aim[i] = owner.value(now+lead, i)
        return aim

    def _hold(self, now, q, positions, dt):
        band = self.settings['hold_deadband_rad']
        if positions is None:
            return
        if self.measured_filtered is None:
            self.measured_filtered = list(positions)
        else:
            alpha = 1-math.exp(-max(dt, 1e-4)/MEASURED_FILTER_S)
            mf = self.measured_filtered
            for i in self.active:
                mf[i] += alpha*(positions[i]-mf[i])
        if band <= 0 and self.weight <= 0:
            self.offset = [0.]*20
            return
        if self.converged and band > 0:
            if self.idle_since is None:
                self.idle_since = now
            if now-self.idle_since >= self.settings['hold_delay_s']:
                self.weight = min(1., self.weight+dt/HOLD_BLEND_S)
        else:
            self.idle_since = None
            self.weight = max(0., self.weight-dt/RELEASE_BLEND_S)
        offset = [0.]*20
        if self.weight > 0:
            limit = max(band, 0.)
            for i in self.active:
                error = self.measured_filtered[i]-q[i]
                offset[i] = self.weight*max(-limit, min(limit, error))
        self.offset = offset

    def status(self, now=None):
        trajectory = None
        if self.trajectory is not None:
            trajectory = {k: v for k, v in self.trajectory.items()}
            if now is not None:
                trajectory['elapsed_s'] = max(0., now-self.trajectory['start'])
                trajectory['duration_s'] = self.trajectory['end']-self.trajectory['start']
        return {'sequence': self.sequence, 'moving': self.moving, 'goal_static': self.goal_static,
                'reference_converged': self.converged, 'settled': self.settle.settled,
                'braking': self.braking, 'target_positions': list(self.latest_target),
                'trajectory': trajectory, 'friction_active': any(self.friction),
                'deadband_weight': self.weight, 'limit_clamps': self.limit_clamps,
                'arrival_period_s': self.arrival_period,
                'settings': public_settings(self.settings)}


# ---------------------------------------------------------------- impedance profiles

def profile_parameters(args, cfg, current):
    """Resolve a named preset plus overrides into nominal kp/kd/current caps.

    Returns (desired, info). Values are clamped to the operator ceilings in cfg; caps
    supplied explicitly must not exceed them. Disabled joints stay at zero.
    """
    allowed = {'name', 'kp', 'kd', 'current_limit_a', 'hold_deadband_rad'}
    if not isinstance(args, dict) or set(args) - allowed:
        raise ValueError(f'unknown profile arguments {sorted(set(args or {}) - allowed)}')
    name = args.get('name', 'profile')
    if name not in PROFILE_NAMES:
        raise ValueError(f'profile name must be one of {PROFILE_NAMES}')
    scale = cfg['gain_scale']
    if name == 'profile':
        kp, kd = list(cfg['kp']), list(cfg['kd'])
        deadband = 0.
    else:
        preset = PRESETS[name]
        kp = [min(ceiling, preset['stiffness_a_per_rad']/scale) for ceiling in cfg['kp']]
        kd = [min(ceiling, preset['damping_a_s_per_rad']/scale) for ceiling in cfg['kd']]
        deadband = preset['hold_deadband_rad']
    for key, values in (('kp', kp), ('kd', kd)):
        if key in args:
            override = per_joint(args[key], key, 0., 100.)
            for joint, value, ceiling in zip(JOINT_NAMES, override, cfg[key]):
                if value > ceiling+1e-12 or (key == 'kp' and value <= 0):
                    raise ValueError(f'{joint} {key} must be {"positive and " if key == "kp" else ""}'
                                     f'no greater than the operator ceiling {ceiling:g}')
            values[:] = override
    caps = list(current['current_limit_a'])
    if 'current_limit_a' in args:
        caps = per_joint(args['current_limit_a'], 'current_limit_a', 0., 100.)
    for joint, value, ceiling in zip(JOINT_NAMES, caps, cfg['current_limit_a']):
        if value > ceiling+1e-12:
            raise ValueError(f'{joint} current_limit_a {value:g} exceeds the operator ceiling {ceiling:g}')
    if 'hold_deadband_rad' in args:
        deadband = number(args['hold_deadband_rad'], 'hold_deadband_rad', 0., .02)
    excluded = set(disabled_indices(cfg))
    for i in range(20):
        if i in excluded:
            kp[i] = kd[i] = caps[i] = 0.
        elif caps[i] <= 0 or kp[i] <= 0:
            raise ValueError(f'{JOINT_NAMES[i]} needs a positive kp and current_limit_a')
    desired = {'kp': kp, 'kd': kd, 'current_limit_a': caps}
    return desired, {'name': name, 'hold_deadband_rad': deadband}


def profile_summary(parameters, cfg, name=None):
    """Nominal and effective gains plus the tracking error at which the cap saturates."""
    scale = cfg['gain_scale']
    active = active_indices(cfg)

    def compact(values):
        values = [round(v, 6) for v in values]
        return values[0] if len(set(values)) == 1 else values
    stiffness = [parameters['kp'][i]*scale for i in range(20)]
    saturation = [parameters['current_limit_a'][i]/stiffness[i] if stiffness[i] > 0 else None for i in range(20)]
    return {'name': name, 'gain_scale': scale,
            'kp': compact(parameters['kp']), 'kd': compact(parameters['kd']),
            'current_limit_a': compact(parameters['current_limit_a']),
            'effective_stiffness_a_per_rad': compact([stiffness[i] for i in active]),
            'effective_damping_a_s_per_rad': compact([parameters['kd'][i]*scale for i in active]),
            'cap_saturation_error_rad': compact([saturation[i] for i in active])}
