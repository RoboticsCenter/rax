"""Smooth Wuji Hand 2 control from your own Python: policies, MPC and skill code.

Every call is an MCP tool call through the session's `Context`, so arming, limits and
stops stay with the host. Streaming calls return immediately; the hand service
interpolates between your targets at its command rate (jerk/acceleration/velocity
limited, no stop between targets). Joint order: thumb, index, middle, ring, pinky,
S1..S4 each, absolute radians.

    from api.runtime import Context
    from api.hand import Rate

    def main(ctx, **params):                       # mooncake skill entry point
        hand = ctx.hand('right')                   # bimanual: 'left' | 'right'; single hand: ctx.hand()
        hand.configure(friction=True, filter_tau_s=0.03)
        hand.profile('soft', current_limit_a=0.6)  # per-stage cap within the operator ceiling
        with hand.recording(controller='stream+ff', skill='pinch', params=params) as trial:
            rate = Rate(30)
            for q in my_policy():                  # 20 radians, or hand.stream(index=[...])
                hand.stream(q)
                rate.sleep()
            hand.trajectory([(0.5, q1), (1.0, q2), (1.6, q3)])   # one smooth motion, waits
            result = hand.wait()
            trial.success = result['settled']
        return {'ok': result['settled'], 'measured': {'trace': trial.path, **result}, 'why': '...'}
"""
from contextlib import contextmanager
import time

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
JOINT_NAMES = tuple(f'{finger}_S{joint}' for finger in FINGERS for joint in range(1, 5))


class Rate:
    """Drift-free loop timing: sleep() waits until the next period boundary."""
    def __init__(self, hz):
        if not 0 < hz <= 1000:
            raise ValueError('hz must be in (0, 1000]')
        self.period = 1/hz
        self.next = time.monotonic()+self.period
        self.missed = 0

    def sleep(self):
        now = time.monotonic()
        if now < self.next:
            time.sleep(self.next-now)
            self.next += self.period
        else:
            self.missed += 1
            self.next = now+self.period


class Trial:
    def __init__(self):
        self.id = self.path = self.directory = self.result = None
        self.success = None


class Hand:
    """One Wuji hand. ``side`` is 'left' or 'right' in bimanual sessions, None otherwise
    ('both' is accepted by recording() only)."""
    def __init__(self, ctx, side=None):
        if side not in (None, 'left', 'right', 'both'):
            raise ValueError("side must be None, 'left', 'right' or 'both'")
        self.ctx, self.side = ctx, side
        self._selector = None

    # -- plumbing
    def _needs_selector(self):
        if self._selector is None:
            tools = self.ctx.request('tools/list', {})['tools']
            schema = next((t['inputSchema'] for t in tools if t['name'] == 'wuji_stream'), None)
            if schema is None:
                raise RuntimeError('this session has no Wuji smooth-control tools')
            self._selector = 'hand' in schema.get('properties', {})
        return self._selector

    def _call(self, tool, **arguments):
        arguments = {k: v for k, v in arguments.items() if v is not None}
        if self._needs_selector():
            if self.side is None:
                raise ValueError("bimanual session: use ctx.hand('left') or ctx.hand('right')")
            if self.side == 'both':
                raise ValueError("'both' is only for recording(); stream each hand, or use stream_both()")
            arguments['hand'] = self.side
        return self.ctx.value(tool, **arguments)

    @staticmethod
    def _target(positions=None, joints=None, fingers=None, **finger_values):
        unknown = set(finger_values) - set(FINGERS)
        if unknown:
            raise ValueError(f'unknown keyword {sorted(unknown)}; fingers are {FINGERS}')
        if finger_values:
            fingers = {**(fingers or {}), **{k: list(v) for k, v in finger_values.items()}}
        given = [x is not None for x in (positions, joints, fingers)]
        if sum(given) != 1:
            raise ValueError('give exactly one of positions (20), joints {name: rad} or fingers/index=[4]')
        if positions is not None:
            return {'positions': [float(v) for v in positions]}
        if joints is not None:
            return {'joints': {k: float(v) for k, v in joints.items()}}
        return {'fingers': {k: [float(x) for x in v] for k, v in fingers.items()}}

    # -- observation
    def status(self):
        return self._call('wuji_status')

    def positions(self):
        return self.status()['positions']

    # -- motion
    def stream(self, positions=None, *, joints=None, fingers=None, relative=False, duration_s=None,
               shape=None, **finger_values):
        """Non-blocking target update; omitted joints keep their targets."""
        return self._call('wuji_stream', **self._target(positions, joints, fingers, **finger_values),
                          relative=relative or None, duration_s=duration_s, shape=shape)

    def trajectory(self, waypoints, *, wait=True, timeout_s=None, record=None):
        """waypoints: [(t_seconds, positions | {joint: rad} | {'fingers': {...}}), ...] or tool dicts."""
        items = []
        for waypoint in waypoints:
            if isinstance(waypoint, dict):
                items.append(waypoint)
                continue
            t, value = waypoint
            if isinstance(value, dict) and set(value) <= {'positions', 'joints', 'fingers'}:
                items.append({'t': float(t), **value})
            elif isinstance(value, dict):
                items.append({'t': float(t), 'joints': {k: float(v) for k, v in value.items()}})
            else:
                items.append({'t': float(t), 'positions': [float(v) for v in value]})
        return self._call('wuji_trajectory', waypoints=items, wait=wait, timeout_s=timeout_s, record=record)

    def wait(self, timeout_s=None):
        """Until the reference converged and measured joints settled (or timeout)."""
        return self._call('wuji_wait', timeout_s=timeout_s)

    def hold(self):
        return self._call('wuji_hold')

    def move(self, positions, duration_s=None):
        """The original blocking, rest-to-rest move (waits for settling)."""
        return self._call('wuji_move_joints', positions=list(positions), duration_s=duration_s)

    # -- control settings
    def profile(self, name='profile', *, kp=None, kd=None, current_limit_a=None, hold_deadband_rad=None):
        """Impedance preset: profile | soft | default | firm | hold (reference must be at rest)."""
        return self._call('wuji_set_profile', name=name, kp=kp, kd=kd, current_limit_a=current_limit_a,
                          hold_deadband_rad=hold_deadband_rad)

    def configure(self, **settings):
        """wuji_stream_config: filter_tau_s, interpolation_s, synchronization, max_*_rad_s*,
        friction, friction_current_a, friction_velocity_rad_s, hold_deadband_rad, command_hz, reset."""
        return self._call('wuji_stream_config', **settings)

    # -- traces (robot-level trace_start/trace_stop; one trial directory per run)
    def _sides(self):
        if self.side in ('left', 'right'):
            return [self.side]
        return None  # 'both' or a single-hand session: every configured hand.

    def trace_start(self, *, directory=None, label=None, **meta):
        arguments = {k: v for k, v in {'directory': directory, 'label': label, 'sides': self._sides(),
                                       **meta}.items() if v is not None}
        return self.ctx.value('trace_start', **arguments)

    def trace_stop(self, *, success=None, notes=None):
        arguments = {'success': success, **({'notes': notes} if notes else {})}
        return self.ctx.value('trace_stop', **arguments)

    @contextmanager
    def recording(self, *, controller=None, skill=None, variant=None, params=None, notes=None,
                  max_seconds=None, directory=None, label=None):
        """Record a trace trial; set ``trial.success`` inside the block (exceptions record False).
        ``trial.path`` is the trial directory relative to the workspace (runtime/traces/...)."""
        trial = Trial()
        started = self.trace_start(directory=directory, label=label, controller=controller, skill=skill,
                                   variant=variant, params=params, notes=notes, max_seconds=max_seconds)
        trial.id, trial.path, trial.directory = started['trial'], started.get('path'), started['directory']
        try:
            yield trial
        except BaseException as exc:
            try:
                trial.result = self.trace_stop(success=False, notes=f'{type(exc).__name__}: {exc}'[:4000])
            except Exception:
                pass
            raise
        trial.result = self.trace_stop(success=trial.success)


# ---- function API for skill libraries (e.g. dexlib.io.bind_streaming) ----------------------
#     io.bind_streaming(follow=hand.follow, stream=hand.stream, record=hand.record, available=hand.available)

def available(ctx, tools=None):
    """True when the session serves the smooth layer (non-blocking stream + trace tools).
    ``tools``: an optional set of tool names already listed (avoids another tools/list)."""
    if tools is None:
        tools = {t['name'] for t in ctx.request('tools/list', {}).get('tools', [])}
    return {'wuji_stream', 'wuji_trajectory', 'trace_start', 'trace_stop'} <= set(tools)


def stream(ctx, side, q, **options):
    """One non-blocking 20-joint target (radians) for a policy loop; options are wuji_stream's
    (relative, duration_s, shape). Returns immediately with the accepted target."""
    return Hand(ctx, side).stream(list(q), **options)


def follow(ctx, side, times, positions, **options):
    """A timed trajectory as one smooth motion: times (s from now, increasing) and 20-joint
    positions; options are wuji_trajectory's (wait=True, timeout_s, record)."""
    if len(times) != len(positions):
        raise ValueError('times and positions must have the same length')
    return Hand(ctx, side).trajectory(list(zip(times, positions)), **options)


@contextmanager
def record(ctx, sides, directory=None, **meta):
    """Trace one trial for ``sides`` into ``directory`` (inside the workspace; otherwise the reply's
    directory says where the files went). Yields the trace_start reply; exceptions record success=False."""
    arguments = {'sides': list(sides), **({'directory': str(directory)} if directory else {}),
                 **{k: v for k, v in meta.items() if v is not None}}
    started = ctx.value('trace_start', **arguments)
    try:
        yield started
    except BaseException as exc:
        try:
            ctx.value('trace_stop', success=False, notes=f'{type(exc).__name__}: {exc}'[:4000])
        except Exception:
            pass
        raise
    started['stopped'] = ctx.value('trace_stop')


def profile(ctx, side, name='default', *, current_limit_a=None, **overrides):
    """Impedance preset (profile|soft|default|firm|hold) and per-stage current cap (A)."""
    return Hand(ctx, side).profile(name, current_limit_a=current_limit_a, **overrides)


def stream_both(ctx, left=None, right=None, **common):
    """Bimanual: one MCP call streaming both hands (each a wuji_stream payload)."""
    def payload(value):
        if isinstance(value, dict) and set(value) & {'positions', 'joints', 'fingers'}:
            return {**common, **value}
        return {**common, **Hand._target(value)}
    return ctx.value('wuji_stream_both', left=payload(left), right=payload(right))
