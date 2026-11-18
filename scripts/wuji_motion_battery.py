"""Wuji Hand 2 low-level motion battery (free space, research protocol Phase A).

Compares control conditions on one or both hands and writes one trace trial directory
per run (docs/wuji-smooth-control.md, "Trace format"). Metrics are computed elsewhere.

DEFAULT IS A DRY RUN: it prints and validates the plan against the session's hand
profiles and touches no hardware. Moving hardware needs ALL of:
  --execute, a session whose RAX frontend/agent is NOT running (or --attach to its
  running hand owners), the operator arming the battery session with the existing
  tool (scripts/arm_robot.py <session> arm), and typing READY (or --yes after the
  operator has confirmed the printed plan). Arms are never connected or moved.

    python scripts/wuji_motion_battery.py --session <session dir>                 # plan only
    python scripts/wuji_motion_battery.py --session <dir> --hands right --conditions M0_baseline,M2_stream_friction_ff
    python scripts/wuji_motion_battery.py --session <dir> --execute               # operator at the e-stop
"""
import argparse
from copy import deepcopy
from dataclasses import replace
from datetime import datetime, timezone
import fcntl
import json
import math
from pathlib import Path
import random
import sys
import time
import uuid

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.wuji.config import JOINT_NAMES, validate_config  # noqa: E402
from services.wuji.partial import active_indices, disabled_indices  # noqa: E402
from services.wuji.smooth import hermite, parse_targets, pchip_slopes, Spline  # noqa: E402

FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
J = {name: i for i, name in enumerate(JOINT_NAMES)}

# delivery: blocking = one wuji_move_joints-style rest-to-rest call per waypoint (with
# settling); targets = one stream target per waypoint (minimum-jerk, no settle dwell);
# continuous = the waypoint spline streamed at --stream-hz (online replanning, no
# stops); trajectory = one timed-trajectory call.
# Names are the stable meta.controller grouping keys (research report IDs M0-M4).
CONDITIONS = {
    'M0_baseline': {'delivery': 'blocking', 'profile': 'profile', 'friction': False, 'command_hz': 200,
                    'about': 'current RAX baseline: profile gains/cap, blocking rest-to-rest S-curve calls, 200 Hz'},
    'M1_stream_1khz': {'delivery': 'targets', 'profile': 'profile', 'friction': False,
                       'about': 'streaming targets interpolated at the configured command rate (1 kHz when set)'},
    'M2_stream_friction_ff': {'delivery': 'targets', 'profile': 'profile', 'friction': True,
                              'about': 'M1 plus tanh friction feed-forward'},
    'M3_soft_gains_cap': {'delivery': 'targets', 'profile': 'default', 'friction': True, 'current_limit_a': 1.0,
                          'thermal_guard': True,
                          'about': 'M2 with nominal ~5/0.1 gains (default preset) and a 1.0 A cap clamped to the '
                                   'operator ceiling, with the battery thermal guard'},
    'M4_online_replan': {'delivery': 'continuous', 'profile': 'default', 'friction': True, 'current_limit_a': 1.0,
                         'thermal_guard': True,
                         'about': 'M3 driven by online replanning toward a 30 Hz waypoint stream (no stops)'},
    # Optional extras (run only when named):
    'M4t_trajectory': {'delivery': 'trajectory', 'profile': 'default', 'friction': True, 'current_limit_a': 1.0,
                       'thermal_guard': True,
                       'about': 'M3 with one timed-trajectory call through the waypoints'},
    'M2s_soft_preset': {'delivery': 'targets', 'profile': 'soft', 'friction': True,
                        'about': 'M2 with the soft preset (~3/0.06 nominal)'},
    'M2h_hold_preset': {'delivery': 'targets', 'profile': 'hold', 'friction': True,
                        'about': 'M2 with the hold preset (soft gains plus hold deadband)'},
}
DEFAULT_CONDITIONS = ('M0_baseline', 'M1_stream_1khz', 'M2_stream_friction_ff', 'M3_soft_gains_cap',
                      'M4_online_replan')
ITEMS = ('single_joint', 'stick_slip', 'fist_pinch', 'tapping', 'hold', 'waypoints5')
SINGLE_JOINTS = ('thumb_S2', 'index_S1', 'index_S3', 'ring_S1')
STICK_SLIP_JOINTS = ('index_S1', 'ring_S1')
LIMIT_MARGIN = .05


# ---------------------------------------------------------------- plan

def keyframe_duration(a, b, speed):
    """Minimum-jerk segment time whose peak joint speed is `speed` (1.875 x mean)."""
    distance = max((abs(x-y) for x, y in zip(a, b)), default=0.)
    return max(.3, 1.875*distance/speed) if distance > 0 else .5


def within(cfg, i, value):
    return cfg['lower_rad'][i]+LIMIT_MARGIN <= value <= cfg['upper_rad'][i]-LIMIT_MARGIN


def build_items(cfg, postures, *, speeds, items, hold_s):
    """[(item, variant, params, keyframes, continuous, skip_reason)] for one hand at each speed.

    Speed-independent items (stick-slip probe, holds) are generated once.
    """
    clipped = sorted({round(min(v, .9*cfg['max_velocity_rad_s']), 4) for v in speeds})
    plan = []
    for k, speed in enumerate(clipped):
        selected = [i for i in items if k == 0 or i not in ('stick_slip', 'hold')]
        for item, variant, params, frames, continuous, reason in build_items_at(
                cfg, postures, speed=speed, items=selected, hold_s=hold_s):
            if item not in ('stick_slip', 'hold') and variant is not None:
                variant = f'{variant}@{speed:g}'
            plan.append((item, variant, params, frames, continuous, reason))
    return plan


def build_items_at(cfg, postures, *, speed, items, hold_s):
    home = postures.get('home')
    if home is None:
        return [(item, None, {}, None, False, 'no commissioned home posture') for item in items]
    excluded = set(disabled_indices(cfg))
    plan = []

    def path(points, dwell=.5):
        frames, t = [(0., list(home))], 0.
        for q in points:
            t += keyframe_duration(frames[-1][1], q, speed)
            frames.append((t, list(q)))
            if dwell:
                t += dwell
                frames.append((t, list(q)))
        return frames

    def moved(base, changes):
        q = list(base)
        for i, delta in changes.items():
            q[i] = base[i]+delta
        return q

    for item in items:
        if item == 'single_joint':
            for name in SINGLE_JOINTS:
                i = J[name]
                if i in excluded:
                    plan.append((item, name, {}, None, False, f'{name} is disabled on this hand'))
                    continue
                points = []
                for amplitude in (.05, .4):
                    for sign in (1, -1):
                        if within(cfg, i, home[i]+sign*amplitude):
                            points += [moved(home, {i: sign*amplitude}), list(home)]
                plan.append((item, name, {'joint': name, 'amplitudes_rad': [.05, .4], 'speed_rad_s': speed},
                             path(points), False, None if points else 'no amplitude fits the joint limits'))
        elif item == 'stick_slip':
            for name in STICK_SLIP_JOINTS:
                i = J[name]
                if i in excluded:
                    plan.append((item, name, {}, None, False, f'{name} is disabled on this hand'))
                    continue
                sign = 1 if within(cfg, i, home[i]+.2) else -1
                if not within(cfg, i, home[i]+sign*.2):
                    plan.append((item, name, {}, None, False, 'no 0.2 rad ramp fits the joint limits'))
                    continue
                ramp_v, frames, t = .03, [(0., list(home))], 0.
                for k in range(1, 11):          # collinear knots: constant-velocity spline
                    frames.append((t+k*(.02/ramp_v), moved(home, {i: sign*.02*k})))
                t = frames[-1][0]+1.
                frames.append((t, moved(home, {i: sign*.2})))
                for k in range(1, 11):
                    frames.append((t+k*(.02/ramp_v), moved(home, {i: sign*.2-sign*.02*k})))
                t = frames[-1][0]+1.
                frames.append((t, list(home)))
                amplitude = .1 if within(cfg, i, home[i]+.1) and within(cfg, i, home[i]-.1) else .05
                for k in range(1, 17):          # 0.2 Hz sinusoid, 2 cycles, 8 knots per cycle
                    frames.append((t+k*.625, moved(home, {i: amplitude*math.sin(2*math.pi*k/8)})))
                plan.append((item, name, {'joint': name, 'ramp_rad_s': ramp_v, 'ramp_rad': .2,
                                          'sine_hz': .2, 'sine_amplitude_rad': amplitude},
                             frames, True, None))
        elif item == 'fist_pinch':
            for pose in ('fist', 'pinch'):
                if pose not in postures:
                    plan.append((item, pose, {}, None, False, f'no commissioned {pose!r} posture (--postures)'))
                    continue
                target = [home[i] if i in excluded else v for i, v in enumerate(postures[pose])]
                plan.append((item, pose, {'posture': pose, 'speed_rad_s': speed},
                             path([target, list(home)], dwell=1.), False, None))
        elif item == 'tapping':
            taps = [f'tap_{finger}' for finger in FINGERS[1:]]
            if all(name in postures for name in taps):
                points = []
                for name in taps:
                    points += [[home[i] if i in excluded else v for i, v in enumerate(postures[name])], list(home)]
                plan.append((item, 'thumb_opposition', {'postures': taps, 'speed_rad_s': speed},
                             path(points, dwell=.2), False, None))
            else:
                points = []
                for finger in FINGERS[1:]:
                    i = J[f'{finger}_S1']
                    if i not in excluded and within(cfg, i, home[i]+.3):
                        points += [moved(home, {i: .3}), list(home)]
                plan.append((item, 'finger_sequence', {'flex_rad': .3, 'joints': 'index..pinky S1',
                                                       'speed_rad_s': speed,
                                                       'note': 'no thumb contact; tap_* postures enable opposition'},
                             path(points, dwell=.2), False, None if points else 'no finger flex fits'))
        elif item == 'hold':
            for pose in ('home', 'fist'):
                if pose not in postures:
                    continue
                q = [home[i] if i in excluded else v for i, v in enumerate(postures[pose])]
                frames = [(0., list(home))] if pose == 'home' else path([q], dwell=0.)
                frames.append((frames[-1][0]+hold_s, list(q)))
                plan.append((item, pose, {'posture': pose, 'hold_s': hold_s}, frames, False, None))
        elif item == 'waypoints5':
            if 'fist' in postures:
                delta = [0. if i in excluded else postures['fist'][i]-home[i] for i in range(20)]
                basis = 'home->fist'
            else:
                delta = [0.]*20
                for finger in FINGERS[1:]:
                    for joint in ('S1', 'S3', 'S4'):
                        i = J[f'{finger}_{joint}']
                        if i not in excluded and within(cfg, i, home[i]+.3):
                            delta[i] = .3
                basis = 'mild flexion of index..pinky S1/S3/S4 (+0.3 rad)'
            points = [[home[i]+s*delta[i] for i in range(20)] for s in (.3, .6, .9, .5, 0.)]
            plan.append((item, 'five', {'fractions': [.3, .6, .9, .5, 0.], 'basis': basis, 'speed_rad_s': speed},
                         path(points, dwell=0.), False, None))
    return plan


def check_keyframes(cfg, frames):
    """Validate every keyframe with the owner's own target rules; returns summary numbers."""
    previous = list(frames[0][1])
    for _, q in frames:
        parse_targets({'positions': q}, cfg, previous)
        previous = list(q)
    times = [t for t, _ in frames]
    active = active_indices(cfg)
    peak = 0.
    for i in active:
        column = [q[i] for _, q in frames]
        if len(times) > 1:
            v, _ = Spline.peaks(times, [column], [0.])
            peak = max(peak, v)
    margin = min(min(q[i]-cfg['lower_rad'][i], cfg['upper_rad'][i]-q[i]) for _, q in frames for i in active)
    return {'duration_s': times[-1], 'keyframes': len(frames), 'peak_spline_speed_rad_s': peak,
            'min_limit_margin_rad': margin,
            'max_excursion_rad': max(abs(q[i]-frames[0][1][i]) for _, q in frames for i in active)}


def hand_profiles(options):
    """{side: validated hand cfg} for a single-arm or bimanual session."""
    if 'arms' in options:
        return {side: validate_config(options['arms'][side]['end_effector']) for side in ('left', 'right')}
    cfg = validate_config(options['end_effector'])
    return {cfg['handedness']: cfg}


PATCHABLE = frozenset({'command_hz', 'current_limit_a', 'kp', 'kd', 'max_acceleration_rad_s2', 'max_jerk_rad_s3',
                       'stream_max_acceleration_rad_s2', 'stream_max_jerk_rad_s3', 'friction_current_a',
                       'friction_velocity_rad_s', 'max_feedforward_fraction', 'max_scheduler_gap_s'})


def patched_options(options, patch):
    """Operator profile patch for the battery's own session: {"all"|side: {key: value}}.

    Limited to rate, gain/cap ceilings and stream limits; validated by the normal profile
    rules (ceilings: current <= 2 A, kp <= 10, kd <= 2). Never used with --attach.
    """
    if not patch:
        return options
    options = deepcopy(options)
    sides = list(options['arms']) if 'arms' in options else [options['end_effector']['handedness']]
    for key, change in patch.items():
        if key not in ('all', *sides) or not isinstance(change, dict) or set(change) - PATCHABLE:
            raise ValueError(f'--hand-patch accepts {{"all"|side: {{{", ".join(sorted(PATCHABLE))}}}}}')
    for side in sides:
        hand = options['arms'][side]['end_effector'] if 'arms' in options else options['end_effector']
        hand.update({**patch.get('all', {}), **patch.get(side, {})})
        validate_config(hand)
    return options


def make_plan(options, *, hands, conditions, items, repeats, speeds, hold_s, extra_postures, shuffle, seed):
    profiles = hand_profiles(options)
    trials, skipped = [], []
    for side in hands:
        if side not in profiles:
            raise ValueError(f'session has no {side} hand; available: {sorted(profiles)}')
        cfg = profiles[side]
        postures = {**cfg['postures'], **{k: v for k, v in extra_postures.get(side, {}).items()}}
        for name, positions in extra_postures.get(side, {}).items():
            from services.wuji.config import target
            target(positions, cfg)
        for item, variant, params, frames, continuous, reason in build_items(
                cfg, postures, speeds=speeds, items=items, hold_s=hold_s):
            if reason:
                skipped.append({'hand': side, 'item': item, 'variant': variant, 'reason': reason})
                continue
            summary = check_keyframes(cfg, frames)
            for condition in conditions:
                for repeat in range(repeats):
                    trials.append({'hand': side, 'condition': condition, 'item': item, 'variant': variant,
                                   'repeat': repeat, 'params': params, 'keyframes': frames,
                                   'continuous': continuous, 'summary': summary})
    if shuffle:
        random.Random(seed).shuffle(trials)
    return {'trials': trials, 'skipped': skipped, 'profiles': {
        side: {k: profiles[side][k] for k in ('profile_revision', 'command_hz', 'max_velocity_rad_s',
                                              'max_acceleration_rad_s2', 'max_jerk_rad_s3', 'stream_max_acceleration_rad_s2',
                                              'stream_max_jerk_rad_s3', 'friction_current_a', 'disabled_joints',
                                              'current_limit_a', 'kp', 'kd', 'max_temperature_c')}
        for side in hands}}


def describe(plan, *, stream_hz):
    lines = ['Wuji motion battery plan (DRY RUN unless --execute):']
    for side, profile in plan['profiles'].items():
        stream_a = profile['stream_max_acceleration_rad_s2'] or profile['max_acceleration_rad_s2']
        stream_j = profile['stream_max_jerk_rad_s3'] or profile['max_jerk_rad_s3']
        lines.append(f"  {side}: {profile['profile_revision']}, command_hz {profile['command_hz']:g}, "
                     f"S-curve v/a/j {profile['max_velocity_rad_s']}/{profile['max_acceleration_rad_s2']}/"
                     f"{profile['max_jerk_rad_s3']}, stream a/j {stream_a}/{stream_j}, "
                     f"cap ceiling {max(profile['current_limit_a']):g} A, kp/kd ceiling "
                     f"{max(profile['kp']):g}/{max(profile['kd']):g}, disabled {profile['disabled_joints'] or 'none'}")
    total = 0.
    for k, trial in enumerate(plan['trials']):
        s = trial['summary']
        total += s['duration_s']+6.
        lines.append(f"  {k+1:3d}. {trial['hand']:5s} {trial['condition']:16s} {trial['item']:12s} "
                     f"{str(trial['variant']):16s} {s['keyframes']:3d} waypoints {s['duration_s']:6.1f} s "
                     f"excursion {s['max_excursion_rad']:.3f} rad, limit margin {s['min_limit_margin_rad']:.3f} rad")
    for item in plan['skipped']:
        lines.append(f"  skipped: {item['hand']} {item['item']} {item['variant'] or ''}: {item['reason']}")
    lines.append(f'  {len(plan["trials"])} trials, about {total/60:.1f} min of motion plus setup and rests; '
                 f'streams at {stream_hz:g} Hz.')
    used = {t['condition'] for t in plan['trials']}
    for side, profile in plan['profiles'].items():
        if 'M1_stream_1khz' in used and profile['command_hz'] < 1000:
            lines.append(f"  note: {side} command_hz is {profile['command_hz']:g}, so M1..M4 stream at that rate. "
                         'For 1 kHz pass --hand-patch with {"all": {"command_hz": 1000}} after checking host timing.')
        stream_j = profile['stream_max_jerk_rad_s3'] or profile['max_jerk_rad_s3']
        if used - {'M0_baseline'} and stream_j < 10:
            lines.append(f"  note: {side} stream jerk limit is {stream_j:g} rad/s^3 (the S-curve value); streamed "
                         'references will lag and look sluggish. See docs/wuji-smooth-control.md, "Tuning", for '
                         'stream_max_acceleration_rad_s2 / stream_max_jerk_rad_s3 (--hand-patch).')
        if used & {'M3_soft_gains_cap', 'M4_online_replan'} and max(profile['current_limit_a']) < 1.0:
            lines.append(f"  note: {side} cap ceiling is {max(profile['current_limit_a']):g} A, so M3/M4 run at that "
                         'cap, not 1.0 A. Raising it is an operator decision (--hand-patch current_limit_a).')
    lines.append('  Conditions: ' + '; '.join(f'{name}: {CONDITIONS[name]["about"]}'
                                              for name in dict.fromkeys(t['condition'] for t in plan['trials'])))
    return '\n'.join(lines)


# ---------------------------------------------------------------- hardware link (IPC to hand owners)

class HandLink:
    """Operator access to one existing hand owner over its private socket (never MCP)."""
    def __init__(self, root, side, cfg, *, socket_side):
        from services.wuji import client
        self.client, self.root, self.side, self.cfg, self.socket_side = client, Path(root), side, cfg, socket_side

    def status(self):
        return self.client.request(self.client.socket_path(self.root, self.socket_side), 'status')

    def _operation(self, op, args=None, timeout=10.):
        return self.client.operation(self.root, op, args, timeout=timeout, side=self.socket_side)

    def prepare(self):
        return self._operation('prepare', timeout=15.)

    def move(self, positions, duration_s=None):
        args = {'positions': positions}
        if duration_s is not None:
            args['duration_s'] = min(max(duration_s, .1), self.cfg['max_duration_s'])
        timeout = (self.cfg['max_duration_s']*self.cfg['reference_timeout_scale']
                   + 2*self.cfg['settle_timeout_s']+5)
        return self._operation('move', args, timeout=timeout)

    def direct(self, op, args=None):
        return self.client.direct(self.root, op, args, side=self.socket_side)

    def stream(self, **args):
        return self.direct('stream', args)

    def trajectory(self, waypoints):
        return self.direct('trajectory', {'waypoints': waypoints})

    def configure(self, **settings):
        return self.direct('stream_config', settings)

    def set_profile(self, **args):
        return self._operation('set_profile', args, timeout=self.cfg['stop_timeout_s']+5)

    def wait(self, timeout_s, after=None):
        return self.client.wait_settled(self.root, side=self.socket_side, timeout_s=timeout_s, after=after)

    def record_start(self, directory, **info):
        return self.direct('record', {'action': 'start', 'directory': str(directory), **info})

    def record_stop(self):
        return self.client.record_stop(self.root, side=self.socket_side)

    def stop(self):
        return self._operation('emergency', timeout=self.cfg['stop_timeout_s']+3)


def spline_sampler(frames):
    times = [t for t, _ in frames]
    columns = [[q[i] for _, q in frames] for i in range(20)]
    slopes = [pchip_slopes(times, column) for column in columns]

    def sample(t):
        if t >= times[-1]:
            return list(frames[-1][1])
        k = max(0, min(len(times)-2, next(k for k in range(len(times)-1) if t < times[k+1])))
        h = times[k+1]-times[k]
        s = max(0., (t-times[k])/h)
        return [hermite(columns[i][k], columns[i][k+1], slopes[i][k], slopes[i][k+1], h, s) for i in range(20)]
    return sample


class ThermalGuard:
    """Battery-level thermal guard for raised-cap conditions: polls the owner's hottest MCU
    temperature and requests an emergency stop above ``limit_c`` (below the profile's own
    max_temperature_c stop, which remains active)."""
    def __init__(self, link, limit_c, *, period_s=.2, log=print):
        import threading
        self.link, self.limit_c, self.period_s, self.log = link, limit_c, period_s, log
        self.halt, self.tripped = threading.Event(), None
        self.thread = threading.Thread(target=self._run, name='battery-thermal-guard', daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.halt.set()
        self.thread.join(2)

    def _run(self):
        while not self.halt.wait(self.period_s):
            try:
                hottest = self.link.direct('stream_status', {}).get('temperature_max_c')
            except Exception:  # noqa: BLE001 -- the owner's own watchdogs still apply.
                continue
            if hottest is not None and hottest >= self.limit_c:
                self.tripped = f'{self.link.side} thermal guard: {hottest:.1f} C >= {self.limit_c:g} C'
                self.log(self.tripped)
                try:
                    self.link.stop()
                finally:
                    return


class Runner:
    def __init__(self, links, out, *, stream_hz=30., start_temp_c=50., cooldown_max_s=600., rest_s=10.,
                 thermal_guard_c=58., log=print, sleep=time.sleep, clock=time.monotonic):
        from services.wuji.trace import TrialRecorder
        self.links, self.out, self.stream_hz = links, Path(out), stream_hz
        self.thermal_guard_c = thermal_guard_c
        self.start_temp_c, self.cooldown_max_s, self.rest_s = start_temp_c, cooldown_max_s, rest_s
        self.log, self.sleep, self.clock = log, sleep, clock
        self.recorder = TrialRecorder(self.out/'trials')
        self.applied = {}

    def healthy(self, link):
        state = link.status()
        if (state.get('mode') != 'holding' or not state.get('fresh') or state.get('fault') or state.get('faults')):
            raise RuntimeError(f'{link.side} hand is not a fresh, fault-free hold (mode={state.get("mode")}, '
                               f'fault={state.get("fault")}); battery stopped')
        return state

    def cool(self, link):
        deadline = self.clock()+self.cooldown_max_s
        while True:
            state = self.healthy(link)
            temperatures = (state.get('diagnostics') or {}).get('mcu_temp_c_fb') or []
            hottest = max(temperatures) if temperatures else None
            if hottest is None or hottest < self.start_temp_c:
                return hottest
            if self.clock() >= deadline:
                raise RuntimeError(f'{link.side} hand stayed at {hottest:.1f} C, above {self.start_temp_c} C')
            self.log(f'{link.side}: cooling ({hottest:.1f} C >= {self.start_temp_c} C)')
            self.sleep(10.)

    def live(self, link, q):
        """Copy disabled joints from the live reference (partial hands)."""
        target = link.status()['target_positions']
        excluded = set(disabled_indices(link.cfg))
        return [target[i] if i in excluded else v for i, v in enumerate(q)]

    def apply_condition(self, link, name):
        condition = CONDITIONS[name]
        rate = min(link.cfg['command_hz'], condition.get('command_hz') or link.cfg['command_hz'])
        link.wait(link.cfg['settle_timeout_s'])
        link.configure(reset=True, friction=condition['friction'], command_hz=rate)
        # The per-stage cap never exceeds the operator ceiling of each joint.
        cap = condition.get('current_limit_a')
        caps = [min(cap, c) if cap is not None else c for c in link.cfg['current_limit_a']]
        caps = [0. if i in set(disabled_indices(link.cfg)) else c for i, c in enumerate(caps)]
        wanted = (condition['profile'], tuple(caps))
        if self.applied.get(link.side) != wanted:
            link.wait(link.cfg['settle_timeout_s'])
            link.set_profile(name=condition['profile'], current_limit_a=caps)
            self.applied[link.side] = wanted
        return rate, caps

    def deliver(self, link, trial):
        frames = [(t, self.live(link, q)) for t, q in trial['keyframes']]
        delivery = CONDITIONS[trial['condition']]['delivery']
        active = active_indices(link.cfg)
        if trial['continuous'] and delivery == 'targets':
            delivery = 'continuous'  # Constant-velocity probes need a continuous reference.

        def joints(q):
            return {'joints': {JOINT_NAMES[i]: q[i] for i in active}}
        start = self.clock()
        if delivery == 'blocking':
            for (t0, a), (t1, b) in zip(frames, frames[1:]):
                if max(abs(x-y) for x, y in zip(a, b)) < 1e-9:
                    self.sleep(max(0., t1-t0))
                else:
                    link.move(b, t1-t0)
        elif delivery == 'targets':
            for (t0, a), (t1, b) in zip(frames, frames[1:]):
                self._until(start+t0)
                link.stream(**joints(b), duration_s=t1-t0)
            self._until(start+frames[-1][0])
        elif delivery == 'continuous':
            sample, period = spline_sampler(frames), 1/self.stream_hz
            tick = 0
            while True:
                t = tick*period
                if t > frames[-1][0]:
                    break
                self._until(start+t)
                link.stream(**joints(sample(t)))
                tick += 1
            link.stream(**joints(frames[-1][1]))
        elif delivery == 'trajectory':
            accepted = link.trajectory([{'t': t, **joints(q)} for t, q in frames[1:]])
            self._until(start+accepted['estimated_duration_s'])
        return link.wait(link.cfg['settle_timeout_s']+2)

    def _until(self, when):
        remaining = when-self.clock()
        if remaining > 0:
            self.sleep(remaining)

    def run(self, plan):
        results = []
        for k, trial in enumerate(plan['trials']):
            link = self.links[trial['hand']]
            temperature = self.cool(link)
            rate, caps = self.apply_condition(link, trial['condition'])
            link.move(self.live(link, trial['keyframes'][0][1]), 1.)   # Unrecorded approach to the start pose.
            link.wait(link.cfg['settle_timeout_s'])
            label = f"{trial['condition']}-{trial['item']}-{trial['variant']}-{trial['hand']}"
            self.recorder.start({trial['hand']: link}, controller=trial['condition'], skill=trial['item'],
                                variant=f"{trial['variant']}-r{trial['repeat']}", label=label,
                                params={**trial['params'], 'condition': CONDITIONS[trial['condition']],
                                        'command_hz': rate, 'stream_hz': self.stream_hz,
                                        'current_limit_a': caps, 'keyframes': trial['keyframes']})
            outcome, error = None, None
            guard = ThermalGuard(link, self.thermal_guard_c, log=self.log) if (
                CONDITIONS[trial['condition']].get('thermal_guard') and self.thermal_guard_c) else None
            try:
                if guard:
                    guard.start()
                outcome = self.deliver(link, trial)
                self.sleep(.5)  # Recorded tail.
                if guard and guard.tripped:
                    raise RuntimeError(guard.tripped)
            except BaseException as exc:
                error = f'{type(exc).__name__}: {exc}'
                raise
            finally:
                if guard:
                    guard.stop()
                saved = self.recorder.stop(success=error is None and bool(outcome), notes=error or '',
                                           battery={'index': k, 'start_temperature_c': temperature,
                                                    'outcome': outcome})
                results.append({'trial': saved['trial'], 'hand': trial['hand'], 'condition': trial['condition'],
                                'item': trial['item'], 'variant': trial['variant'], 'error': error,
                                'settled': (outcome or {}).get('settled')})
                self.log(f"[{k+1}/{len(plan['trials'])}] {label}: "
                         f"{'ERROR '+error if error else (outcome or {}).get('outcome')} -> {saved['trial']}")
            if self.rest_s:
                self.sleep(self.rest_s)
        return results


# ---------------------------------------------------------------- operator entry point

def confirm(yes):
    if yes:
        return
    with open('/dev/tty', 'r+') as tty:
        tty.write('Hand(s) in free space, arm stopped, person at the e-stop, plan reviewed? Type READY: ')
        tty.flush()
        if tty.readline().strip() != 'READY':
            raise SystemExit('battery cancelled; nothing moved')


def wait_for_arming(root, minutes, log=print):
    from services.xarm import arming
    log(f'Arm the battery session with the existing tool, then return here:\n'
        f'  python scripts/arm_robot.py {root} arm --minutes 30')
    deadline = time.monotonic()+60*minutes
    while time.monotonic() < deadline:
        if arming.read(root)['armed']:
            return
        time.sleep(1.)
    raise SystemExit('the operator did not arm the battery session; nothing moved')


def execute(args, config, plan):
    from services.controller.config import Configuration
    from services.wuji.client import WujiClient, socket_path
    from services.xarm import arming
    source = config.root
    options = patched_options(config.backend_options, args.patch)
    bimanual = 'arms' in options
    sides = sorted({t['hand'] for t in plan['trials']})
    profiles = hand_profiles(options)
    clients = []
    if args.attach and args.patch:
        raise SystemExit('--hand-patch needs the battery to own its hand service; it cannot change a running owner')
    if args.attach:
        root = source
        out = Path(args.out) if args.out else source/'operator'/f'wuji-battery-{stamp()}'
    else:
        with (source/'launch.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise SystemExit('exit the running RAX session first, or use --attach to its hand owners') from None
        for side in profiles:
            if socket_path(source, side if bimanual else None).exists():
                raise SystemExit('a hand owner is already running for this session; use --attach')
        root = source.parent/f'{stamp()}_wuji_motion_battery_{uuid.uuid4().hex[:6]}'
        root.mkdir(mode=0o700)
        value = config.to_dict()
        value.update(root=str(root), task='wuji_motion_battery', session_seconds=4*3600, backend_options=options,
                     task_description='Wuji motion battery (operator script; no agent).', subagents=0)
        config = Configuration.from_dict(value)
        (root/'operator-input.json').write_text(json.dumps(config.to_dict(), indent=2) + '\n')
        out = Path(args.out) if args.out else root
    out.mkdir(parents=True, exist_ok=True)
    (out/'battery-plan.json').write_text(json.dumps({k: v for k, v in plan.items()}, indent=2, default=str))
    print(f'Battery session: {root}\nTrials: {out/"trials"}', flush=True)
    faults, links = [], {}
    try:
        for side in sides:
            socket_side = side if bimanual else None
            if not args.attach:
                child = (replace(config, robot='xarm', backend_options={**_arm_options(options, side)})
                         if bimanual else config)
                clients.append(WujiClient(child, lambda side=side: faults.append(side), side=socket_side))
            links[side] = HandLink(root, side, profiles[side], socket_side=socket_side)
        wait_for_arming(root, args.arm_wait_min)
        confirm(args.yes)
        for side, link in links.items():
            state = link.status()
            if state.get('mode') != 'holding':
                print(f'{side}: preparing the hand at its measured posture', flush=True)
                link.prepare()
        runner = Runner(links, out, stream_hz=args.stream_hz, start_temp_c=args.start_temp_c,
                        cooldown_max_s=args.cooldown_max_s, rest_s=args.rest_s,
                        thermal_guard_c=args.thermal_guard_c)
        results = runner.run(plan)
        (out/'battery-results.json').write_text(json.dumps(results, indent=2, default=str))
        print(json.dumps({'trials': len(results), 'errors': sum(1 for r in results if r['error'])}), flush=True)
    finally:
        try:
            arming.write(root, False, reason='wuji motion battery finished')
        finally:
            for link in links.values():
                try:
                    link.stop()
                except Exception as exc:  # noqa: BLE001
                    print(f'{link.side} stop request failed: {exc}', file=sys.stderr)
            for client in clients:
                client.close()


def _arm_options(options, side):
    from services.xarm_bimanual.config import arm_options
    return arm_options(options, side)


def stamp():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--session', type=Path, required=True, help='session directory or operator-input.json')
    parser.add_argument('--hands', default=None, help='left,right (default: every configured hand)')
    parser.add_argument('--conditions', default=','.join(DEFAULT_CONDITIONS),
                        help=f'comma list from {", ".join(CONDITIONS)}')
    parser.add_argument('--items', default=','.join(ITEMS), help=f'comma list from {", ".join(ITEMS)}')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--speeds', default='0.3',
                        help='peak joint speeds rad/s, comma list (protocol: 0.1,0.3,0.6); clipped to 0.9x profile')
    parser.add_argument('--hold-s', type=float, default=30.)
    parser.add_argument('--stream-hz', type=float, default=30.)
    parser.add_argument('--postures', type=Path, help='JSON {side: {name: [20 rad]}} e.g. fist, pinch, tap_index')
    parser.add_argument('--hand-patch', type=Path,
                        help='JSON {"all"|side: {...}} raising rate/cap/gain/stream ceilings for the battery '
                             'session only (operator decision; validated; not with --attach)')
    parser.add_argument('--thermal-guard-c', type=float, default=58.,
                        help='emergency stop in M3/M4 conditions above this MCU temperature (0 disables)')
    parser.add_argument('--shuffle', action='store_true', help='randomize trial order (use --seed)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--out', help='trial output directory (default: the battery session)')
    parser.add_argument('--plan-out', type=Path, help='also write the plan JSON here (dry run too)')
    parser.add_argument('--execute', action='store_true', help='move hardware (see the safety requirements above)')
    parser.add_argument('--attach', action='store_true', help='use the running session hand owners')
    parser.add_argument('--yes', action='store_true', help='operator already confirmed the printed plan')
    parser.add_argument('--arm-wait-min', type=float, default=10.)
    parser.add_argument('--start-temp-c', type=float, default=50.)
    parser.add_argument('--cooldown-max-s', type=float, default=600.)
    parser.add_argument('--rest-s', type=float, default=10.)
    args = parser.parse_args(argv)
    from services.controller.config import Configuration
    path = args.session.expanduser().resolve()
    config = Configuration.load(path if path.is_file() else path/'operator-input.json')
    profiles = hand_profiles(config.backend_options)
    hands = args.hands.split(',') if args.hands else sorted(profiles)
    conditions = args.conditions.split(',')
    items = args.items.split(',')
    for name in conditions:
        if name not in CONDITIONS:
            parser.error(f'unknown condition {name}')
    for name in items:
        if name not in ITEMS:
            parser.error(f'unknown item {name}')
    try:
        speeds = [float(v) for v in args.speeds.split(',')]
    except ValueError:
        parser.error('--speeds must be a comma list of numbers')
    if not 1 <= args.repeats <= 50 or not 1 <= args.stream_hz <= 100 or not all(.01 <= v <= 3 for v in speeds):
        parser.error('repeats 1..50, stream-hz 1..100, speeds 0.01..3')
    extra = json.loads(args.postures.read_text()) if args.postures else {}
    args.patch = json.loads(args.hand_patch.read_text()) if args.hand_patch else {}
    try:
        options = patched_options(config.backend_options, args.patch)
    except ValueError as exc:
        parser.error(str(exc))
    plan = make_plan(options, hands=hands, conditions=conditions, items=items,
                     repeats=args.repeats, speeds=speeds, hold_s=args.hold_s, extra_postures=extra,
                     shuffle=args.shuffle, seed=args.seed)
    plan['hand_patch'] = args.patch
    print(describe(plan, stream_hz=args.stream_hz), flush=True)
    if args.plan_out:
        args.plan_out.write_text(json.dumps(plan, indent=2, default=str))
    if not args.execute:
        print('Dry run: nothing was started, armed or moved. Add --execute to run on hardware.')
        return plan
    if not plan['trials']:
        raise SystemExit('no trials to run')
    execute(args, config, plan)
    return plan


if __name__ == '__main__':
    main()
