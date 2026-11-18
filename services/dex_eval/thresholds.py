"""Pass/fail thresholds: defaults from the protocol, self-referencing and self-calibration.

Three layers make the thresholds self-calibrated rather than absolute guesses:

1. Self-referenced metrics. ``sparc_ratio`` compares each trial's measured SPARC with
   the SPARC of its own commanded reference; audio metrics compare each trial with its
   own background level and hold click rate; stick-slip spikes are defined relative to
   the trial's own reference speed.
2. Calibration from a reference condition (``calibrate``). Given trials of an ideal
   or reference condition (for example a simulated ideal tracker, or a replay of the
   reference), each threshold is relaxed to that condition's 95th percentile (5th for
   lower bounds) times a margin, so no threshold sits below the rig's own sensing floor.
   Calibration only relaxes defaults; it never tightens them.
3. Operator overrides from a JSON file (``load``).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path

import numpy as np

OPS = {'<=': lambda a, b: a <= b, '<': lambda a, b: a < b,
       '>=': lambda a, b: a >= b, '>': lambda a, b: a > b}


@dataclass(frozen=True)
class Threshold:
    key: str
    op: str
    value: float
    description: str
    source: str = 'default'

    def passes(self, value):
        """True/False, or None when the value is missing or not finite."""
        if value is None or isinstance(value, bool):
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return bool(OPS[self.op](value, self.value))

    def text(self):
        return f'{self.key} {self.op} {self.value:g}'


DEFAULTS = (
    Threshold('sparc_ratio', '<=', 1.15,
              'Hand SPARC within 15% of the SPARC of the trial\'s own commanded reference'),
    Threshold('stick_slip_count', '<=', 1, 'At most one stick-slip event per trial'),
    Threshold('hold_p2p_max_rad', '<', 0.004, 'Hold peak-to-peak below 0.004 rad over every 1 s window'),
    Threshold('hold_oscillation_events', '<=', 0, 'No >=3-reversal oscillation event while holding'),
    Threshold('tracking_rms_motion_rad', '<', 0.03, 'RMS tracking error during commanded motion below 0.03 rad'),
    Threshold('current_frac_near_cap', '<', 0.05, 'Under 5% of samples with a joint within 5% of the current cap'),
    Threshold('temperature_rise_c', '<', 5.0, 'Joint MCU temperature rise below 5 C'),
    Threshold('timing_rate_ratio', '>=', 0.95, 'Achieved command rate at least 95% of nominal'),
    Threshold('timing_max_gap_periods', '<', 3.0, 'Every command interval shorter than 3 nominal periods'),
    Threshold('audio_motion_above_background_db', '<=', 10.0,
              'A-weighted level during motion at most 10 dB above the trial background'),
    Threshold('audio_excess_clicks_motion', '<', 1.0,
              'No more clicks during motion than the trial\'s own hold click rate predicts'),
    Threshold('arm_sparc_ratio', '<=', 1.15, 'Arm SPARC within 15% of its commanded reference'),
)

COUNT_KEYS = {'stick_slip_count', 'hold_oscillation_events', 'audio_excess_clicks_motion'}


def check(summary, thresholds=DEFAULTS):
    """{key: {'value', 'op', 'threshold', 'pass', 'source'}} for every threshold."""
    out = {}
    for threshold in thresholds:
        value = summary.get(threshold.key)
        out[threshold.key] = {'value': value, 'op': threshold.op, 'threshold': threshold.value,
                              'pass': threshold.passes(value), 'source': threshold.source}
    return out


def calibrate(thresholds, summaries, *, label='reference', quantile=0.95, margin=1.10, min_trials=3):
    """Relax each threshold to what the reference trials achieve (never tighten)."""
    out = []
    for threshold in thresholds:
        values = np.asarray([s.get(threshold.key) for s in summaries
                             if isinstance(s.get(threshold.key), (int, float))
                             and not isinstance(s.get(threshold.key), bool)], dtype=float)
        values = values[np.isfinite(values)]
        if values.size < min_trials:
            out.append(threshold)
            continue
        upper = threshold.op in ('<=', '<')
        if upper:
            floor = float(np.quantile(values, quantile))
            floor = floor * margin if floor > 0 else floor / margin
            if threshold.key in COUNT_KEYS:
                floor = float(math.ceil(floor))
            value = max(threshold.value, floor)
        else:
            floor = float(np.quantile(values, 1 - quantile))
            floor = floor / margin if floor > 0 else floor * margin
            value = min(threshold.value, floor)
        if value != threshold.value:
            out.append(replace(threshold, value=value,
                               source=f'calibrated from {label} (n={values.size}, q={quantile:g}, x{margin:g})'))
        else:
            out.append(replace(threshold, source=f'default (checked against {label}, n={values.size})'))
    return tuple(out)


def load(path, base=DEFAULTS):
    """Apply overrides from JSON: {key: value} or {key: {"op": ..., "value": ..., "description": ...}}.

    A key absent from base adds a new threshold (op and value required); null removes one.
    """
    data = json.loads(Path(path).read_text())
    if isinstance(data, list):
        data = {item['key']: item for item in data}
    current = {t.key: t for t in base}
    for key, spec in data.items():
        if spec is None:
            current.pop(key, None)
            continue
        if isinstance(spec, (int, float)):
            spec = {'value': spec}
        old = current.get(key)
        op = spec.get('op', old.op if old else None)
        if op not in OPS or 'value' not in spec and old is None:
            raise ValueError(f'threshold {key}: op must be one of {list(OPS)} and value is required')
        current[key] = Threshold(key=key, op=op, value=float(spec.get('value', old.value if old else 0)),
                                 description=spec.get('description', old.description if old else key),
                                 source=f'file {Path(path).name}')
    return tuple(current.values())


def to_json(thresholds):
    return [asdict(t) for t in thresholds]
