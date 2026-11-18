"""Comparison report: aggregate trials into a Markdown report with tables and PNG plots.

Inputs are results.jsonl files from the runner, run directories, or any directory of
trace directories (for example wuji_motion_battery output). Trials are grouped by
primitive (meta.skill; rows are conditions) or, for battery traces, by item with
meta.controller as rows. Variants are ranked by success rate (Wilson 95% CI), then by a
smoothness metric (median with a percentile-bootstrap 95% CI). Conditions whose median
fails a self-calibrated threshold are flagged.
"""
from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re

import numpy as np

from .evaluate import SUMMARY_METRICS, EvalConfig, clean, evaluate_trial
from .outcome import outcome as task_outcome
from .stats import bootstrap_ci, holm, mann_whitney, wilson
from .thresholds import DEFAULTS, calibrate
from .trace import STATUSES, TraceError, find_trial_dirs, is_trial_dir, load_trial

NAN = float('nan')
BATTERY_SKILLS = ('battery', 'wuji_motion_battery', 'motion_battery')
RANK_TABLE = ('sparc', 'sparc_ratio', 'stick_slip_count', 'tracking_rms_motion_rad', 'hold_p2p_max_rad',
              'current_frac_near_cap', 'audio_clicks_motion')
RATIO_KEYS = ('sparc_ratio', 'arm_sparc_ratio')
SECONDARY = ('sparc_ref', 'sparc_joint_median', 'fingertip_sparc', 'ldlj', 'rms_jerk', 'tracking_max_rad',
             'tracking_final_max_rad', 'tracking_lag_s', 'overshoot_max_rad', 'stick_slip_peak_speed', 'burst_count',
             'hold_oscillation_events', 'hold_reversals_per_s', 'current_rms_a', 'current_peak_a',
             'temperature_rise_c', 'timing_rate_ratio', 'timing_jitter_ms', 'timing_max_gap_periods',
             'audio_laeq_motion_db', 'audio_motion_above_background_db', 'audio_excess_clicks_motion',
             'audio_dominant_hz', 'arm_sparc', 'arm_sparc_ratio', 'arm_tracking_rms_rad', 'duration_s')

# Chart tokens (dataviz reference palette, light mode).
SURFACE, INK, INK_2, MUTED, GRID, AXIS = '#fcfcfb', '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7'
SERIES_1, CRITICAL = '#2a78d6', '#d03b3b'


@dataclass
class TrialRecord:
    path: Path
    meta: dict
    line: dict | None
    evaluation: dict | None
    error: str | None = None

    @property
    def result(self):
        if self.line and isinstance(self.line.get('result'), dict):
            return self.line['result']
        return self.meta.get('result') if isinstance(self.meta.get('result'), dict) else None

    @property
    def summary(self):
        if self.evaluation:
            return self.evaluation['summary']
        verdict = task_outcome(self.result, self.meta)
        return {'success': verdict['success'], 'status': verdict['status'], 'ok': verdict['ok']}

    def field(self, key):
        if self.line and self.line.get(key) is not None:
            return self.line[key]
        return self.meta.get(key)

    @property
    def condition(self):
        explicit = self.field('condition_id')
        if explicit:
            return str(explicit)
        skill, variant = self.field('skill'), self.field('variant')
        return f'{skill}/{variant}' if skill and variant else str(variant or skill or self.path.name)


@dataclass
class Row:
    label: str
    trials: list
    stats: dict = field(default_factory=dict)


# --------------------------------------------------------------------------- collection

def _results_files(path):
    path = Path(path)
    if path.is_file():
        return [path]
    return sorted(path.rglob('results*.jsonl'))


def collect(inputs, config: EvalConfig | None = None, *, recompute=False, log=None):
    """TrialRecords for every trial referenced by results files or found as a trace directory."""
    config = config or EvalConfig()
    seen, records = set(), []

    def add(trial_dir, line):
        trial_dir = Path(trial_dir).resolve()
        if trial_dir in seen:
            return
        seen.add(trial_dir)
        meta, evaluation, error = {}, None, None
        try:
            meta = json.loads((trial_dir / 'meta.json').read_text())
        except (OSError, ValueError) as exc:
            error = f'meta.json: {exc}'
        result = (line or {}).get('result') if line else meta.get('result')
        if error is None:
            try:
                evaluation = evaluate_trial(load_trial(trial_dir), config, result, cache=True, recompute=recompute)
            except TraceError as exc:
                error = '; '.join(exc.problems)
            except Exception as exc:  # Keep the outcome even when a metric fails on odd data.
                error = f'{type(exc).__name__}: {exc}'
        if error and log:
            log(f'[dex-eval] {trial_dir}: {error}')
        records.append(TrialRecord(trial_dir, meta, line, evaluation, error))

    for item in inputs:
        item = Path(item)
        for results in _results_files(item):
            for raw in results.read_text().splitlines():
                if raw.strip():
                    line = json.loads(raw)
                    if line.get('trace_dir'):
                        add(results.parent / line['trace_dir'], line)
        if item.is_dir():
            for trial_dir in find_trial_dirs(item) if not is_trial_dir(item) else [item]:
                add(trial_dir, None)
    return records


# --------------------------------------------------------------------------- grouping and statistics

def is_battery(record):
    skill = record.field('skill')
    return not skill or str(skill) in BATTERY_SKILLS or str(skill).startswith('battery')


def battery_item(record, key=None):
    params = record.meta.get('params') or {}
    if key:
        value = params.get(key, record.meta.get(key))
        return str(value) if value is not None else 'all'
    for candidate in ('item', 'motion', 'battery_item', 'test'):
        if params.get(candidate) is not None:
            return str(params[candidate])
    return str(record.field('variant') or 'all')


def group(records, mode='auto', item_key=None):
    """{group label: {row label: [TrialRecord]}} and the mode used ('skill' or 'controller')."""
    if mode == 'auto':
        mode = 'controller' if records and all(is_battery(r) for r in records) else 'skill'
    groups = {}
    for record in records:
        if mode == 'skill':
            key, row = str(record.field('skill') or 'unknown'), record.condition
        else:
            key, row = battery_item(record, item_key), str(record.meta.get('controller') or 'unknown')
        groups.setdefault(key, {}).setdefault(row, []).append(record)
    if mode == 'controller' and len(groups) > 1:
        combined = {}
        for rows in groups.values():
            for row, trials in rows.items():
                combined.setdefault(row, []).extend(trials)
        groups = {'all items': combined, **groups}
    return groups, mode


def _number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value))


def row_stats(trials, thresholds, *, seed=0):
    summaries = [t.summary for t in trials]
    known = [s.get('success') for s in summaries if s.get('success') is not None]
    successes = int(sum(bool(v) for v in known))
    rate, low, high = wilson(successes, len(known)) if known else (NAN, NAN, NAN)
    statuses = {}
    for s in summaries:
        status = s.get('status') if s.get('status') in STATUSES else ('none' if s.get('status') is None else 'other')
        statuses[status] = statuses.get(status, 0) + 1
    metrics = {}
    for key in SUMMARY_METRICS:
        values = [s.get(key) for s in summaries if _number(s.get(key))]
        if values:
            point, lo, hi = bootstrap_ci(values, seed=seed)
            metrics[key] = {'median': point, 'low': lo, 'high': hi, 'n': len(values), 'values': values}
    checks = {}
    for threshold in thresholds:
        per_trial = [threshold.passes(s.get(threshold.key)) for s in summaries]
        evaluated = [p for p in per_trial if p is not None]
        median = metrics.get(threshold.key, {}).get('median')
        checks[threshold.key] = {'median': median, 'pass': threshold.passes(median),
                                 'trials_passing': sum(evaluated), 'trials_evaluated': len(evaluated),
                                 'rule': threshold.text(), 'source': threshold.source}
    return {'n': len(trials), 'valid_traces': sum(t.evaluation is not None for t in trials),
            'success': {'k': successes, 'n': len(known), 'rate': rate, 'low': low, 'high': high},
            'statuses': statuses, 'metrics': metrics, 'checks': checks,
            'flags': [k for k, c in checks.items() if c['pass'] is False]}


def rank(rows, metric):
    """Rows sorted by success rate (desc), then the smoothness metric's median, then stick-slip and tracking."""
    better = SUMMARY_METRICS.get(metric, ('', '', 'higher'))[2]

    def median(row, key, sign):
        value = row.stats['metrics'].get(key, {}).get('median')
        if key in RATIO_KEYS and _number(value):
            value = max(value, 1.0)  # "As smooth as commanded" is the ideal; below 1 is usually noise bias.
        return sign * value if _number(value) else math.inf

    def key(row):
        rate = row.stats['success']['rate']
        return (-(rate if _number(rate) else -1.0), median(row, metric, -1 if better == 'higher' else 1),
                median(row, 'stick_slip_count', 1), median(row, 'tracking_rms_motion_rad', 1), row.label)
    ordered = sorted(rows, key=key)
    if ordered:
        best = ordered[0].stats
        for row in ordered:
            s, b = row.stats['success'], best['success']
            row.stats['success_overlaps_best'] = (None if not (_number(s['rate']) and _number(b['rate'])) else
                                                  bool(s['high'] >= b['low'] and b['high'] >= s['low']))
            m, bm = row.stats['metrics'].get(metric), best['metrics'].get(metric)
            row.stats['metric_overlaps_best'] = (
                None if not (m and bm and _number(m['low']) and _number(bm['low'])) else
                bool(m['high'] >= bm['low'] and bm['high'] >= m['low']))
    return ordered


def compare_baseline(rows, baseline, keys=('sparc', 'sparc_ratio', 'stick_slip_count')):
    """Mann-Whitney U of each row against the baseline row, Holm-adjusted within the group."""
    base = next((r for r in rows if r.label == baseline), None)
    if base is None:
        return {}
    tests, labels = [], []
    for row in rows:
        if row is base:
            continue
        for key in keys:
            a = row.stats['metrics'].get(key, {}).get('values', [])
            b = base.stats['metrics'].get(key, {}).get('values', [])
            tests.append(mann_whitney(a, b))
            labels.append((row.label, key))
    adjusted = holm(tests)
    out = {}
    for (label, key), p, q in zip(labels, tests, adjusted):
        out.setdefault(label, {})[key] = {'p': p, 'p_holm': q}
    return out


# --------------------------------------------------------------------------- formatting

def fmt(value, key=None, digits=3):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return 'n/a'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if key in ('stick_slip_count', 'burst_count', 'hold_oscillation_events', 'audio_clicks_motion',
               'audio_clicks', 'timing_gaps', 'timing_skipped_slots') and float(value).is_integer():
        return f'{int(value)}'
    if key in ('current_frac_near_cap',):
        return f'{100 * value:.1f}%'
    magnitude = abs(value)
    if magnitude != 0 and (magnitude < 1e-3 or magnitude >= 1e5):
        return f'{value:.2e}'
    return f'{value:.{digits}g}' if magnitude < 1 else f'{value:.{max(digits, 3)}g}'


def metric_cell(stats, key, ci=False):
    m = stats['metrics'].get(key)
    if not m:
        return 'n/a'
    text = fmt(m['median'], key)
    if ci and _number(m['low']):
        text += f' [{fmt(m["low"], key)}, {fmt(m["high"], key)}]'
    return text


def success_cell(stats):
    s = stats['success']
    if not s['n']:
        return 'n/a', 'n/a'
    return f'{s["k"]}/{s["n"]} ({100 * s["rate"]:.0f}%)', f'{100 * s["low"]:.0f}-{100 * s["high"]:.0f}%'


def md_table(header, rows):
    escape = lambda v: str(v).replace('|', '\\|').replace('\n', ' ')  # noqa: E731
    lines = ['| ' + ' | '.join(header) + ' |', '|' + '|'.join('---' for _ in header) + '|']
    lines += ['| ' + ' | '.join(escape(c) for c in row) + ' |' for row in rows]
    return '\n'.join(lines)


def file_slug(text):
    return re.sub(r'[^A-Za-z0-9_.-]+', '-', str(text)).strip('-')[:80] or 'group'


# --------------------------------------------------------------------------- plots

def _matplotlib():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': AXIS, 'axes.labelcolor': INK_2, 'axes.titlecolor': INK, 'text.color': INK,
        'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelcolor': INK_2, 'ytick.labelcolor': INK_2,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8, 'grid.linestyle': '-',
        'axes.spines.top': False, 'axes.spines.right': False, 'font.size': 9, 'axes.titlesize': 10,
        'axes.titlelocation': 'left', 'legend.frameon': False, 'lines.solid_capstyle': 'round',
    })
    return plt


HEADER_IN = 0.52  # Title and legend band above the plots (inches).


def _header(fig, title, handles, labels):
    """Title on the first line and the legend on the second, in a fixed band above the axes."""
    height = fig.get_figheight()
    fig.suptitle(title, x=0.01, y=1 - 0.08 / height, ha='left', va='top', fontsize=10, color=INK)
    unique = {}
    for handle, label in zip(handles, labels):
        unique.setdefault(label, handle)
    fig.legend(list(unique.values()), list(unique), loc='upper left', bbox_to_anchor=(0.005, 1 - 0.3 / height),
               ncol=len(unique), fontsize=8, frameon=False, handlelength=1.6, columnspacing=1.4)
    fig.tight_layout(rect=(0, 0, 1, 1 - HEADER_IN / height))


def plot_distribution(rows, metric, path, title, threshold=None):
    """Per-condition strip of trial values with the median and the reference median."""
    plt = _matplotlib()
    label, unit, better = SUMMARY_METRICS.get(metric, (metric, '', 'higher'))
    ref_key = {'sparc': 'sparc_ref', 'ldlj': 'ldlj_ref', 'fingertip_sparc': 'fingertip_sparc_ref'}.get(metric)
    shown = [r for r in rows if r.stats['metrics'].get(metric)]
    if not shown:
        return None
    fig, ax = plt.subplots(figsize=(7.5, HEADER_IN + 1.0 + 0.42 * len(shown)), dpi=130)
    rng = np.random.default_rng(0)
    for i, row in enumerate(shown):
        y = len(shown) - 1 - i
        values = np.asarray(row.stats['metrics'][metric]['values'])
        jitter = rng.uniform(-0.12, 0.12, values.size)
        ax.scatter(values, y + jitter, s=26, color=SERIES_1, alpha=0.75, linewidths=0.8, edgecolors=SURFACE,
                   zorder=3, label='trial' if i == 0 else None)
        ax.plot([row.stats['metrics'][metric]['median']] * 2, [y - 0.28, y + 0.28], color=INK, lw=2,
                solid_capstyle='round', zorder=4, label='median' if i == 0 else None)
        reference = row.stats['metrics'].get(ref_key, {}).get('median') if ref_key else None
        if _number(reference):
            ax.scatter([reference], [y], marker='D', s=34, facecolors=SURFACE, edgecolors=MUTED, linewidths=1.5,
                       zorder=4, label='commanded reference (median)' if i == 0 else None)
    ax.set_yticks(range(len(shown)))
    ax.set_yticklabels([r.label for r in reversed(shown)])
    ax.grid(axis='y', visible=False)
    direction = 'closer to 0 is smoother' if metric in ('sparc', 'ldlj', 'fingertip_sparc') else (
        f'{better} is better' if better else '')
    ax.set_xlabel(f'{label}{f" ({unit})" if unit else ""}; {direction}')
    ax.set_ylim(-0.6, len(shown) - 0.4)
    if threshold is not None:
        ax.axvline(threshold.value, color=INK_2, lw=1, zorder=2, label=f'pass threshold ({threshold.op} {threshold.value:g})')
    _header(fig, title, *ax.get_legend_handles_labels())
    fig.savefig(path)
    plt.close(fig)
    return path


def representative(trials, metric='sparc'):
    """The valid trial whose metric is closest to the condition median (first valid one otherwise)."""
    valid = [t for t in trials if t.evaluation and t.evaluation.get('hands')]
    if not valid:
        return None
    values = [(t, t.summary.get(metric)) for t in valid if _number(t.summary.get(metric))]
    if not values:
        return valid[0]
    median = float(np.median([v for _, v in values]))
    return min(values, key=lambda item: abs(item[1] - median))[0]


def plot_trajectories(record, path, title, max_joints=4):
    """q_ref vs q_meas for the most informative joints of one trial, stick-slip events marked."""
    plt = _matplotlib()
    trial = load_trial(record.path, validate=False)
    hands = record.evaluation['hands']
    candidates = []
    for side, hand in trial.hands.items():
        evaluation = hands.get(side) or {}
        events = (evaluation.get('stick_slip') or {}).get('events') or []
        counts = {}
        for event in events:
            counts[event['joint']] = counts.get(event['joint'], 0) + 1
        for name in evaluation.get('moving_joints') or []:
            j = trial.meta.get('joint_names', [])[:20].index(name) if name in trial.meta.get('joint_names', []) else None
            if j is None:
                continue
            excursion = float(np.nanmax(hand.q_ref[:, j]) - np.nanmin(hand.q_ref[:, j]))
            candidates.append((counts.get(name, 0), excursion, side, j, name, events))
    if not candidates:
        return None
    candidates.sort(key=lambda c: (-c[0], -c[1]))
    chosen = candidates[:max_joints]
    t0 = trial.t0
    fig, axes = plt.subplots(len(chosen), 2, figsize=(9.0, HEADER_IN + 0.45 + 1.6 * len(chosen)), dpi=130,
                             squeeze=False, gridspec_kw={'width_ratios': [2.2, 1]})
    for row, (count, _, side, j, name, events) in enumerate(chosen):
        hand = trial.hands[side]
        t = hand.t - t0
        joint_events = [e for e in events if e['joint'] == name]
        times = np.asarray([e['t'] for e in joint_events]) - t0
        values = np.interp(times + t0, hand.t, np.nan_to_num(hand.q_meas[:, j])) if joint_events else []
        if joint_events:  # Zoom on the strongest event; otherwise on the fastest reference motion.
            centre = joint_events[int(np.argmax([e['peak_speed'] for e in joint_events]))]['t'] - t0
        else:
            centre = t[int(np.argmax(np.abs(np.gradient(hand.q_ref[:, j], hand.t))))]
        for col, ax in enumerate(axes[row]):
            ax.plot(t, hand.q_ref[:, j], color=MUTED, lw=1.6, label='reference sent (q_ref)')
            ax.plot(t, hand.q_meas[:, j], color=SERIES_1, lw=1.3, label='measured (q_meas)')
            if joint_events:
                ax.scatter(times, values, s=40 if len(times) <= 30 or col else 20, color=CRITICAL,
                           edgecolors=SURFACE, linewidths=1.2, zorder=5, label='stick-slip event')
            if col:
                window = (t >= centre - 0.25) & (t <= centre + 0.25)
                ax.set_xlim(centre - 0.25, centre + 0.25)
                visible = np.concatenate([hand.q_ref[window, j], hand.q_meas[window, j]])
                visible = visible[np.isfinite(visible)]
                if visible.size:
                    pad = max(1e-3, 0.15 * np.ptp(visible))
                    ax.set_ylim(visible.min() - pad, visible.max() + pad)
                ax.set_title('zoom, 0.5 s', fontsize=8, color=MUTED)
            else:
                ax.set_title(f'{side} {name}' + (f'  ({count} stick-slip events)' if count else ''),
                             fontsize=9, color=INK_2)
                ax.set_ylabel('rad')
                ax.axvspan(centre - 0.25, centre + 0.25, color=GRID, alpha=0.6, lw=0, zorder=0)
    axes[-1, 0].set_xlabel('time from trial start (s)')
    axes[-1, 1].set_xlabel('time (s)')
    handles, labels = [], []
    for ax in axes[:, 0]:
        h, l = ax.get_legend_handles_labels()
        handles += h
        labels += l
    _header(fig, title, handles, labels)
    fig.savefig(path)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- report

@dataclass
class ReportOptions:
    group_by: str = 'auto'
    item_key: str | None = None
    rank_metric: str | None = None          # default: sparc (skills) or sparc_ratio (battery)
    baseline: str | None = None             # row label to test others against
    calibrate_from: str | None = None       # row label whose trials calibrate thresholds
    thresholds: tuple = DEFAULTS
    ratings: list = field(default_factory=list)
    plots: bool = True
    seed: int = 0


def build(records, out_dir, options: ReportOptions | None = None, *, inputs=()):
    """Write report.md, summary.json, trials.csv and plots/ into out_dir; return the report path."""
    options = options or ReportOptions()
    out_dir = Path(out_dir)
    plots_dir = out_dir / 'plots'
    out_dir.mkdir(parents=True, exist_ok=True)
    if options.plots:
        plots_dir.mkdir(exist_ok=True)
    groups, mode = group(records, options.group_by, options.item_key)
    metric = options.rank_metric or ('sparc' if mode == 'skill' else 'sparc_ratio')
    thresholds = tuple(options.thresholds)
    if options.calibrate_from:
        reference = [r.summary for rows in groups.values() for label, trials in rows.items()
                     if label == options.calibrate_from for r in trials]
        thresholds = calibrate(thresholds, reference, label=f'condition {options.calibrate_from}')
    backends = sorted({str(r.field('backend')) for r in records if r.field('backend')})
    seeds = sorted({r.field('seed') for r in records if r.field('seed') is not None})
    lines = ['# Dexterous skill evaluation report', '',
             f'Generated {datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")} from '
             + ', '.join(f'`{p}`' for p in inputs) + '.', '',
             f'- Trials: {len(records)} ({sum(r.evaluation is not None for r in records)} with valid traces); '
             f'backends: {", ".join(backends) or "n/a"}; seeds: {", ".join(map(str, seeds)) or "n/a"}.',
             f'- Grouping: by {"primitive (meta.skill), rows are conditions" if mode == "skill" else "battery item, rows are meta.controller"}.',
             f'- Ranking: success rate (Wilson 95% CI), then median {SUMMARY_METRICS[metric][0]} '
             f'({SUMMARY_METRICS[metric][2]} is better; percentile-bootstrap 95% CI), then stick-slip, then '
             'tracking error. "Overlaps best" means the 95% intervals overlap the top row: the order is not '
             'statistically settled.',
             '- SPARC and LDLJ are negative; closer to 0 is smoother. SPARC / reference is 1.0 when the hand '
             'moved as smoothly as it was commanded; larger is worse. Metric definitions: docs/dex-eval.md.', '',
             '## Thresholds', '',
             md_table(['Metric', 'Rule', 'Source', 'Meaning'],
                      [[t.key, f'{t.op} {t.value:g}', t.source, t.description] for t in thresholds]), '']
    summary = {'generated_at': datetime.now(timezone.utc).isoformat(), 'mode': mode, 'rank_metric': metric,
               'thresholds': [t.__dict__ for t in thresholds], 'groups': {}}
    flagged_total = []
    for group_label, rows_by_label in groups.items():
        rows = [Row(label, trials) for label, trials in rows_by_label.items()]
        for row in rows:
            row.stats = row_stats(row.trials, thresholds, seed=options.seed)
        rows = rank(rows, metric)
        baseline = compare_baseline(rows, options.baseline) if options.baseline else {}
        heading = (f'Primitive: {group_label}' if mode == 'skill' else
                   ('Battery: all items (per controller)' if group_label == 'all items' else f'Battery item: {group_label}'))
        lines += [f'## {heading}', '', '### Ranking', '']
        table = []
        first = 'Condition' if mode == 'skill' else 'Controller'
        for i, row in enumerate(rows, 1):
            s = row.stats
            success, ci = success_cell(s)
            controller = sorted({str(t.meta.get('controller')) for t in row.trials if t.meta.get('controller')})
            table.append([i, row.label, *([', '.join(controller) or 'n/a'] if mode == 'skill' else []), s['n'],
                          success, ci, fmt(s.get('success_overlaps_best')), metric_cell(s, metric, ci=True),
                          fmt(s.get('metric_overlaps_best')),
                          *[metric_cell(s, k) for k in RANK_TABLE if k != metric],
                          ', '.join(s['flags']) or 'none'])
        lines.append(md_table(['Rank', first, *(['Controller'] if mode == 'skill' else []), 'n',
                               'Success', 'Success 95% CI', 'Overlaps best', f'{metric} median [95% CI]',
                               'Overlaps best', *[k for k in RANK_TABLE if k != metric], 'Flags'], table))
        lines += ['', '### Outcomes by status', '']
        status_keys = [*STATUSES, 'other', 'none']
        lines.append(md_table([first, *status_keys],
                              [[row.label, *[row.stats['statuses'].get(k, 0) for k in status_keys]] for row in rows]))
        secondary = [k for k in SECONDARY if any(k in row.stats['metrics'] for row in rows)]
        if secondary:
            lines += ['', '### Secondary metrics (medians)', '']
            lines.append(md_table(['Metric', *[row.label for row in rows]],
                                  [[f'{SUMMARY_METRICS[k][0]}{f" ({SUMMARY_METRICS[k][1]})" if SUMMARY_METRICS[k][1] else ""}',
                                    *[metric_cell(row.stats, k) for row in rows]] for k in secondary]))
        if baseline:
            lines += ['', f'### Against baseline `{options.baseline}` (Mann-Whitney U, Holm-adjusted)', '']
            keys = sorted({k for v in baseline.values() for k in v})
            lines.append(md_table(['Condition', *[f'{k} p (Holm)' for k in keys]],
                                  [[label, *[fmt(v.get(k, {}).get('p_holm')) for k in keys]]
                                   for label, v in baseline.items()]))
        flagged = [row for row in rows if row.stats['flags']]
        lines += ['', '### Threshold flags', '']
        if flagged:
            for row in flagged:
                details = []
                for key in row.stats['flags']:
                    c = row.stats['checks'][key]
                    details.append(f'{key} median {fmt(c["median"], key)} fails `{c["rule"]}` '
                                   f'({c["trials_passing"]}/{c["trials_evaluated"]} trials pass)')
                lines.append(f'- **{row.label}**: ' + '; '.join(details))
                flagged_total.append((group_label, row.label))
        else:
            lines.append('- No condition fails a threshold on its median.')
        if options.plots:
            lines += ['', '### Plots', '']
            name = file_slug(group_label)
            limits = {t.key: t for t in thresholds}
            plotted = plot_distribution(rows, metric, plots_dir / f'{name}_{metric}.png',
                                        f'{heading}: {SUMMARY_METRICS[metric][0]} per trial', limits.get(metric))
            if plotted:
                lines.append(f'![{metric} distribution]({plotted.relative_to(out_dir)})')
                lines.append('')
            if metric != 'sparc':
                plotted = plot_distribution(rows, 'sparc', plots_dir / f'{name}_sparc.png',
                                            f'{heading}: SPARC per trial')
                if plotted:
                    lines += [f'![sparc distribution]({plotted.relative_to(out_dir)})', '']
            for row in rows:
                record = representative(row.trials)
                if record is None:
                    continue
                try:
                    plotted = plot_trajectories(
                        record, plots_dir / f'{name}_{file_slug(row.label)}_trajectory.png',
                        f'{row.label}: representative trial {record.path.name} '
                        f'(SPARC {fmt(record.summary.get("sparc"))})')
                except Exception as exc:
                    lines.append(f'- Trajectory plot for {row.label} failed: {type(exc).__name__}: {exc}')
                    continue
                if plotted:
                    lines += [f'![{row.label} trajectories]({plotted.relative_to(out_dir)})', '']
        summary['groups'][group_label] = {row.label: clean({k: v for k, v in row.stats.items()}) for row in rows}
        for row_stats_value in summary['groups'][group_label].values():
            for m in row_stats_value['metrics'].values():
                m.pop('values', None)
        lines.append('')
    if options.ratings:
        from .rating import ranking_table
        lines += ['## Naturalness (blind pairwise ratings)', '']
        for session in options.ratings:
            lines += [ranking_table(session), '']
    errors = [r for r in records if r.error]
    lines += ['## Trials', '', 'Per-trial values: `trials.csv`. Machine-readable aggregates: `summary.json`.', '']
    if errors:
        lines += ['Trials without usable traces (outcome still counted):', '']
        lines += [f'- `{r.path}`: {r.error}' for r in errors]
        lines.append('')
    if flagged_total:
        lines += ['Conditions flagged: ' + ', '.join(f'{g} / {r}' for g, r in flagged_total), '']
    path = out_dir / 'report.md'
    path.write_text('\n'.join(lines) + '\n')
    (out_dir / 'summary.json').write_text(json.dumps(clean(summary), indent=2, allow_nan=False) + '\n')
    write_csv(records, out_dir / 'trials.csv', mode, options.item_key)
    return path


def write_csv(records, path, mode, item_key=None):
    keys = list(SUMMARY_METRICS)
    with open(path, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['trial_dir', 'skill', 'condition', 'variant', 'controller', 'item', 'backend', 'success',
                         'status', 'error', *keys])
        for r in records:
            s = r.summary
            writer.writerow([str(r.path), r.field('skill'), r.condition, r.field('variant'), r.meta.get('controller'),
                             battery_item(r, item_key) if mode == 'controller' else '', r.field('backend'),
                             s.get('success'), s.get('status'), r.error or '',
                             *['' if not _number(s.get(k)) else s.get(k) for k in keys]])
