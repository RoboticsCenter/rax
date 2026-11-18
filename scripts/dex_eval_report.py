"""Compare dexterous skill variants (or low-level controllers) from recorded trials.

    python scripts/dex_eval_report.py RUN_DIR [MORE ...] [--out DIR]
    python scripts/dex_eval_report.py results.jsonl --baseline grasp/scripted_v1
    python scripts/dex_eval_report.py BATTERY_DIR --group-by controller [--item-key item]

Inputs: runner output directories, results*.jsonl files, or any directory of trace
directories (for example wuji_motion_battery output, grouped by meta.controller).
Writes report.md, summary.json, trials.csv and plots/*.png (default <first input>/report).
Each trial's trace metrics are cached in <trial>/metrics.json (--recompute refreshes).
"""
import argparse
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.dex_eval.audio import AudioConfig  # noqa: E402
from services.dex_eval.evaluate import SUMMARY_METRICS, EvalConfig  # noqa: E402
from services.dex_eval.metrics import MetricConfig  # noqa: E402
from services.dex_eval.report import ReportOptions, build, collect  # noqa: E402
from services.dex_eval import thresholds as threshold_module  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--out', type=Path, help='report directory (default <first input>/report)')
    parser.add_argument('--group-by', choices=('auto', 'skill', 'controller'), default='auto',
                        help='auto: controller when every trial is a battery trace, else skill')
    parser.add_argument('--item-key', help='controller mode: params/meta key naming the battery item')
    parser.add_argument('--rank-metric', choices=sorted(SUMMARY_METRICS),
                        help='smoothness tiebreak after success (default sparc; sparc_ratio for batteries)')
    parser.add_argument('--baseline', help='row label to test the others against (Mann-Whitney, Holm)')
    parser.add_argument('--thresholds', type=Path, help='JSON threshold overrides')
    parser.add_argument('--calibrate-from', help='row label of an ideal/reference condition that relaxes '
                                                 'thresholds to its 95th percentile (never tightens)')
    parser.add_argument('--ratings', type=Path, action='append', default=[],
                        help='rating session directory from dex_eval_rate.py (naturalness section)')
    parser.add_argument('--current-cap', type=float, default=0.8, help='current cap (A) when meta has none')
    parser.add_argument('--smooth-cutoff', type=float, default=20.0, help='position low-pass before SPARC (Hz)')
    parser.add_argument('--resample-hz', type=float, help='analysis rate (default: each trace\'s median rate)')
    parser.add_argument('--wuji-description', help='wuji-description checkout for fingertip FK')
    parser.add_argument('--no-fingertip', action='store_true', help='skip fingertip FK')
    parser.add_argument('--audio-background', help='background recording for the acoustic metrics')
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--recompute', action='store_true', help='ignore cached metrics.json')
    parser.add_argument('--seed', type=int, default=0, help='bootstrap seed')
    args = parser.parse_args(argv)

    config = EvalConfig(metrics=MetricConfig(current_cap_a=args.current_cap, smooth_cutoff_hz=args.smooth_cutoff),
                        audio=AudioConfig(), resample_hz=args.resample_hz, fingertip=not args.no_fingertip,
                        wuji_description=args.wuji_description, audio_background=args.audio_background)
    records = collect(args.inputs, config, recompute=args.recompute, log=lambda m: print(m, file=sys.stderr))
    if not records:
        parser.error('no trials found in the inputs')
    thresholds = threshold_module.load(args.thresholds) if args.thresholds else threshold_module.DEFAULTS
    out = args.out or (args.inputs[0] if args.inputs[0].is_dir() else args.inputs[0].parent) / 'report'
    report = build(records, out, ReportOptions(group_by=args.group_by, item_key=args.item_key,
                                               rank_metric=args.rank_metric, baseline=args.baseline,
                                               calibrate_from=args.calibrate_from, thresholds=thresholds,
                                               ratings=args.ratings, plots=not args.no_plots, seed=args.seed),
                   inputs=[str(p) for p in args.inputs])
    print(report)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
