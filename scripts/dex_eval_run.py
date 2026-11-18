"""Run randomized, interleaved trials of dexterous skill variants and record traces and results.

    python scripts/dex_eval_run.py PLAN.json                       # mock backend (default)
    python scripts/dex_eval_run.py PLAN.json --backend sim [--sim-robot KEY]
    python scripts/dex_eval_run.py PLAN.json --dry-run             # print the seeded schedule only
    python scripts/dex_eval_run.py PLAN.json --real --session <session dir>

PLAN.json: {"name", "trials" (per condition, >= 10 recommended), "seed", "sides",
"skill_paths", "backend_options", "nominal_rate_hz", "notes", "conditions": [{"skill",
"variant", "params", "controller", "entry", "library_name", "notes", "id"}]}. Params may
use {"uniform": [a, b]}, {"loguniform": [a, b]}, {"normal": [mu, sd]}, {"randint": [a, b]}
or {"choice": [...]} leaves, sampled per trial from the recorded seed.

Real robot (--real): connects to the session's already running robot service; it never
starts, enables or arms the robot. Arm the session yourself (scripts/arm_robot.py) with a
person at the emergency stop. Each condition needs a typed confirmation, every trial a
scene-reset prompt, and the operator gives each trial's success verdict.

Output (default <artifact root>/dex-eval/<name>-<UTC stamp>/): run.json (plan, seed,
schedule, git commit), results.jsonl (one line per trial), trials/<NNNN>_<skill>_<variant>/
(trace directory: hand_<side>.npz, optional arm_<side>.npz and audio.wav, meta.json).
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.dex_eval.evaluate import EvalConfig  # noqa: E402
from services.dex_eval.runner import (Operator, RealSession, Runner, load_plan, mock_session,  # noqa: E402
                                      session_root_for, sim_session)
from services.dex_eval.skills import library_loader  # noqa: E402


def options(text):
    if not text:
        return {}
    if text.startswith('@'):
        return json.loads(Path(text[1:]).read_text())
    return json.loads(text)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('plan', type=Path)
    parser.add_argument('--out', type=Path, help='output directory (must not already hold results)')
    parser.add_argument('--trials', type=int, help='override trials per condition')
    parser.add_argument('--seed', type=int, help='override the plan seed')
    parser.add_argument('--backend', choices=('mock', 'sim', 'real'), default=None,
                        help='default mock; real requires --real')
    parser.add_argument('--real', action='store_true',
                        help='run on the real robot (needs --session, an armed session and an operator)')
    parser.add_argument('--session', type=Path, help='real: the RAX session directory (operator-input.json)')
    parser.add_argument('--socket', type=Path, help='real: robot service socket (default: the session\'s)')
    parser.add_argument('--single-hand', action='store_true', help='real: single-arm session (no side sockets)')
    parser.add_argument('--sim-robot', help='sim: services.backends key (default: the first key containing "sim")')
    parser.add_argument('--backend-options', help='JSON object or @file merged into backend options (mock/sim)')
    parser.add_argument('--skill-path', type=Path, action='append', default=[], help='skill search directory')
    parser.add_argument('--skill-loader', help='module:function returning a skill main for a name')
    parser.add_argument('--trace', choices=('auto', 'service', 'poll', 'none'), default='auto',
                        help='auto: robot-service trace tools if listed, else poll telemetry')
    parser.add_argument('--trace-tools', help='START,STOP tool names of the robot service recorder')
    parser.add_argument('--poll-hz', type=float, default=100.0, help='fallback telemetry poll rate')
    parser.add_argument('--audio-device', help='ALSA device for 48 kHz audio capture with arecord, e.g. hw:2,0')
    parser.add_argument('--evaluate', action='store_true', help='compute metrics after each trial')
    parser.add_argument('--interactive', action='store_true', help='mock/sim: prompt for a reset before each trial')
    parser.add_argument('--dry-run', action='store_true', help='print the schedule and exit')
    args = parser.parse_args(argv)

    backend = args.backend or ('real' if args.real else 'mock')
    if backend == 'real' and not args.real:
        parser.error('--backend real requires the explicit --real flag')
    if args.real and backend != 'real':
        parser.error('--real cannot be combined with --backend mock/sim')
    if args.real and not args.dry_run:
        if not args.session:
            parser.error('--real requires --session <session dir>')
        if not sys.stdin.isatty():
            parser.error('--real needs an interactive terminal for operator confirmations')

    plan = load_plan(args.plan)
    if args.trials:
        plan.trials = args.trials
    if args.seed is not None:
        plan.seed = args.seed
    plan.skill_paths = [*plan.skill_paths, *map(str, args.skill_path)]
    if plan.trials < 10:
        print(f'[dex-eval] note: {plan.trials} trials per condition; the protocol asks for at least 10.')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    if args.out:
        out = args.out
    else:
        from services.storage_root import artifact_root
        out = artifact_root() / 'dex-eval' / f'{plan.name}-{stamp}'
    if (out / 'results.jsonl').exists():
        parser.error(f'{out} already holds results; choose a new --out')
    trace_tools = tuple(args.trace_tools.split(',')) if args.trace_tools else None
    if trace_tools and len(trace_tools) != 2:
        parser.error('--trace-tools takes START,STOP')

    if args.dry_run:
        record = Runner(plan, _Dry(backend), out, operator=Operator()).run(dry_run=True)
        print(json.dumps({'seed': record['seed'], 'schedule': record['schedule']}, indent=1))
        return 0

    if backend == 'mock':
        session = mock_session(session_root_for(out, f'{plan.name}-{stamp}'), plan, options(args.backend_options))
    elif backend == 'sim':
        session = sim_session(session_root_for(out, f'{plan.name}-{stamp}'), plan, args.sim_robot,
                              options(args.backend_options))
    else:
        session = RealSession(args.session, args.socket, bimanual=not args.single_hand)
    operator = Operator() if backend == 'real' or args.interactive else None
    loader = library_loader(args.skill_loader) if args.skill_loader else None
    try:
        runner = Runner(plan, session, out, operator=operator, interactive=args.interactive, trace=args.trace,
                        trace_tools=trace_tools, poll_hz=args.poll_hz, audio_device=args.audio_device,
                        loader=loader, evaluate=args.evaluate, eval_config=EvalConfig(), argv=sys.argv)
        record = runner.run()
    finally:
        session.close()
    print(f'[dex-eval] {record["trials_completed"]} trials -> {out}'
          + (f' (stopped: {record["stopped_by"]})' if record['stopped_by'] else ''))
    print(f'[dex-eval] report: python scripts/dex_eval_report.py {out}')
    return 0


class _Dry:
    """Session stand-in for --dry-run (no backend is constructed)."""

    def __init__(self, kind):
        self.kind = kind


if __name__ == '__main__':
    raise SystemExit(main())
