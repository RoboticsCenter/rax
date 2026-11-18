"""Randomized, interleaved trials of (skill, variant, params) conditions through the RAX controller runtime.

Each trial calls the skill's ``main(ctx, **params)`` with a ctx that has the runtime
``Context`` API (``call``, ``value``, ``request``, ``images``):

* ``mock`` / ``sim``: an in-process ``services.controller.frontend.Frontend`` around a
  backend (the mock rig, or a simulator registered in ``services.backends``), wrapped
  by ``InProcessContext``, which subclasses the runtime Context and replaces only its
  socket transport. Every call is audited in the frontend session's mcp.jsonl.
* ``real``: the runtime ``Context`` connected to an already running RAX robot service
  socket. The runner never starts, enables or arms the robot. It refuses to run unless
  the session is armed by the operator, asks for an interactive confirmation of each
  condition, prompts for a scene reset before every trial and asks for the success verdict.

Design: ``trials`` blocks; each block runs every condition once in a fresh random order
(randomized, interleaved, balanced). The seed is recorded in run.json and every result.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import random
import re
import subprocess
import threading
import time
import traceback

from .outcome import normalize_result, outcome
from .recorder import AudioRecorder, PollingRecorder, ServiceRecorder, backend_status, hand_service_status
from .skills import library_loader, resolve
from .trace import SIDES, default_meta, validate_trial_dir, write_meta

from auto_research_agent.api.runtime import Context  # The controller runtime ctx used by agent code.

PROJECT = Path(__file__).resolve().parents[2]

DISTRIBUTIONS = ('uniform', 'loguniform', 'normal', 'randint', 'choice')


# --------------------------------------------------------------------------- plan

@dataclass
class Condition:
    skill: str
    variant: str
    params: dict = field(default_factory=dict)
    controller: str | None = None
    entry: str | None = None
    library_name: str | None = None
    notes: str | None = None
    id: str | None = None

    def __post_init__(self):
        if not isinstance(self.skill, str) or not self.skill:
            raise ValueError('condition skill must be a non-empty string')
        if not isinstance(self.variant, str) or not self.variant:
            raise ValueError('condition variant must be a non-empty string')
        if not isinstance(self.params, dict):
            raise ValueError(f'{self.skill}/{self.variant}: params must be an object')
        self.id = self.id or f'{self.skill}/{self.variant}'


@dataclass
class Plan:
    conditions: list
    trials: int = 10
    seed: int | None = None
    name: str = 'dex-eval'
    sides: list = field(default_factory=lambda: ['left', 'right'])
    skill_paths: list = field(default_factory=list)
    backend_options: dict = field(default_factory=dict)
    nominal_rate_hz: float | None = None
    notes: str | None = None

    def __post_init__(self):
        self.conditions = [c if isinstance(c, Condition) else Condition(**c) for c in self.conditions]
        if not self.conditions:
            raise ValueError('plan has no conditions')
        ids = [c.id for c in self.conditions]
        duplicates = sorted({i for i in ids if ids.count(i) > 1})
        if duplicates:
            raise ValueError(f'duplicate condition ids {duplicates}; give each an explicit "id"')
        if type(self.trials) is not int or self.trials < 1:
            raise ValueError('trials must be a positive integer')
        if any(s not in SIDES for s in self.sides):
            raise ValueError(f'sides must be within {SIDES}')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', self.name):
            raise ValueError('plan name may contain letters, digits, _ . - only')

    def to_dict(self):
        return {**asdict(self), 'conditions': [asdict(c) for c in self.conditions]}


def load_plan(path):
    """Plan from JSON (or YAML when PyYAML is installed and the suffix is .yaml/.yml)."""
    path = Path(path)
    text = path.read_text()
    if path.suffix in ('.yaml', '.yml'):
        import yaml
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    if not isinstance(data, dict):
        raise ValueError('plan must be an object with "conditions"')
    known = set(Plan.__dataclass_fields__)
    unknown = set(data) - known
    if unknown:
        raise ValueError(f'unknown plan keys {sorted(unknown)}; allowed {sorted(known)}')
    return Plan(**data)


def schedule(n_conditions, trials, seed):
    """[{'index', 'block', 'condition'}]: every block runs each condition once, in a seeded random order."""
    rng = random.Random(seed)
    items = []
    for block in range(trials):
        order = list(range(n_conditions))
        rng.shuffle(order)
        for condition in order:
            items.append({'index': len(items), 'block': block, 'condition': condition})
    return items


def sample_params(value, rng):
    """Resolve {"uniform": [a, b]}, {"loguniform": [a, b]}, {"normal": [mu, sd]}, {"randint": [a, b]}
    and {"choice": [...]} leaves (recursively) with rng; other values pass through."""
    if isinstance(value, dict):
        if len(value) == 1 and next(iter(value)) in DISTRIBUTIONS:
            kind, spec = next(iter(value.items()))
            if kind == 'choice':
                return sample_params(rng.choice(list(spec)), rng)
            a, b = spec
            if kind == 'uniform':
                return rng.uniform(a, b)
            if kind == 'loguniform':
                import math
                return math.exp(rng.uniform(math.log(a), math.log(b)))
            if kind == 'normal':
                return rng.gauss(a, b)
            return rng.randint(int(a), int(b))
        return {k: sample_params(v, rng) for k, v in value.items()}
    if isinstance(value, list):
        return [sample_params(v, rng) for v in value]
    return value


# --------------------------------------------------------------------------- ctx and sessions

class InProcessContext(Context):
    """The runtime Context over an in-process Frontend: identical call/value/request semantics,
    including the JSON wire round trip, without a socket."""

    def __init__(self, frontend):
        self._frontend = frontend
        self._lock = threading.RLock()
        self._sequence = 0
        self._socket = None
        self.request('initialize', {'protocolVersion': '2025-06-18', 'capabilities': {},
                                    'clientInfo': {'name': 'dex-eval-runner', 'version': '1'}})

    def request(self, method, params):
        with self._lock:
            self._sequence += 1
            message = json.loads(json.dumps({'jsonrpc': '2.0', 'id': self._sequence, 'method': method,
                                             'params': params}, allow_nan=False))
            response = json.loads(json.dumps(self._frontend.handle(message), allow_nan=False))
            if response.get('id') != self._sequence:
                raise RuntimeError('MCP request failed: identity mismatch')
            if 'error' in response:
                raise RuntimeError('MCP tool failed: ' + json.dumps({'error': response['error']}, allow_nan=False))
            return response['result']


class InProcessSession:
    """mock/sim: this process owns a Frontend and its backend for the run."""

    def __init__(self, kind, backend, config):
        from services.controller.frontend import Frontend
        from services.controller.session import Session
        self.kind, self.backend, self.config = kind, backend, config
        self.frontend = Frontend(Session(config), backend)
        self.ctx = InProcessContext(self.frontend)
        self.status_fn = backend_status(backend) if any(
            t['name'] == 'wuji_status' for t in backend.tools()) else None

    def ready(self):
        return True, ''

    def stop_motion(self):
        _hold(self.ctx)

    def close(self):
        self.frontend.close()


class RealSession:
    """real: connect to the operator's running robot service. Never starts, enables or arms it."""
    kind = 'real'

    def __init__(self, session_root, socket=None, *, bimanual=True):
        self.root = Path(session_root).resolve()
        if not (self.root / 'operator-input.json').is_file():
            raise RuntimeError(f'{self.root} is not a RAX session directory (no operator-input.json)')
        if socket is None:
            from harness.claude_cli.launcher import socket_path
            socket = socket_path(self.root)
        if not Path(socket).exists():
            raise RuntimeError(f'no robot service socket at {socket}. Start the session\'s robot service first; '
                               'the runner never starts, enables or arms the robot.')
        self.ctx = Context(str(socket))
        self.status_fn = hand_service_status(self.root, bimanual)

    def ready(self):
        from services.xarm import arming
        state = arming.read(self.root)  # Read only: the runner never writes the arming file.
        return bool(state.get('armed')), state.get('reason', '')

    def stop_motion(self):
        _hold(self.ctx)

    def close(self):
        self.ctx.close()


def _hold(ctx):
    """Best-effort stop after an interrupt (xarm_hold also disarms; it never arms)."""
    try:
        tools = {t['name'] for t in ctx.request('tools/list', {}).get('tools', [])}
        if 'xarm_hold' in tools:
            ctx.value('xarm_hold')
        else:
            for side in SIDES:
                if 'wuji_hold' in tools:
                    try:
                        ctx.value('wuji_hold', hand=side)
                    except Exception:
                        pass
    except Exception:
        pass


def session_root_for(out_dir, run_id):
    """Frontend session root: inside out_dir when it is under the artifact root, else under it."""
    from services.storage_root import artifact_root
    out_dir = Path(out_dir).resolve()
    if out_dir.is_relative_to(artifact_root()):
        return out_dir / 'session'
    return artifact_root() / 'dex-eval' / run_id / 'session'


def _configuration(root, robot, options, plan):
    from services.controller.config import Configuration
    return Configuration(root=root, task='dex_eval', robot=robot, backend_options=options,
                         task_description=f'Evaluation run {plan.name}: '
                                          + ', '.join(c.id for c in plan.conditions))


def mock_session(root, plan, options=None):
    from .mock_rig import MockRigBackend
    options = {'sides': list(plan.sides), **plan.backend_options, **(options or {})}
    if plan.nominal_rate_hz:
        options.setdefault('nominal_rate_hz', plan.nominal_rate_hz)
    config = _configuration(root, 'mock', options, plan)
    return InProcessSession('mock', MockRigBackend(config), config)


def sim_robot_key(requested=None):
    from services.backends import BACKENDS
    if requested:
        if requested not in BACKENDS:
            raise RuntimeError(f'simulation backend {requested!r} is not registered in services.backends '
                               f'(registered: {sorted(BACKENDS)}); use --backend mock meanwhile')
        return requested
    simulated = sorted(k for k in BACKENDS if 'sim' in k)
    if not simulated:
        raise RuntimeError('no simulation backend is registered in services.backends yet '
                           f'(registered: {sorted(BACKENDS)}); use --backend mock or --sim-robot')
    return simulated[0]


def sim_session(root, plan, robot=None, options=None):
    from services.backends import make_backend
    key = sim_robot_key(robot)
    config = _configuration(root, key, {**plan.backend_options, **(options or {})}, plan)
    return InProcessSession('sim', make_backend(config), config)


# --------------------------------------------------------------------------- operator

class Operator:
    """Interactive prompts for real-robot runs (input_fn/print_fn injectable for tests)."""

    def __init__(self, input_fn=input, print_fn=print):
        self.input, self.print = input_fn, print_fn

    def confirm_condition(self, condition, params_text, trials):
        self.print('\n' + '=' * 72)
        self.print(f'REAL ROBOT condition {condition.id}: skill {condition.skill}, variant {condition.variant}, '
                   f'controller {condition.controller or "-"}')
        self.print(f'params: {params_text}')
        self.print(f'{trials} trials, interleaved with other conditions.')
        answer = self.input('Person at the emergency stop, workspace clear, condition reviewed? '
                            'Type RUN to allow it (anything else skips it): ')
        return answer.strip() == 'RUN'

    def wait_armed(self, reason):
        answer = self.input(f'Session is not armed ({reason}). The runner never arms; arm it yourself '
                            '(scripts/arm_robot.py <session> arm). Enter = check again, q = quit: ')
        return answer.strip().lower() != 'q'

    def reset(self, label):
        answer = self.input(f'Reset the scene for {label}. Enter = start, s = skip, q = quit: ').strip().lower()
        return {'': 'go', 's': 'skip', 'q': 'quit'}.get(answer, 'go' if answer in ('y', 'yes', 'go') else 'skip')

    def judge(self, result):
        status = (result.get('measured') or {}).get('status')
        answer = self.input(f'Skill reported ok={result.get("ok")} status={status}. '
                            'Task success? y / n / Enter = use the skill result: ').strip().lower()
        return True if answer in ('y', 'yes') else False if answer in ('n', 'no') else None


class Quit(Exception):
    pass


# --------------------------------------------------------------------------- runner

def git_commit():
    try:
        return subprocess.run(['git', '-C', str(PROJECT), 'rev-parse', 'HEAD'], capture_output=True,
                              text=True, timeout=5).stdout.strip() or None
    except Exception:
        return None


def slug(text):
    return re.sub(r'[^A-Za-z0-9_.-]+', '-', text).strip('-')[:60]


class Runner:
    def __init__(self, plan: Plan, session, out_dir, *, operator=None, interactive=False, trace='auto',
                 trace_tools=None, poll_hz=100.0, audio_device=None, loader=None, evaluate=False,
                 eval_config=None, log=print, argv=None):
        if session.kind == 'real' and operator is None:
            raise ValueError('real runs need an interactive operator')
        if trace not in ('auto', 'service', 'poll', 'none'):
            raise ValueError('trace must be auto, service, poll or none')
        self.plan, self.session, self.out_dir = plan, session, Path(out_dir)
        self.operator = operator
        self.interactive = interactive or session.kind == 'real'
        self.trace, self.trace_tools, self.poll_hz = trace, trace_tools, poll_hz
        self.audio_device = audio_device
        self.loader = loader if loader is not None else library_loader()
        self.evaluate, self.eval_config = evaluate, eval_config
        self.log = log
        self.argv = argv
        self._mains = {}
        self._service = None

    # ---------------------------------------------------------------- helpers
    def _main(self, condition):
        if condition.id not in self._mains:
            self._mains[condition.id] = resolve(condition.skill, condition.variant, entry=condition.entry,
                                                library_name=condition.library_name,
                                                search_paths=[Path(p) for p in self.plan.skill_paths],
                                                loader=self.loader)
        return self._mains[condition.id]

    def _recorders(self):
        recorders = []
        if self.trace in ('auto', 'service'):
            if self._service is None:
                self._service = ServiceRecorder.detect(self.session.ctx, self.trace_tools) or False
            if self._service:
                recorders.append(ServiceRecorder(self.session.ctx, self._service.start_tool,
                                                 self._service.stop_tool))
            elif self.trace == 'service':
                raise RuntimeError('the robot service lists no trace start/stop tools')
        if self.trace == 'poll' or (self.trace == 'auto' and not recorders):
            if self.session.status_fn is None:
                if self.trace == 'poll':
                    raise RuntimeError('no telemetry source for the polling recorder')
            else:
                recorders.append(PollingRecorder(self.session.status_fn, self.plan.sides, self.poll_hz))
        if self.audio_device:
            recorders.append(AudioRecorder(self.audio_device))
        return recorders

    def _write_run(self, record):
        path = self.out_dir / 'run.json'
        path.write_text(json.dumps(record, indent=2, default=str) + '\n')

    # ---------------------------------------------------------------- run
    def run(self, *, dry_run=False):
        plan = self.plan
        if plan.seed is None:
            plan.seed = int.from_bytes(os.urandom(4), 'big')
        items = schedule(len(plan.conditions), plan.trials, plan.seed)
        record = {'plan': plan.to_dict(), 'seed': plan.seed, 'backend': self.session.kind if self.session else None,
                  'schedule': [{**i, 'condition_id': plan.conditions[i['condition']].id} for i in items],
                  'git_commit': git_commit(), 'argv': self.argv, 'host': os.uname().nodename,
                  'started_at': datetime.now(timezone.utc).isoformat(), 'finished_at': None,
                  'trials_completed': 0, 'skipped': [], 'declined_conditions': [], 'stopped_by': None}
        if dry_run:
            return record
        self.out_dir.mkdir(parents=True, exist_ok=True)
        (self.out_dir / 'trials').mkdir(exist_ok=True)
        results_path = self.out_dir / 'results.jsonl'
        self._write_run(record)
        confirmed = {}
        try:
            for item in items:
                condition = plan.conditions[item['condition']]
                label = f'trial {item["index"] + 1}/{len(items)} ({condition.id}, block {item["block"] + 1})'
                if self.session.kind == 'real':
                    if condition.id not in confirmed:
                        confirmed[condition.id] = self.operator.confirm_condition(
                            condition, json.dumps(condition.params), plan.trials)
                        if not confirmed[condition.id]:
                            record['declined_conditions'].append(condition.id)
                    if not confirmed[condition.id]:
                        record['skipped'].append({'index': item['index'], 'condition_id': condition.id,
                                                  'reason': 'condition declined'})
                        continue
                if self.interactive and self.operator:
                    action = self.operator.reset(label)
                    if action == 'quit':
                        raise Quit('operator quit')
                    if action == 'skip':
                        record['skipped'].append({'index': item['index'], 'condition_id': condition.id,
                                                  'reason': 'operator skipped'})
                        continue
                while True:
                    armed, reason = self.session.ready()
                    if armed:
                        break
                    if not self.operator or not self.operator.wait_armed(reason):
                        raise Quit(f'session not armed: {reason}')
                self.log(f'[dex-eval] {label}')
                line = self.run_trial(item, condition)
                with results_path.open('a') as stream:
                    stream.write(json.dumps(line, allow_nan=False, default=str) + '\n')
                record['trials_completed'] += 1
                self._write_run(record)
                if line.get('interrupted'):
                    raise KeyboardInterrupt
        except Quit as exc:
            record['stopped_by'] = str(exc)
        except KeyboardInterrupt:
            record['stopped_by'] = 'interrupted'
            raise
        finally:
            record['finished_at'] = datetime.now(timezone.utc).isoformat()
            self._write_run(record)
        return record

    def run_trial(self, item, condition):
        plan = self.plan
        trial_id = f'{item["index"]:04d}'
        trial_dir = self.out_dir / 'trials' / f'{trial_id}_{slug(condition.skill + "_" + condition.variant)}'
        trial_dir.mkdir(parents=True, exist_ok=False)
        params = sample_params(condition.params, random.Random(plan.seed * 1_000_003 + item['index']))
        info = {'trial_id': trial_id, 'sides': plan.sides, 'condition_id': condition.id}
        started_at = datetime.now(timezone.utc).isoformat()
        start = time.monotonic()
        started, recorder_errors, stops = [], [], []
        for recorder in self._recorders():
            try:
                recorder.start(trial_dir, info)
                started.append(recorder)
            except Exception as exc:
                recorder_errors.append(f'{type(recorder).__name__} start: {type(exc).__name__}: {exc}')
        result, error, interrupted, source = None, None, False, None
        try:
            main, source = self._main(condition)
            result = main(self.session.ctx, **params)
        except KeyboardInterrupt:
            interrupted = True
            self.session.stop_motion()
            result = {'ok': False, 'measured': {'status': 'aborted'}, 'why': 'interrupted by the operator'}
        except Exception as exc:
            error = traceback.format_exc(limit=8)
            result = {'ok': False, 'measured': {'status': 'aborted'}, 'why': f'{type(exc).__name__}: {exc}'}
        finally:
            for recorder in reversed(started):
                try:
                    stops.append(recorder.stop())
                except Exception as exc:
                    recorder_errors.append(f'{type(recorder).__name__} stop: {type(exc).__name__}: {exc}')
        duration = time.monotonic() - start
        normalized, problems = normalize_result(result)
        success = None
        if self.session.kind == 'real' and not interrupted:
            success = self.operator.judge(normalized)
        recorder_errors += [s['error'] for s in stops if s.get('error')]
        meta = self._meta(trial_dir, condition, params, started_at, stops, success, item, normalized, source)
        try:
            write_meta(trial_dir / 'meta.json', meta)
        except ValueError:
            (trial_dir / 'meta.json').write_text(json.dumps(meta, indent=2, default=str) + '\n')
        trace_errors, _ = validate_trial_dir(trial_dir)
        verdict = outcome(normalized, meta)
        line = {'run': plan.name, 'trial_id': trial_id, 'index': item['index'], 'block': item['block'],
                'condition_id': condition.id, 'skill': condition.skill, 'variant': condition.variant,
                'controller': meta.get('controller'), 'params': params, 'backend': self.session.kind,
                'seed': plan.seed, 'trace_dir': str(trial_dir.relative_to(self.out_dir)),
                'started_at': started_at, 'duration_s': duration, 'result': normalized,
                'status': verdict['status'], 'ok': verdict['ok'], 'success': verdict['success'],
                'success_source': verdict['success_source'], 'result_problems': problems, 'error': error,
                'trace_source': meta.get('trace_source'), 'recorder_errors': recorder_errors,
                'trace_errors': trace_errors, 'skill_source': source, 'interrupted': interrupted}
        if self.evaluate and not trace_errors:
            from .evaluate import evaluate_trial
            try:
                line['summary'] = evaluate_trial(trial_dir, self.eval_config, normalized, cache=True)['summary']
            except Exception as exc:
                line['evaluation_error'] = f'{type(exc).__name__}: {exc}'
        return line

    def _meta(self, trial_dir, condition, params, started_at, stops, success, item, result, source):
        recorded = {}
        for stop in stops:
            for key, value in (stop.get('meta') or {}).items():
                if key not in recorded or recorded[key] in (None, [], {}):
                    recorded[key] = value
        meta = default_meta()
        meta.update(recorded)
        sides = [s for s in SIDES if (trial_dir / f'hand_{s}.npz').is_file()]
        meta.update(sides=sides or [s for s in recorded.get('sides') or [] if s in SIDES],
                    skill=condition.skill, variant=condition.variant, params=params,
                    backend=self.session.kind, started_at=started_at, success=success,
                    notes=condition.notes or self.plan.notes)
        meta['controller'] = condition.controller or recorded.get('controller') or 'unknown'
        meta['nominal_rate_hz'] = float(recorded.get('nominal_rate_hz') or self.plan.nominal_rate_hz or 200.0)
        meta.update(trial_id=f'{item["index"]:04d}', block=item['block'], condition_id=condition.id, run=self.plan.name,
                    seed=self.plan.seed, result=result, skill_source=source)
        return meta


def read_results(path):
    """Result lines of a results.jsonl (blank lines skipped)."""
    lines = []
    for raw in Path(path).read_text().splitlines():
        if raw.strip():
            lines.append(json.loads(raw))
    return lines
