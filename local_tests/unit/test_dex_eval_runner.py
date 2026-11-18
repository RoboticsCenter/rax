"""Trial runner tests with fake skills on the hardware-free mock rig. Nothing here touches hardware."""
import json
from pathlib import Path
import random

import pytest

from services.dex_eval import runner as runner_module
from services.dex_eval import skills
from services.dex_eval.runner import (InProcessContext, Plan, Runner, load_plan, mock_session,
                                      read_results, sample_params, schedule, session_root_for)
from services.dex_eval.trace import validate_trial_dir

PROJECT = Path(__file__).resolve().parents[2]


def load_script(name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, PROJECT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

SKILL = '''
import time
CALLS = []

def main(ctx, amplitude=0.4, duration_s=0.6, hand="right", fail=False, explode=False, bad=False, sleep_s=0.0):
    CALLS.append(dict(amplitude=amplitude, duration_s=duration_s))
    if explode:
        raise ValueError("skill exploded")
    if bad:
        return "not a dict"
    if sleep_s:
        time.sleep(sleep_s)
    goal = [0.0] * 20
    for j in (4, 5, 6, 8, 9, 10):
        goal[j] = amplitude
    reached = ctx.value("wuji_move_joints", hand=hand, positions=goal, duration_s=duration_s)
    ctx.value("wuji_move_joints", hand=hand, positions=[0.0] * 20, duration_s=duration_s)
    status = "failed_precondition" if fail else "success"
    return {"ok": not fail and reached["reached"], "measured": {"status": status}, "why": ""}
'''


@pytest.fixture
def skill_dir(tmp_path):
    root = tmp_path / 'skills'
    (root / 'grasp').mkdir(parents=True)
    for variant in ('fast', 'slow'):
        (root / 'grasp' / f'{variant}.py').write_text(SKILL)
    return root


def plan_for(skill_dir, trials=3, **extra):
    conditions = extra.pop('conditions', [
        {'skill': 'grasp', 'variant': 'fast', 'controller': 'C1',
         'params': {'duration_s': 0.5, 'amplitude': {'uniform': [0.3, 0.5]}}},
        {'skill': 'grasp', 'variant': 'slow', 'params': {'duration_s': 4.0}}])
    return Plan(conditions=conditions, trials=trials, seed=11, sides=['right'], skill_paths=[str(skill_dir)],
                **extra)


def run_mock(plan, out, **kwargs):
    session = mock_session(session_root_for(out, plan.name), plan, kwargs.pop('options', None))
    try:
        record = Runner(plan, session, out, loader=False, **kwargs).run()
    finally:
        session.close()
    return record, read_results(out / 'results.jsonl')


def test_schedule_is_balanced_interleaved_and_seeded():
    items = schedule(3, 10, seed=5)
    assert len(items) == 30
    for block in range(10):
        assert sorted(i['condition'] for i in items if i['block'] == block) == [0, 1, 2]
    assert items == schedule(3, 10, seed=5)
    orders = {tuple(i['condition'] for i in items if i['block'] == b) for b in range(10)}
    assert len(orders) > 1
    assert schedule(3, 10, seed=6) != items


def test_sample_params_distributions_are_reproducible():
    spec = {'a': {'uniform': [0, 1]}, 'b': {'choice': ['x', 'y']}, 'c': [{'randint': [1, 3]}], 'd': 4,
            'e': {'nested': {'normal': [0, 1]}}, 'f': {'loguniform': [0.1, 10]}}
    one, two = sample_params(spec, random.Random(1)), sample_params(spec, random.Random(1))
    assert one == two
    assert 0 <= one['a'] <= 1 and one['b'] in ('x', 'y') and one['c'][0] in (1, 2, 3) and one['d'] == 4
    assert isinstance(one['e']['nested'], float) and 0.1 <= one['f'] <= 10


def test_plan_validation(tmp_path):
    with pytest.raises(ValueError, match='duplicate'):
        Plan(conditions=[{'skill': 'a', 'variant': 'b'}, {'skill': 'a', 'variant': 'b'}])
    with pytest.raises(ValueError, match='no conditions'):
        Plan(conditions=[])
    path = tmp_path / 'plan.json'
    path.write_text(json.dumps({'conditions': [{'skill': 'a', 'variant': 'b'}], 'bogus': 1}))
    with pytest.raises(ValueError, match='unknown plan keys'):
        load_plan(path)


def test_mock_run_records_interleaved_trials_traces_and_results(tmp_path, skill_dir):
    out = tmp_path / 'run'
    record, lines = run_mock(plan_for(skill_dir), out, evaluate=True)
    assert record['trials_completed'] == 6 and record['seed'] == 11
    assert [l['index'] for l in lines] == list(range(6))
    for block in range(3):
        assert sorted(l['condition_id'] for l in lines if l['block'] == block) == ['grasp/fast', 'grasp/slow']
    run = json.loads((out / 'run.json').read_text())
    assert run['plan']['seed'] == 11 and len(run['schedule']) == 6 and run['finished_at']
    fast = [l for l in lines if l['variant'] == 'fast']
    assert len({l['params']['amplitude'] for l in fast}) == 3
    assert all(0.3 <= l['params']['amplitude'] <= 0.5 for l in fast)
    for line in lines:
        assert line['status'] == 'success' and line['success'] is True and line['error'] is None
        assert line['trace_source'].startswith('service:') and line['trace_errors'] == []
        trial = out / line['trace_dir']
        assert validate_trial_dir(trial)[0] == []
        meta = json.loads((trial / 'meta.json').read_text())
        assert meta['backend'] == 'mock' and meta['skill'] == 'grasp' and meta['variant'] == line['variant']
        assert meta['params'] == line['params'] and meta['success'] is None and meta['sides'] == ['right']
        assert meta['controller'] == ('C1' if line['variant'] == 'fast' else 'mock-second-order-breakaway')
        assert meta['seed'] == 11 and meta['result']['measured']['status'] == 'success'
    # The mock plant's breakaway friction makes the slow variant stick-slip; the fast one does not.
    slow = [l['summary'] for l in lines if l['variant'] == 'slow']
    assert all(s['stick_slip_count'] > 10 for s in slow)
    assert all(l['summary']['stick_slip_count'] == 0 for l in fast)
    # Every robot call went through the controller frontend and its audit log.
    audit = (out / 'session' / 'mcp.jsonl').read_text()
    assert '"tool": "wuji_move_joints"' in audit and '"tool": "trace_start"' in audit
    # Same seed, same schedule and parameters.
    _, again = run_mock(plan_for(skill_dir), tmp_path / 'again')
    assert [(l['condition_id'], l['params']) for l in again] == [(l['condition_id'], l['params']) for l in lines]


def test_skill_failures_are_recorded_as_typed_outcomes(tmp_path, skill_dir):
    plan = plan_for(skill_dir, trials=1, conditions=[
        {'skill': 'grasp', 'variant': 'fast', 'id': 'explode', 'params': {'explode': True}},
        {'skill': 'grasp', 'variant': 'fast', 'id': 'bad', 'params': {'bad': True}},
        {'skill': 'grasp', 'variant': 'fast', 'id': 'fail', 'params': {'fail': True}},
        {'skill': 'missing', 'variant': 'nope', 'id': 'missing'}])
    _, lines = run_mock(plan, tmp_path / 'run')
    by_id = {l['condition_id']: l for l in lines}
    assert by_id['explode']['status'] == 'aborted' and 'skill exploded' in by_id['explode']['result']['why']
    assert 'ValueError' in by_id['explode']['error']
    assert by_id['bad']['status'] == 'aborted' and by_id['bad']['result_problems'] == ['result is not a dict']
    assert by_id['fail']['status'] == 'failed_precondition' and by_id['fail']['success'] is False
    assert by_id['missing']['status'] == 'aborted' and 'SkillLoadError' in by_id['missing']['error']
    # Traces are still recorded for failed trials (the recorder brackets the call).
    assert by_id['explode']['trace_errors'] == []


def test_polling_fallback_when_the_service_has_no_trace_tools(tmp_path, skill_dir):
    plan = plan_for(skill_dir, trials=1, conditions=[
        {'skill': 'grasp', 'variant': 'fast', 'params': {'sleep_s': 0.15}}])
    _, lines = run_mock(plan, tmp_path / 'run', options={'trace_tools': False}, poll_hz=200.0)
    line = lines[0]
    assert line['trace_source'] == 'poll' and line['trace_errors'] == []
    meta = json.loads((tmp_path / 'run' / line['trace_dir'] / 'meta.json').read_text())
    assert meta['trace_source'] == 'poll' and meta['nominal_rate_hz'] == 200.0


def test_in_process_context_has_runtime_semantics(tmp_path, skill_dir):
    plan = plan_for(skill_dir)
    session = mock_session(session_root_for(tmp_path / 'run', 'x'), plan)
    try:
        ctx = session.ctx
        assert isinstance(ctx, InProcessContext)
        names = {t['name'] for t in ctx.request('tools/list', {})['tools']}
        assert {'status', 'wuji_move_joints', 'trace_start'} <= names
        assert len(ctx.value('wuji_status', hand='right')['positions']) == 20
        with pytest.raises(RuntimeError, match='MCP tool failed'):
            ctx.value('wuji_move_joints', hand='right', positions=[5.0] * 20)
        assert ctx.value('status')['robot'] == 'mock'
    finally:
        session.close()


def test_skill_resolution_order_and_library_adapter(tmp_path, skill_dir, monkeypatch):
    main, source = skills.resolve('grasp', 'fast', search_paths=[skill_dir], loader=False)
    assert callable(main) and source.endswith('grasp/fast.py')
    module_file = tmp_path / 'entry_skill.py'
    module_file.write_text('def run(ctx, **p):\n    return {"ok": True, "measured": {"status": "success"}, "why": ""}\n')
    main, source = skills.resolve('x', 'y', entry=f'{module_file}:run')
    assert main(None)['ok'] and source.startswith('entry')
    asked = []

    def load_skill(name):
        asked.append(name)
        if name != 'grasp_fast':
            raise KeyError(name)
        return lambda ctx, **p: {'ok': True, 'measured': {'status': 'success'}, 'why': 'library'}
    main, source = skills.resolve('grasp', 'fast', loader=load_skill)
    assert asked == ['grasp/fast', 'grasp_fast'] and main(None)['why'] == 'library'
    import services.skill_library as library
    monkeypatch.setattr(library, 'load_skill', load_skill, raising=False)
    monkeypatch.delattr(library, 'load_skill_main', raising=False)  # probed first when present (the real loader)
    assert skills.library_loader() is load_skill
    with pytest.raises(skills.SkillLoadError, match='cannot resolve'):
        skills.resolve('nothing', 'here', search_paths=[skill_dir], loader=False)


def test_interrupt_stops_motion_and_reraises(tmp_path, skill_dir):
    plan = plan_for(skill_dir, trials=2)
    out = tmp_path / 'run'
    session = mock_session(session_root_for(out, 'x'), plan)
    runner = Runner(plan, session, out, loader=False)

    def interrupted(ctx, **params):
        raise KeyboardInterrupt
    runner._mains = {'grasp/fast': (interrupted, 'test'), 'grasp/slow': (interrupted, 'test')}
    try:
        with pytest.raises(KeyboardInterrupt):
            runner.run()
    finally:
        session.close()
    lines = read_results(out / 'results.jsonl')
    assert len(lines) == 1 and lines[0]['interrupted'] and lines[0]['status'] == 'aborted'
    assert 'xarm_hold' in session.backend.calls
    assert json.loads((out / 'run.json').read_text())['stopped_by'] == 'interrupted'


# --------------------------------------------------------------------------- real-robot safety

class ScriptedOperator(runner_module.Operator):
    def __init__(self, answers):
        self.answers, self.prompts = list(answers), []
        super().__init__(input_fn=self._input, print_fn=lambda *a: None)

    def _input(self, prompt):
        self.prompts.append(prompt)
        return self.answers.pop(0)


class FakeRealSession:
    """A 'real' session over the mock rig, with arming read from an operator file like the real one."""
    kind = 'real'

    def __init__(self, root, plan):
        self.inner = mock_session(root / 'session', plan)
        self.ctx, self.status_fn, self.root = self.inner.ctx, self.inner.status_fn, root

    def ready(self):
        from services.xarm import arming
        state = arming.read(self.root)
        return state['armed'], state.get('reason', '')

    def stop_motion(self):
        self.inner.stop_motion()

    def close(self):
        self.inner.close()


def test_real_runs_refuse_when_not_armed_and_never_arm(tmp_path, skill_dir):
    from services.xarm import arming
    plan = plan_for(skill_dir, trials=1)
    root = tmp_path / 'operator-session'
    root.mkdir()
    session = FakeRealSession(root, plan)
    operator = ScriptedOperator(['RUN', '', 'q'])
    try:
        record = Runner(plan, session, tmp_path / 'run', operator=operator, loader=False).run()
    finally:
        session.close()
    assert record['trials_completed'] == 0 and 'not armed' in record['stopped_by']
    assert not arming.arm_file(root).exists()
    assert 'never arms' in operator.prompts[-1]


def test_real_runs_confirm_each_condition_prompt_resets_and_take_verdicts(tmp_path, skill_dir):
    from services.xarm import arming
    plan = plan_for(skill_dir, trials=2)
    root = tmp_path / 'operator-session'
    root.mkdir()
    arming.write(root, True, minutes=5, by='test-operator')   # The operator arms; the runner only reads.
    before = arming.arm_file(root).read_bytes()
    order = [plan.conditions[i['condition']].id for i in schedule(2, 2, plan.seed)]
    answers = []
    confirmed = set()
    for condition in order:
        if condition not in confirmed:
            answers.append('RUN' if condition == 'grasp/fast' else 'no')
            confirmed.add(condition)
        if condition == 'grasp/fast':
            answers += ['', 'y']   # Reset prompt, then success verdict.
    session = FakeRealSession(root, plan)
    operator = ScriptedOperator(answers)
    try:
        record = Runner(plan, session, tmp_path / 'run', operator=operator, loader=False).run()
    finally:
        session.close()
    lines = read_results(tmp_path / 'run' / 'results.jsonl')
    assert [l['condition_id'] for l in lines] == ['grasp/fast', 'grasp/fast']
    assert record['declined_conditions'] == ['grasp/slow'] and len(record['skipped']) == 2
    assert all(l['backend'] == 'real' and l['success'] is True and l['success_source'] == 'meta' for l in lines)
    assert sum('Type RUN' in p for p in operator.prompts) == 2
    assert sum('Reset the scene' in p for p in operator.prompts) == 2
    assert arming.arm_file(root).read_bytes() == before
    assert not operator.answers


def test_runner_sources_never_write_arming():
    for path in (PROJECT / 'services/dex_eval/runner.py', PROJECT / 'scripts/dex_eval_run.py',
                 PROJECT / 'services/dex_eval/recorder.py'):
        text = path.read_text()
        assert 'arming.write' not in text and 'arm_robot' not in text.replace('scripts/arm_robot.py', '')


def test_cli_guards_real_mode(tmp_path, skill_dir, capsys):
    cli = load_script('dex_eval_run')
    plan = tmp_path / 'plan.json'
    plan.write_text(json.dumps(plan_for(skill_dir).to_dict()))
    for argv, message in (([str(plan), '--backend', 'real'], 'requires the explicit --real'),
                          ([str(plan), '--real'], '--real requires --session'),
                          ([str(plan), '--real', '--backend', 'mock'], 'cannot be combined'),
                          ([str(plan), '--real', '--session', str(tmp_path)], 'interactive terminal')):
        with pytest.raises(SystemExit):
            cli.main(argv)
        assert message in capsys.readouterr().err
    with pytest.raises(RuntimeError, match='not a RAX session'):
        runner_module.RealSession(tmp_path)
    (tmp_path / 'operator-input.json').write_text('{}')
    with pytest.raises(RuntimeError, match='never starts, enables or arms'):
        runner_module.RealSession(tmp_path, tmp_path / 'missing.sock')


def test_cli_dry_run_and_mock_run(tmp_path, skill_dir, capsys):
    cli = load_script('dex_eval_run')
    plan = tmp_path / 'plan.json'
    plan.write_text(json.dumps(plan_for(skill_dir, trials=2).to_dict()))
    assert cli.main([str(plan), '--dry-run', '--seed', '3']) == 0
    printed = json.loads(capsys.readouterr().out.split('\n', 1)[1])
    assert printed['seed'] == 3 and len(printed['schedule']) == 4
    out = tmp_path / 'artifacts-run'
    assert cli.main([str(plan), '--out', str(out), '--evaluate']) == 0
    assert len(read_results(out / 'results.jsonl')) == 4
    with pytest.raises(SystemExit):
        cli.main([str(plan), '--out', str(out)])   # Refuses to overwrite results.
