"""Report generation on synthetic fixtures: skill comparisons, battery traces and ratings."""
import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from services.dex_eval import rating
from services.dex_eval.evaluate import EvalConfig
from services.dex_eval.report import ReportOptions, build, collect, group
from services.dex_eval.synthetic import (breakaway, demo_waypoints, hand_arrays, second_order, waypoint_reference,
                                         write_trial)

PROJECT = Path(__file__).resolve().parents[2]
FS = 200.0
CONFIG = EvalConfig(fingertip=False)


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, PROJECT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def result(success):
    return {'ok': success, 'measured': {'status': 'success' if success else 'dropped'}, 'why': ''}


def trial(root, name, *, sticky, seed, meta):
    rng = np.random.default_rng(seed)
    move = 6.0 if sticky else 1.5
    reference = waypoint_reference(demo_waypoints(rng), FS, move_s=move, hold_s=1.0)
    measured = (breakaway(reference, FS, break_rad=0.012)[0] if sticky else
                second_order(reference, FS) + rng.normal(0, 1e-4, reference.shape))
    return write_trial(root / name, {'right': hand_arrays(reference, measured, FS)}, meta=meta)


@pytest.fixture
def skill_fixture(tmp_path):
    root = tmp_path / 'run' / 'trials'
    successes = {'smooth': [True, True, True], 'sticky': [True, False, False]}
    for variant, outcomes in successes.items():
        for k, success in enumerate(outcomes):
            trial(root, f'{variant}{k}', sticky=variant == 'sticky', seed=k,
                  meta={'skill': 'grasp', 'variant': variant, 'controller': 'C0', 'backend': 'mock', 'seed': 4,
                        'condition_id': f'grasp/{variant}', 'result': result(success)})
    # A video for the rating session.
    for k in range(3):
        for variant in ('smooth', 'sticky'):
            (root / f'{variant}{k}' / 'video_front.mp4').write_bytes(f'{variant}{k}'.encode() * 100)
    return tmp_path / 'run'


@pytest.fixture
def battery_fixture(tmp_path):
    root = tmp_path / 'battery'
    for controller, sticky in (('C0-baseline', True), ('C2-friction-ff', False)):
        for item in ('A1', 'A2'):
            for k in range(2):
                trial(root, f'{controller}_{item}_{k}', sticky=sticky, seed=k,
                      meta={'skill': 'wuji_motion_battery', 'variant': item, 'controller': controller,
                            'params': {'item': item}, 'backend': 'sim'})
    return root


def test_skill_report_ranks_flags_and_plots(skill_fixture, tmp_path):
    records = collect([skill_fixture], CONFIG)
    assert len(records) == 6 and all(r.evaluation for r in records)
    path = build(records, tmp_path / 'report', ReportOptions(baseline='grasp/sticky'), inputs=[skill_fixture])
    text = path.read_text()
    assert '## Primitive: grasp' in text
    ranking = text.split('### Ranking')[1].split('###')[0]
    rows = [line for line in ranking.splitlines() if line.startswith('| 1 ') or line.startswith('| 2 ')]
    assert 'grasp/smooth' in rows[0] and '3/3 (100%)' in rows[0]
    assert 'grasp/sticky' in rows[1] and '1/3 (33%)' in rows[1]
    flags = text.split('### Threshold flags')[1].split('###')[0]
    assert '**grasp/sticky**' in flags and 'stick_slip_count' in flags and 'sparc_ratio' in flags
    assert '**grasp/smooth**' not in flags
    assert 'Mann-Whitney' in text
    plots = sorted(p.name for p in (tmp_path / 'report' / 'plots').iterdir())
    assert 'grasp_sparc.png' in plots
    assert 'grasp_grasp-smooth_trajectory.png' in plots and 'grasp_grasp-sticky_trajectory.png' in plots
    assert all((tmp_path / 'report' / 'plots' / p).read_bytes()[:4] == b'\x89PNG' for p in plots)
    summary = json.loads((tmp_path / 'report' / 'summary.json').read_text())
    sticky = summary['groups']['grasp']['grasp/sticky']
    assert sticky['success'] == {'k': 1, 'n': 3, 'rate': pytest.approx(1 / 3), 'low': pytest.approx(0.0615, abs=1e-3),
                                 'high': pytest.approx(0.7923, abs=1e-3)}
    assert sticky['statuses'] == {'success': 1, 'dropped': 2}
    with open(tmp_path / 'report' / 'trials.csv') as stream:
        assert len(list(csv.DictReader(stream))) == 6


def test_battery_traces_group_by_controller(battery_fixture, tmp_path):
    records = collect([battery_fixture], CONFIG)
    groups, mode = group(records)
    assert mode == 'controller'
    assert set(groups) == {'all items', 'A1', 'A2'}
    assert set(groups['all items']) == {'C0-baseline', 'C2-friction-ff'}
    assert len(groups['all items']['C0-baseline']) == 4
    path = build(records, tmp_path / 'report', inputs=[battery_fixture])
    text = path.read_text()
    assert '## Battery: all items (per controller)' in text and '## Battery item: A1' in text
    ranking = text.split('### Ranking')[1].split('###')[0]
    first = next(line for line in ranking.splitlines() if line.startswith('| 1 '))
    assert 'C2-friction-ff' in first and 'sparc_ratio median' in ranking
    assert (tmp_path / 'report' / 'plots' / 'all-items_sparc_ratio.png').is_file()


def test_threshold_calibration_from_a_reference_condition(skill_fixture, tmp_path):
    records = collect([skill_fixture], CONFIG)
    path = build(records, tmp_path / 'report', ReportOptions(calibrate_from='grasp/sticky', plots=False))
    text = path.read_text()
    assert 'calibrated from condition grasp/sticky' in text
    assert '### Plots' not in text


def test_report_cli_and_ratings_section(skill_fixture, tmp_path):
    session = tmp_path / 'ratings'
    summary = rating.build_session([skill_fixture], session, seed=1, per_pair=3)
    assert summary == {'pairs': 3, 'clips': 6, 'groups': ['grasp']}
    for pair in rating.load_pairs(session):
        key = json.loads((session / 'key.json').read_text())['clips']
        smooth_is_a = key[pair['a']]['condition'] == 'grasp/smooth'
        rating.record_rating(session, pair['pair_id'], 'A' if smooth_is_a else 'B', 'rater1')
    cli = load_script('dex_eval_report')
    assert cli.main([str(skill_fixture), '--out', str(tmp_path / 'cli-report'), '--no-fingertip',
                     '--ratings', str(session)]) == 0
    text = (tmp_path / 'cli-report' / 'report.md').read_text()
    naturalness = text.split('## Naturalness')[1]
    assert naturalness.index('grasp/smooth') < naturalness.index('grasp/sticky')
    assert '| 1 | grasp/smooth |' in naturalness


def test_unreadable_traces_still_count_outcomes(skill_fixture, tmp_path):
    broken = skill_fixture / 'trials' / 'smooth0' / 'hand_right.npz'
    broken.write_bytes(b'not a zip')
    records = collect([skill_fixture], CONFIG)
    bad = [r for r in records if r.error]
    assert len(bad) == 1 and 'unreadable' in bad[0].error
    path = build(records, tmp_path / 'report', ReportOptions(plots=False))
    assert 'Trials without usable traces' in path.read_text()
    assert '3/3 (100%)' in path.read_text()
