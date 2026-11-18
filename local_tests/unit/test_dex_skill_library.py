"""The rig's dexterous skill library: configuration, session checkout, the variant format, merges, sync and fork,
the skill_library_merge tool, workspace wiring, and the host-side load/run API. Hardware-free."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

from conftest import write_minimal_library
from services import skill_library as lib
from services.controller.config import Configuration

ROOT = Path(__file__).resolve().parents[2]
# A checkout of skill-library/xarm-wuji-bimanual/dex beside the RAX checkout (or its main checkout, from a worktree).
REAL_LIBRARY = Path(os.environ.get('RAX_TEST_DEX_LIBRARY') or next(
    (p / 'mooncake-skill-dex' for p in (ROOT.parent, *ROOT.parents[2:4]) if (p / 'mooncake-skill-dex').is_dir()),
    ROOT.parent / 'mooncake-skill-dex'))
OPTION = {'name': 'xarmwujibimanual', 'merge': True,
          'source': {'repository': 'unused-url', 'ref': 'skill-library/xarm-wuji-bimanual/dex'}}
SECTIONS = '\n\n'.join(f'## {s}\ntext' for s in lib.SECTIONS[:4]) + \
    '\n\n## Checks\ntext\n\n### Perception checkpoints\nNone\n\n## Failure modes\ntext\n\n## Evidence\ntext\n'
ENTRY = ('def check_preconditions(ctx, **params):\n    return {"ok": not params.get("refuse"), "why": "refused"}\n\n'
         'def main(ctx, progress=None, **params):\n'
         '    if progress:\n        progress("act", 0.5)\n'
         '    if params.get("explode"):\n        raise RuntimeError("boom")\n'
         '    ctx.value("wuji_status", hand=params.get("side", "right"))\n'
         '    return {"ok": True, "measured": {"status": "success", "echo": params}, "why": "done"}\n\n'
         'def success(ctx, measured):\n    return not measured["echo"].get("unconvinced")\n')


def git(*args, cwd):
    return subprocess.run(['git', *args], cwd=cwd, check=True, capture_output=True, text=True).stdout


def frontmatter(slug, *, status='draft', representation=None, extra=''):
    primitive, rest = slug.split('--')
    representation = representation or rest.split('-')[0]
    return (f'---\nname: {slug}\ndescription: a variant\nstatus: {status}\nrobot: xarmwujibimanual\n'
            f'entry: code/run.py\nversion: 1\nuses: []\nprimitive: {primitive}\nrepresentation: {representation}\n'
            'hands: either\nrequires:\n  perception: none\n  sim_model: none\n  checkpoint: none\n  gpu: none\n'
            '  hand_features: []\ncontrol:\n  mode: blocking\n  rate_hz: 1\n  impedance: profile\n'
            'preconditions:\n  - "hand-ready: the hand is fresh and holding"\n'
            'postconditions:\n  - "done: the thing is done"\nprogress: [act]\n' + extra + '---\n')


def write_variant(library, slug='power-grasp--code-test', *, status='draft', runs=(), entry=ENTRY, body=SECTIONS,
                  meta=None, params=True):
    folder = Path(library) / 'skills' / slug
    package = folder / 'code' / slug.replace('-', '_')
    package.mkdir(parents=True, exist_ok=True)
    (folder / 'SKILL.md').write_text((meta or frontmatter(slug, status=status)) + '\n# Variant\n\n' + body)
    (folder / 'code' / 'run.py').write_text(entry)
    (package / '__init__.py').write_text('')
    if params:
        (package / 'params.json').write_text(json.dumps({'side': {
            'controls': 'hand', 'value': 'right', 'unit': 'side', 'tested': None, 'tune': 'n/a'}}))
    (folder / 'evidence').mkdir(exist_ok=True)
    (folder / 'evidence' / 'runs.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in runs))
    index = Path(library) / 'INDEX.md'
    if f'(skills/{slug}/SKILL.md)' not in index.read_text():
        index.write_text(index.read_text() + f'| [{slug}](skills/{slug}/SKILL.md) | p | code | either | {status} '
                         '| x | y |\n')
    return folder


def run_line(*, backend='real', success=True, unattended=True, version=1):
    return {'date': '2026-11-20', 'session': 's', 'commit': 'abc', 'version': version, 'backend': backend,
            'trial': 'runs/trial-1', 'params': {'side': 'right'}, 'success': success,
            'status': 'success' if success else 'failed', 'unattended': unattended, 'conditions': 'cube',
            'smoothness': {'sparc': -1.5}}


@pytest.fixture
def source(tmp_path, monkeypatch):
    """A fake mooncake repository with the library branch; the host libraries live in the artifact root."""
    repo = tmp_path / 'mooncake'
    repo.mkdir()
    git('init', '-q', '-b', 'main', cwd=repo)
    git('checkout', '-q', '-b', 'skill-library/xarm-wuji-bimanual/dex', cwd=repo)
    write_minimal_library(repo)
    git('add', '-A', cwd=repo)
    git('-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'init', cwd=repo)
    monkeypatch.setenv('RAX_MOONCAKE_REPO', str(repo))
    return repo


def session(tmp_path, name, option=OPTION):
    config = Configuration(root=tmp_path / 'sessions' / name, task='t', robot='mock',
                           backend_options={'skill_library': option})
    workspace = config.root / 'agent-workspace'
    workspace.mkdir(parents=True)
    return config, workspace, lib.checkout_library(config, workspace)


def commit(clone, message='variant'):
    git('add', '-A', cwd=clone)
    git('commit', '-q', '-m', message, cwd=clone)


# ---- configuration ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize('bad', [{'name': 'Bad Name', 'source': OPTION['source']},
                                 {'name': 'dex', 'source': OPTION['source']},
                                 {'name': 'x1', 'source': {'ref': 'r'}},
                                 {'name': 'x1', 'source': OPTION['source'], 'merge': 'yes'},
                                 {'name': 'x1', 'source': OPTION['source'], 'push': True}])
def test_malformed_library_options_are_refused(bad):
    with pytest.raises(ValueError):
        lib.library_config({'skill_library': bad})


def test_bimanual_examples_configure_the_library_and_validate():
    from services.xarm_bimanual.config import validate_options
    for path in sorted(ROOT.glob('examples/xarm-wuji-bimanual*.json')):
        options = json.loads(path.read_text())
        assert lib.library_config(options)['ref'] == 'skill-library/xarm-wuji-bimanual/dex', path
        validate_options(options)
    options['skill_library'] = {'name': 'dex', 'source': OPTION['source']}
    with pytest.raises(ValueError):
        validate_options(options)


# ---- checkout -----------------------------------------------------------------------------------------------------
def test_checkout_initializes_the_host_library_and_a_session_branch(tmp_path, source):
    config, workspace, info = session(tmp_path, 'S1')
    library = tmp_path / 'skill-library' / 'xarmwujibimanual'  # the artifact root is tmp_path (conftest)
    assert info['library'] == str(library) and info['branch'] == 'session/S1' and info['skills'] == 0
    assert git('rev-parse', '--abbrev-ref', 'HEAD', cwd=library).strip() == 'main'
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    assert git('rev-parse', '--abbrev-ref', 'HEAD', cwd=clone).strip() == 'session/S1'
    assert info['source_commit'] == git('rev-parse', 'skill-library/xarm-wuji-bimanual/dex', cwd=source).strip()
    (clone / 'INDEX.md').write_text('session edits')  # a refresh keeps the agent's work
    assert lib.checkout_library(config, workspace)['branch'] == 'session/S1'
    assert (clone / 'INDEX.md').read_text() == 'session edits'
    assert lib.checkout_library(Configuration(root=tmp_path / 'sessions' / 'S2', task='t', robot='mock'),
                                tmp_path) == {}


def test_rax_skill_library_selects_another_root(tmp_path, source, monkeypatch):
    monkeypatch.setenv(lib.ENVIRONMENT, str(tmp_path / 'own'))
    _, _, info = session(tmp_path, 'S1')
    assert info['library'] == str(tmp_path / 'own' / 'xarmwujibimanual')


# ---- the variant format and merges ----------------------------------------------------------------------------------
def test_a_valid_variant_merges_into_main_for_the_next_session(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    write_variant(clone)
    commit(clone)
    result = lib.merge(config, workspace, 'add a power-grasp variant')
    assert result['merged'] and result['skills'] == ['power-grasp--code-test'], result
    _, workspace2, info2 = session(tmp_path, 'S2')
    assert info2['skills'] == 1
    assert (workspace2 / 'skill-library/xarmwujibimanual/skills/power-grasp--code-test/code/run.py').is_file()


def test_format_problems_are_returned_and_nothing_merges(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    library = lib.library_path('xarmwujibimanual')
    before = git('rev-parse', 'main', cwd=library)
    meta = frontmatter('power-grasp--code-test', status='verified').replace('representation: code',
                                                                          'representation: rl')
    meta = meta.replace('  rate_hz: 1\n', '  rate_hz: 0\n').replace('"done: the thing is done"', 'done')
    write_variant(clone, meta=meta, runs=[run_line(backend='sim'), run_line(), run_line(version=0)],
                  entry='def run(x):\n    pass\n', body=SECTIONS.replace('## Checks', '## Notes'), params=False)
    commit(clone)
    text = ' '.join(lib.merge(config, workspace, 'bad')['problems'])
    assert git('rev-parse', 'main', cwd=library) == before
    for expected in ("representation 'rl' must equal the folder's 'code'", 'needs requires.checkpoint',
                     'control.rate_hz', '"<id>: <checkable statement>"', '"## Checks"', 'main(ctx',
                     'params.json missing', 'at least 2 unattended successful real runs'):
        assert expected in text, expected
    shutil.rmtree(clone / 'skills' / 'power-grasp--code-test')
    write_variant(clone, status='verified', runs=[run_line(), run_line(), run_line(success=False)])
    commit(clone)
    assert lib.merge(config, workspace, 'verified now')['merged']


def test_folder_names_must_follow_primitive_representation_tag(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    folder = write_variant(clone, 'power-grasp--code-test')
    folder.rename(clone / 'skills' / 'power_grasp')
    commit(clone)
    assert 'must be named <primitive>--<representation>' in ' '.join(lib.merge(config, workspace, 'x')['problems'])


def test_a_branch_may_only_change_skills_index_and_comparison(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    write_variant(clone)
    (clone / 'COMPARISON.md').write_text('# Variant comparison\n\n| power-grasp--code-test | 1/1 |\n')
    commit(clone)
    assert lib.merge(config, workspace, 'add')['merged']
    for path in ('README.md', 'library.json', 'lib/dexlib/x.py', 'skills/_template/SKILL.md'):
        (clone / path).parent.mkdir(parents=True, exist_ok=True)
        (clone / path).write_text('mine now')
        commit(clone)
        assert 'may only change' in ' '.join(lib.merge(config, workspace, path)['problems']), path
        git('reset', '-q', '--hard', 'HEAD~1', cwd=clone)
    git('rm', '-q', '-r', 'skills/power-grasp--code-test', cwd=clone)
    commit(clone)
    assert 'deprecated instead' in ' '.join(lib.merge(config, workspace, 'delete')['problems'])


def test_uses_must_name_skills_in_the_library(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    meta = frontmatter('pinch-grasp--code-test').replace('uses: []', 'uses: [power-grasp--code-missing]')
    write_variant(clone, 'pinch-grasp--code-test', meta=meta)
    commit(clone)
    assert "uses 'power-grasp--code-missing'" in ' '.join(lib.merge(config, workspace, 'x')['problems'])


def test_conflicting_sessions_get_conflicts_and_the_library_stays_clean(tmp_path, source):
    config1, workspace1, _ = session(tmp_path, 'S1')
    config2, workspace2, _ = session(tmp_path, 'S2')
    for workspace, text in ((workspace1, 'one'), (workspace2, 'two')):
        write_variant(workspace / 'skill-library/xarmwujibimanual', body=SECTIONS.replace('text', text, 1))
        commit(workspace / 'skill-library/xarmwujibimanual')
    assert lib.merge(config1, workspace1, 'first')['merged']
    result = lib.merge(config2, workspace2, 'second')
    assert not result['merged'] and 'skills/power-grasp--code-test/SKILL.md' in result['conflicts']
    assert git('status', '--porcelain', cwd=lib.library_path('xarmwujibimanual')) == ''


def test_the_harness_never_runs_the_agents_hooks_or_follows_links(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    marker = tmp_path / 'pwned'
    for hook in ('reference-transaction', 'post-checkout', 'post-merge'):
        (clone / '.git' / 'hooks' / hook).write_text(f'#!/bin/sh\ntouch {marker}\n')
        (clone / '.git' / 'hooks' / hook).chmod(0o755)
    write_variant(clone)
    (clone / 'skills' / 'power-grasp--code-test' / 'leak.md').symlink_to('/etc/hostname')
    commit(clone)
    marker.unlink(missing_ok=True)  # the agent's own commit ran its hooks
    result = lib.merge(config, workspace, 'evil')
    assert not marker.exists() and 'symlinks are not allowed' in ' '.join(result['problems'])


def test_read_only_libraries_refuse_merges_and_nothing_committed_is_reported(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    assert 'no committed changes' in lib.merge(config, workspace, 'nothing')['reason']
    config, workspace, _ = session(tmp_path, 'S2', {**OPTION, 'merge': False})
    with pytest.raises(lib.LibraryError, match='read-only'):
        lib.merge(config, workspace, 'x')


# ---- sync and fork -------------------------------------------------------------------------------------------------
def test_sync_merges_new_source_commits_and_never_pushes(tmp_path, source):
    config, workspace, _ = session(tmp_path, 'S1')
    clone = workspace / 'skill-library' / 'xarmwujibimanual'
    write_variant(clone)
    commit(clone)
    assert lib.merge(config, workspace, 'session work')['merged']
    (source / 'README.md').write_text('# updated by the maintainer\n')
    git('-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qam', 'maintainer', cwd=source)
    source_head = git('rev-parse', 'HEAD', cwd=source)
    result = lib.sync('xarmwujibimanual')
    library = lib.library_path('xarmwujibimanual')
    assert result['merged'] and (library / 'README.md').read_text() == '# updated by the maintainer\n'
    assert (library / 'skills/power-grasp--code-test/SKILL.md').is_file()
    assert git('rev-parse', 'HEAD', cwd=source) == source_head  # nothing pushed back
    assert 'push' in result['publish'] and 'not run' in result['publish']
    assert lib.sync('xarmwujibimanual')['reason'] == 'up to date'


def test_fork_starts_a_library_at_an_earlier_state(tmp_path, source, monkeypatch):
    config, workspace, _ = session(tmp_path, 'S1')
    shared = lib.library_path('xarmwujibimanual')
    git('tag', 'base', cwd=shared)
    write_variant(workspace / 'skill-library/xarmwujibimanual')
    commit(workspace / 'skill-library/xarmwujibimanual')
    assert lib.merge(config, workspace, 'later')['merged']
    forked = lib.fork(shared, 'base', tmp_path / 'own' / 'xarmwujibimanual')
    assert forked['skills'] == [] and not git('remote', cwd=tmp_path / 'own' / 'xarmwujibimanual').strip()
    monkeypatch.setenv(lib.ENVIRONMENT, str(tmp_path / 'own'))
    assert session(tmp_path, 'S2')[2]['skills'] == 0


# ---- the merge tool -----------------------------------------------------------------------------------------------
def test_the_frontend_offers_and_runs_skill_library_merge_only_with_a_clone(tmp_path, source):
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    from services.mock_backend import MockBackend
    config, workspace, _ = session(tmp_path, 'S1')
    frontend = Frontend(Session(config), MockBackend(config))
    assert 'skill_library_merge' in [t['name'] for t in frontend.tools()]
    write_variant(workspace / 'skill-library/xarmwujibimanual')
    commit(workspace / 'skill-library/xarmwujibimanual')
    reply = frontend.call('skill_library_merge', {'message': 'from the tool'})
    assert not reply.get('isError') and json.loads(reply['content'][0]['text'])['merged']
    assert frontend.call('skill_library_merge', {'message': ' '})['isError']
    bare = Configuration(root=tmp_path / 'sessions' / 'plain', task='t', robot='mock')
    assert 'skill_library_merge' not in [t['name'] for t in Frontend(Session(bare), MockBackend(bare)).tools()]


# ---- workspace wiring -----------------------------------------------------------------------------------------------
def test_a_bimanual_session_loads_the_library_skill_index_pointer_and_tool(tmp_path, mooncake_source):
    from harness.claude_cli.launcher import populate_workspace
    options = json.loads((ROOT / 'examples/xarm-wuji-bimanual.json').read_text())
    config = Configuration(root=tmp_path / 'session', robot='xarm_bimanual', task='grasp', backend_options=options)
    workspace = config.root / 'agent-workspace'
    built = populate_workspace(config, workspace)
    summary = built['skill_library']['xarmwujibimanual']
    assert summary['branch'] == 'session/session' and {'dex', 'dexmate'} <= set(built['skill_library'])
    assert (workspace / 'skill-library/xarmwujibimanual/INDEX.md').is_file()
    agents = (workspace / 'AGENTS.md').read_text()
    assert 'skill-library/xarmwujibimanual/INDEX.md' in agents and 'skill_library_merge' in agents
    skill = (workspace / '.agents/skills/dex-skill-library/SKILL.md').read_text()
    assert 'skill-library/xarmwujibimanual/' in skill and '{{' not in skill and '{%' not in skill
    assert 'skill_library_merge' in json.loads((workspace / 'MCP_CONTRACT.json').read_text())['tools']


def test_without_the_option_there_is_no_library_skill_or_tool(tmp_path):
    from services.workspace import Setup, build, recipe_for
    from wuji_fakes import hand_config
    build(recipe_for(), Setup('xarm', backend_options={'end_effector': hand_config()}), tmp_path / 'ws')
    assert not (tmp_path / 'ws/.agents/skills/dex-skill-library').exists()
    assert 'skill_library_merge' not in json.loads((tmp_path / 'ws/MCP_CONTRACT.json').read_text())['tools']
    assert 'skill-library/' not in (tmp_path / 'ws/AGENTS.md').read_text().split('screw-lightbulb')[-1]


def test_a_library_inside_a_protected_path_is_refused(tmp_path, monkeypatch):
    from harness.claude_cli import launcher
    monkeypatch.setenv(lib.ENVIRONMENT, str(tmp_path / 'secret' / 'libraries'))
    monkeypatch.setattr(launcher, 'PROTECTED_PATHS', (tmp_path / 'secret',))
    config = Configuration(root=tmp_path / 'sessions' / 'S1', task='t', robot='mock',
                           backend_options={'skill_library': OPTION})
    with pytest.raises(ValueError, match='inside protected path'):
        launcher.validate_workspace_location(config)


# ---- the host-side API ----------------------------------------------------------------------------------------------
class Ctx:
    def __init__(self):
        self.calls = []

    def value(self, tool, **arguments):
        self.calls.append((tool, arguments))
        return {}


@pytest.fixture
def checkout(tmp_path):
    library = write_minimal_library(tmp_path / 'checkout')
    write_variant(library)
    git('init', '-q', '-b', 'main', cwd=library)
    git('add', '-A', cwd=library)
    git('-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'x', cwd=library)
    return library


def test_load_and_run_a_skill_with_a_given_ctx(checkout):
    assert [s['name'] for s in lib.list_skills(checkout)] == ['power-grasp--code-test']
    skill = lib.load_skill('power-grasp--code-test', checkout)
    assert skill.defaults == {'side': 'right'} and skill.meta['representation'] == 'code' and skill.version == 1
    ctx, seen = Ctx(), []
    result = lib.run_skill(ctx, 'power-grasp--code-test', {'side': 'left'}, library=checkout,
                           progress=lambda stage, fraction, **info: seen.append((stage, fraction)))
    assert result['ok'] and result['measured']['status'] == 'success' and result['success_check'] is True
    assert ctx.calls == [('wuji_status', {'hand': 'left'})] and seen == [('act', 0.5)]
    assert result['skill']['commit'] == git('rev-parse', 'HEAD', cwd=checkout).strip()
    refused = lib.run_skill(ctx, 'power-grasp--code-test', {'refuse': True}, library=checkout)
    assert refused['measured']['status'] == 'failed_precondition' and len(ctx.calls) == 1
    crashed = lib.run_skill(ctx, 'power-grasp--code-test', {'explode': True}, library=checkout)
    assert crashed['measured']['status'] == 'aborted' and 'boom' in crashed['why']
    doubted = lib.run_skill(ctx, 'power-grasp--code-test', {'unconvinced': True}, library=checkout)
    assert not doubted['ok'] and doubted['measured']['status'] == 'failed'
    with pytest.raises(lib.LibraryError):
        lib.load_skill('_template', checkout)
    entry = checkout / 'skills/power-grasp--code-test/code/run.py'
    entry.write_text(entry.read_text() + '\n# an uncommitted edit\n')  # evidence must not name a clean commit
    assert lib.run_skill(ctx, 'power-grasp--code-test', library=checkout)['skill']['commit'].endswith('-dirty')


def test_load_skill_main_resolves_the_names_dex_eval_tries(checkout):
    write_variant(checkout, 'in-hand-rotate--rl', meta=frontmatter('in-hand-rotate--rl').replace(
        'checkpoint: none', 'checkpoint: checkpoints/policy.onnx'))
    main = lib.load_skill_main('in-hand-rotate--rl', str(checkout))
    assert callable(main) and main(Ctx(), side='right')['ok']
    for name in ('in-hand-rotate/rl', 'in-hand-rotate/in-hand-rotate--rl', 'in-hand-rotate'):
        assert lib.resolve_name(name, checkout) == 'in-hand-rotate--rl', name
    write_variant(checkout, 'in-hand-rotate--code-gait')
    for name in ('in-hand-rotate', 'in-hand-rotate_rl', 'rl', 'missing', '_template', 'power-grasp/rl'):
        with pytest.raises(LookupError):  # services/dex_eval goes on to its next candidate name
            lib.load_skill_main(name, str(checkout))


def test_dex_eval_finds_the_library_loader(checkout, monkeypatch):
    skills = pytest.importorskip('services.dex_eval.skills')
    assert skills.library_loader() is lib.load_skill_main
    monkeypatch.setenv(lib.CHECKOUT_ENVIRONMENT, str(checkout))
    main, source = skills.resolve('power-grasp', 'code-test')
    assert main(Ctx())['ok'] and 'load_skill_main' in source


def test_checkpoints_live_in_the_skill_folder_and_may_be_large(checkout):
    folder = write_variant(checkout, 'in-hand-rotate--rl', meta=frontmatter('in-hand-rotate--rl').replace(
        'checkpoint: none', 'checkpoint: checkpoints/policy.onnx'))
    assert lib.validate_skill(folder) == []  # a draft's checkpoint may arrive later (trained elsewhere)
    (folder / 'checkpoints').mkdir()
    (folder / 'checkpoints' / 'policy.onnx').write_bytes(b'\0' * 3_000_000)  # over the 2 MB code-file limit
    assert lib.validate_skill(folder) == []
    (folder / 'code' / 'blob.bin').write_bytes(b'\0' * 3_000_000)
    assert any('blob.bin is over 2 MB' in p for p in lib.validate_skill(folder))
    (folder / 'code' / 'blob.bin').unlink()
    escaping = frontmatter('in-hand-rotate--rl').replace('checkpoint: none', 'checkpoint: ../other/policy.onnx')
    (folder / 'SKILL.md').write_text(escaping + '\n# Variant\n\n' + SECTIONS)
    assert any('must be inside the skill folder' in p for p in lib.validate_skill(folder))
    verified = frontmatter('in-hand-rotate--rl', status='verified').replace('checkpoint: none',
                                                                            'checkpoint: checkpoints/gone.onnx')
    (folder / 'SKILL.md').write_text(verified + '\n# Variant\n\n' + SECTIONS)
    (folder / 'evidence' / 'runs.jsonl').write_text(json.dumps(run_line()) + '\n' + json.dumps(run_line()) + '\n')
    assert any('verified needs its checkpoint' in p for p in lib.validate_skill(folder))
    (folder / 'evidence' / 'runs.jsonl').write_text(json.dumps(run_line(backend='mock')) + '\n')
    assert not any('backend' in p for p in lib.validate_skill(folder))  # mock runs are recordable evidence


def test_run_records_append_to_evidence_and_validate(checkout):
    result = lib.run_skill(Ctx(), 'power-grasp--code-test', library=checkout)
    record = lib.run_record(result, trial='runs/t1', backend='real', params={}, session='trial-runner',
                            unattended=True, conditions='cube 5 cm', smoothness=None)
    path = lib.append_run('power-grasp--code-test', record, library=checkout)
    assert json.loads(path.read_text().splitlines()[-1])['success'] is True
    assert lib.validate_library(checkout) == []
    with pytest.raises(ValueError):
        lib.append_run('power-grasp--code-test', {'date': 'x'}, library=checkout)


# ---- the real library (the skill-library/xarm-wuji-bimanual/dex worktree), when present ------------------------------
real = pytest.mark.skipif(not (REAL_LIBRARY / 'library.json').is_file(),
                          reason=f'no library checkout at {REAL_LIBRARY} (set RAX_TEST_DEX_LIBRARY)')


@real
def test_the_real_library_validates_and_its_template_is_a_valid_start():
    assert lib.validate_library(REAL_LIBRARY) == []
    assert {s['primitive'] for s in lib.list_skills(REAL_LIBRARY)} >= {'power-grasp', 'pinch-grasp', 'in-hand-rotate'}


@real
def test_the_template_copied_to_a_variant_name_passes_validation(tmp_path):
    copy = tmp_path / 'library'
    shutil.copytree(REAL_LIBRARY, copy, ignore=shutil.ignore_patterns('.git', '__pycache__'))
    folder = copy / 'skills' / 'example--code-new'
    shutil.copytree(copy / 'skills' / '_template', folder)
    (folder / 'code' / '_template').rename(folder / 'code' / 'example__code_new')
    text = (folder / 'SKILL.md').read_text().replace('name: _template', 'name: example--code-new')
    (folder / 'SKILL.md').write_text(text)
    (copy / 'INDEX.md').write_text((copy / 'INDEX.md').read_text() + '| [x](skills/example--code-new/SKILL.md) |\n')
    assert lib.validate_skill(folder) == [] and lib.validate_library(copy) == []


@real
def test_the_real_variants_run_through_the_host_api_on_a_fake_ctx():
    import sys
    sys.path.insert(0, str(REAL_LIBRARY / 'lib'))
    from dexlib import hand, postures
    from dexlib.testing import FakeCtx
    q = postures.synergy('power', 0.4)
    ctx = FakeCtx(blocks={'right': {n: q[i] for i, n in enumerate(hand.JOINT_NAMES) if n.endswith(('_S1', '_S3'))}})
    result = lib.run_skill(ctx, 'power-grasp--code-synergy', {'side': 'right', 'confirm_interval_s': 0.0,
                                                               'hold_check_s': 0.0}, library=REAL_LIBRARY)
    assert result['ok'] and result['success_check'] and result['skill']['representation'] == 'code', result['why']
    refused = lib.run_skill(FakeCtx(), 'power-grasp--mpc-cem', {'side': 'right'}, library=REAL_LIBRARY)
    assert refused['measured']['status'] == 'failed_precondition'
    assert 'object-described' in refused['measured']['preconditions']['failed']
    rotate = lib.load_skill_main('in-hand-rotate/rl', str(REAL_LIBRARY))(FakeCtx(), side='right')
    assert rotate['measured']['status'] == 'failed_precondition'  # no checkpoints/policy.onnx yet
    assert 'checkpoint-present' in rotate['measured']['preconditions']['failed']
