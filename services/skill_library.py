"""Skill libraries in RAX sessions: read-only references, the rig's own library, and a host-side load/run API.

1. **Reference libraries** (`checkout(workspace)`): Mooncake's dex and dexmate libraries at the commits pinned in
   the screw-lightbulb skill's `references/sources.json`, cloned into `skill-library/dex/` and
   `skill-library/dexmate/` on a local `session` branch. Never merged.

2. **The rig library** (ported from mooncake `sim2real/dexmate` services/skill_library.py), configured by the
   backend option `skill_library` (examples/xarm-wuji-bimanual*.json):

       "skill_library": {"name": "xarmwujibimanual",
                         "source": {"repository": "<git URL or path>", "ref": "skill-library/xarmwujibimanual/dex"},
                         "merge": true}

       <RAX_SKILL_LIBRARY or <artifact root>/skill-library>/<name>/  the host library (git, branch main); only the
                                                                      harness writes it; initialized once from
                                                                      source.ref (RAX_MOONCAKE_REPO, the sibling
                                                                      ../mooncake, then source.repository)
       <session>/agent-workspace/skill-library/<name>/                the session's clone, branch session/<session>

   The agent reads `INDEX.md`, runs skills, and commits new or improved variants on its branch;
   `merge()` (the harness tool `skill_library_merge`) validates the branch as data (code is compiled, never run)
   and merges it into the host main. The harness never runs git inside the agent's clone (its config and hooks are
   the agent's): it copies the clone's object store to a scratch directory and fetches from there. Nothing is ever
   pushed: `sync()` merges new source commits into the host main and prints the push that would publish it.

3. **Host-side API** for runners on the host (not the agent sandbox): `load_skill_main(name, library_root=None)`
   (the callable `main(ctx, **params)`; services/dex_eval looks it up by that name), `load_skill` (the richer
   `Skill`: main, check_preconditions, success, meta, params), `resolve_name`, `list_skills`, `run_skill`,
   `run_record`, `append_run`. They import a library checkout's skill code into the calling process.

The rig library's format (validate_skill; the library's README.md documents it): a folder per variant
`skills/<primitive>--<representation>[-<tag>]/` with SKILL.md (YAML frontmatter: name, description, status, robot,
entry, version, uses, primitive, representation, hands, requires{perception, sim_model, checkpoint, gpu,
hand_features}, control{mode, rate_hz, impedance}, preconditions, postconditions, progress; and the seven
sections with a perception-checkpoint table), code/run.py defining main(ctx, ...), code/<package>/ with
params.json, and evidence/runs.jsonl whose lines point at trial directories in the shared trace format.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import fcntl
import importlib.util
import inspect
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

PROJECT = Path(__file__).resolve().parents[1]
SOURCES = PROJECT / 'agent_skills/end_effectors/wuji_hand2/screw-lightbulb/references/sources.json'

LIBRARY_DIR = 'skill-library'
ENVIRONMENT = 'RAX_SKILL_LIBRARY'           # root of the host libraries (<root>/<name>), as in mooncake
CHECKOUT_ENVIRONMENT = 'RAX_SKILL_LIBRARY_CHECKOUT'  # the host API's default library checkout
DEFAULT_NAME = 'xarmwujibimanual'
STATUSES = ('draft', 'verified', 'deprecated')
SECTIONS = ('When to use', 'Preconditions', 'Procedure', 'Code', 'Checks', 'Failure modes', 'Evidence')
REPRESENTATIONS = ('code', 'mpc', 'rl', 'imitation', 'hybrid')
HANDS = ('left', 'right', 'both', 'either')
GPU = ('none', 'optional', 'required')
HAND_FEATURES = ('contact_completion', 'effort', 'impedance')
CONTROL_MODES = ('blocking', 'streaming')
RESULT_STATUSES = ('success', 'failed_precondition', 'failed', 'dropped', 'timeout', 'aborted')
BACKENDS = ('real', 'sim', 'mock')  # services/dex_eval's backends; only real runs count toward verified
RUN_KEYS = ('date', 'session', 'commit', 'version', 'backend', 'trial', 'params', 'success', 'status', 'unattended',
            'conditions', 'smoothness')
PARAM_KEYS = ('controls', 'value', 'unit', 'tested', 'tune')
VERIFIED_MIN_RUNS = 2       # unattended successful REAL runs of the current version
MAX_IMAGE_BYTES = 400_000
MAX_FILE_BYTES = 2_000_000
MAX_SKILL_BYTES = 25_000_000           # everything but checkpoints/
CHECKPOINT_DIR = 'checkpoints'          # <skill>/checkpoints/: policy files of rl/imitation variants (ONNX, npz)
MAX_CHECKPOINT_BYTES = 200_000_000      # per checkpoint file
SESSION_EDITABLE = ('INDEX.md', 'COMPARISON.md')
VARIANT = re.compile(r'^(?P<primitive>[a-z0-9]+(?:-[a-z0-9]+)*)--(?P<representation>[a-z]+)'
                     r'(?:-(?P<tag>[a-z0-9]+(?:-[a-z0-9]+)*))?$')
NAME = re.compile(r'^[a-z0-9][a-z0-9-]{1,40}$')
CONDITION = re.compile(r'^[a-z0-9][a-z0-9-]*: \S')
LOOSE_OBJECTS = 256
GIT_S, SLOW_GIT_S = 60, 900
GIT_ENV = {'GIT_CONFIG_NOSYSTEM': '1', 'GIT_TERMINAL_PROMPT': '0'}


class LibraryError(RuntimeError):
    pass


class SkillNotFound(LibraryError, LookupError):
    """No variant of that name in the library (a LookupError, so resolvers that try several names go on)."""


def _git(path, *args):
    """git in `path` (reference libraries; raises RuntimeError like the original helper)."""
    result = subprocess.run(['git', '-C', str(path), *args], capture_output=True, text=True, timeout=180)
    if result.returncode:
        raise RuntimeError(f'Mooncake skill library: {result.stderr.strip()}')
    return result.stdout.strip()


def _run(*args, cwd=None, check=True, timeout=GIT_S):
    """git for the rig library, without the user's global hooks or prompts."""
    result = subprocess.run(['git', *args], cwd=cwd, capture_output=True, text=True, timeout=timeout,
                            env={**os.environ, **GIT_ENV})
    if check and result.returncode:
        raise LibraryError(f"git {' '.join(args)}: {(result.stderr or result.stdout).strip()[:500]}")
    return result


def _mooncake_repository(configured):
    sibling = PROJECT.parent / 'mooncake'
    return os.environ.get('RAX_MOONCAKE_REPO') or (str(sibling) if sibling.exists() else configured)


# ===== 1. reference libraries (unchanged behaviour) ================================================================
def checkout(workspace):
    workspace = Path(workspace)
    if not (workspace / '.agents/skills/screw-lightbulb/SKILL.md').is_file():
        return {}
    sources = json.loads(SOURCES.read_text())
    repository = _mooncake_repository(sources['repository'])
    root = workspace / LIBRARY_DIR
    if root.is_symlink():
        raise ValueError(f'Refusing symlinked skill library: {root}')
    root.mkdir(exist_ok=True)
    result = {}
    for robot, source in sources['sources'].items():
        destination = root / robot
        if destination.is_symlink():
            raise ValueError(f'Refusing symlinked skill library: {destination}')
        if not destination.exists():
            with tempfile.TemporaryDirectory(prefix=f'.{robot}-', dir=root) as temporary:
                pending = Path(temporary) / 'library'
                pending.mkdir()
                _git(pending, 'init', '-q')
                _git(pending, 'remote', 'add', 'origin', repository)
                _git(pending, 'fetch', '-q', '--depth=1', 'origin', source['commit'])
                _git(pending, 'checkout', '-q', '-b', 'session', source['commit'])
                pending.rename(destination)
        if not (destination / '.git').is_dir():
            raise ValueError(f'Expected a session Git library: {destination}')
        result[robot] = {'path': str(destination), 'source_ref': source['ref'],
                         'source_commit': source['commit'], 'head': _git(destination, 'rev-parse', 'HEAD')}
    return result


# ===== 2. the rig library ==========================================================================================
def library_config(options):
    """The normalized `skill_library` backend option, or None. Raises ValueError when it is malformed."""
    value = (options or {}).get('skill_library')
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) - {'name', 'source', 'merge'}:
        raise ValueError('skill_library takes name, source {repository, ref[, commit]} and merge')
    name = value.get('name')
    if not isinstance(name, str) or not NAME.match(name):
        raise ValueError('skill_library.name must be lower-case letters, digits and hyphens')
    reserved = set(json.loads(SOURCES.read_text())['sources']) if SOURCES.is_file() else {'dex', 'dexmate'}
    if name in reserved:
        raise ValueError(f'skill_library.name {name!r} is a reference library name')
    source = value.get('source')
    if not isinstance(source, dict) or set(source) - {'repository', 'ref', 'commit'}:
        raise ValueError('skill_library.source takes repository, ref and optional commit')
    for key in ('repository', 'ref'):
        if not isinstance(source.get(key), str) or not source[key].strip():
            raise ValueError(f'skill_library.source.{key} must be a non-empty string')
    commit = source.get('commit')
    if commit is not None and not re.fullmatch(r'[0-9a-f]{7,40}', str(commit)):
        raise ValueError('skill_library.source.commit must be a hex commit id')
    if 'merge' in value and type(value['merge']) is not bool:
        raise ValueError('skill_library.merge must be true or false')
    return {'name': name, 'repository': source['repository'], 'ref': source['ref'], 'commit': commit,
            'merge': value.get('merge', True)}


def library_root():
    configured = os.environ.get(ENVIRONMENT)
    if configured:
        return Path(configured)
    from services.storage_root import artifact_root
    return artifact_root() / LIBRARY_DIR


def library_path(name):
    """The host library of `name`: RAX_SKILL_LIBRARY/<name>, else <artifact root>/skill-library/<name>."""
    return library_root() / name


def ensure_library(cfg):
    """The host library repository for a normalized config, initialized once from its source ref (branch main)."""
    path = library_path(cfg['name'])
    if (path / '.git').is_dir():
        return path
    if path.exists():
        raise LibraryError(f'{path} exists but is not a git library')
    path.parent.mkdir(parents=True, exist_ok=True)
    repository = _mooncake_repository(cfg['repository'])
    with tempfile.TemporaryDirectory(prefix=f'.{cfg["name"]}-', dir=path.parent) as temporary:
        pending = Path(temporary) / 'library'
        pending.mkdir()
        _run('init', '-q', '-b', 'main', cwd=pending)
        _run('remote', 'add', 'source', repository, cwd=pending)
        _run('fetch', '-q', '--no-tags', 'source', cfg['commit'] or cfg['ref'], cwd=pending, timeout=SLOW_GIT_S)
        _run('checkout', '-q', '-B', 'main', 'FETCH_HEAD', cwd=pending)
        _run('config', 'user.name', 'rax harness', cwd=pending)
        _run('config', 'user.email', 'harness@rax.local', cwd=pending)
        commit = _run('rev-parse', 'HEAD', cwd=pending).stdout.strip()
        (pending / '.git' / 'rax-library.json').write_text(json.dumps({
            'name': cfg['name'], 'source_repository': repository, 'source_ref': cfg['ref'],
            'source_commit': commit, 'initialized_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}, indent=2) + '\n')
        pending.rename(path)
    return path


def branch_name(config):
    return f'session/{Path(config.root).name}'


def clone_path(workspace, name):
    return Path(workspace) / LIBRARY_DIR / name


def _summary(cfg, library, clone, branch):
    record = library / '.git' / 'rax-library.json'
    origin = json.loads(record.read_text()) if record.is_file() else {}
    return {'name': cfg['name'], 'library': str(library), 'clone': str(clone), 'branch': branch,
            'head': _run('rev-parse', 'HEAD', cwd=clone).stdout.strip(), 'merge': cfg['merge'],
            'source_ref': cfg['ref'], 'source_commit': origin.get('source_commit'),
            'skills': len([p for p in (clone / 'skills').glob('*/SKILL.md') if not p.parent.name.startswith('_')])}


def checkout_library(config, workspace):
    """Clone the rig library into the workspace on the session's branch (kept when already there); {} when the
    session has no `skill_library` option. Returns its summary for the workspace manifest."""
    cfg = library_config(config.backend_options)
    if cfg is None:
        return {}
    library = ensure_library(cfg)
    root = Path(workspace) / LIBRARY_DIR
    if root.is_symlink():
        raise ValueError(f'Refusing symlinked skill library: {root}')
    root.mkdir(exist_ok=True)
    clone = clone_path(workspace, cfg['name'])
    if clone.is_symlink():
        raise ValueError(f'Refusing symlinked skill library: {clone}')
    branch = branch_name(config)
    if not clone.exists():
        with open(library / '.git' / 'rax-merge.lock', 'w') as lock:  # no merge while it packs and copies
            fcntl.flock(lock, fcntl.LOCK_EX)
            _run('-c', f'gc.auto={LOOSE_OBJECTS}', '-c', 'gc.autoDetach=false', 'gc', '--auto', '--quiet',
                 cwd=library, timeout=SLOW_GIT_S)
            _run('clone', '-q', '--no-hardlinks', str(library), str(clone), timeout=SLOW_GIT_S)
        _run('checkout', '-q', '-b', branch, cwd=clone)
        _run('config', 'user.name', f'agent {Path(config.root).name}', cwd=clone)
        _run('config', 'user.email', 'agent@rax.local', cwd=clone)
    elif not (clone / '.git').is_dir():
        raise ValueError(f'Expected a session Git library: {clone}')
    return _summary(cfg, library, clone, branch)


# ----- validation (files of a commit, read as data) ----------------------------------------------------------------
def parse_frontmatter(text):
    """The YAML frontmatter of a SKILL.md as a dict, or None when it has none."""
    match = re.match(r'^---\n(.*?)\n---\n', text, re.S)
    if not match:
        return None
    try:
        import yaml
        value = yaml.safe_load(match.group(1))
        return value if isinstance(value, dict) else None
    except ImportError:  # pragma: no cover - the robot environment has pyyaml
        fields = {}
        for line in match.group(1).splitlines():
            if ':' in line and not line.startswith((' ', '-')):
                key, value = line.split(':', 1)
                fields[key.strip()] = value.strip().strip('"\'')
        return fields
    except Exception:  # noqa: BLE001 -- yaml.YAMLError and friends: report as malformed
        return None


def _manifest(tree):
    path = Path(tree) / 'library.json'
    try:
        return json.loads(path.read_text()) if path.is_file() and not path.is_symlink() else {}
    except ValueError:
        return {}


def _empty(value):
    return value is None or (isinstance(value, str) and value.strip().lower() in ('', 'none', 'null'))


def _check_frontmatter(meta, slug, manifest, bad):
    for key in ('name', 'description', 'status', 'robot', 'entry', 'version', 'primitive', 'representation', 'hands',
                'requires', 'control', 'preconditions', 'postconditions', 'progress'):
        if key not in meta or meta[key] in (None, '', [], {}):
            bad(f'frontmatter needs {key}')
    if meta.get('name') not in (None, slug):
        bad(f"frontmatter name {meta['name']!r} must equal the folder name")
    if meta.get('status') not in (None, *STATUSES):
        bad(f'status must be one of {", ".join(STATUSES)}')
    robot = manifest.get('robot')
    if robot and meta.get('robot') not in (None, robot):
        bad(f"robot must be {robot!r} (library.json)")
    if 'version' in meta and (type(meta['version']) is not int or meta['version'] < 1):
        bad('version must be a positive integer')
    parsed = VARIANT.match(slug)
    if not parsed:
        bad('folder must be named <primitive>--<representation>[-<tag>] (lower-case words with hyphens)')
    else:
        if meta.get('primitive') not in (None, parsed['primitive']):
            bad(f"primitive {meta['primitive']!r} must equal the folder's {parsed['primitive']!r}")
        if meta.get('representation') not in (None, parsed['representation']):
            bad(f"representation {meta['representation']!r} must equal the folder's {parsed['representation']!r}")
    if meta.get('representation') is not None and meta['representation'] not in REPRESENTATIONS:
        bad(f'representation must be one of {", ".join(REPRESENTATIONS)}')
    if meta.get('hands') is not None and meta['hands'] not in HANDS:
        bad(f'hands must be one of {", ".join(HANDS)}')
    uses = meta.get('uses', [])
    if uses is not None and not (isinstance(uses, list) and all(isinstance(u, str) for u in uses)):
        bad('uses must be a list of skill names')
    requires = meta.get('requires')
    if isinstance(requires, dict):
        missing = [k for k in ('perception', 'sim_model', 'checkpoint', 'gpu') if k not in requires]
        if missing:
            bad(f'requires needs {missing}')
        if requires.get('gpu') not in (None, *GPU):
            bad(f'requires.gpu must be one of {", ".join(GPU)}')
        features = requires.get('hand_features', [])
        if not isinstance(features, list) or set(features) - set(HAND_FEATURES):
            bad(f'requires.hand_features must be a list drawn from {", ".join(HAND_FEATURES)}')
        representation = meta.get('representation')
        if representation in ('rl', 'imitation') and _empty(requires.get('checkpoint')):
            bad(f'a {representation} variant needs requires.checkpoint')
        if representation == 'mpc' and _empty(requires.get('sim_model')):
            bad('an mpc variant needs requires.sim_model (the model it plans on)')
    elif requires is not None:
        bad('requires must be a mapping')
    control = meta.get('control')
    if isinstance(control, dict):
        if control.get('mode') not in CONTROL_MODES:
            bad(f'control.mode must be one of {", ".join(CONTROL_MODES)}')
        rate = control.get('rate_hz')
        if type(rate) not in (int, float) or not rate > 0:
            bad('control.rate_hz must be a positive number')
        if _empty(control.get('impedance')):
            bad('control.impedance is required (profile, a named profile, or {kp, kd, current_limit_a})')
    elif control is not None:
        bad('control must be a mapping')
    for key in ('preconditions', 'postconditions'):
        items = meta.get(key)
        if items is None:
            continue
        if not isinstance(items, list) or not items:
            bad(f'{key} must be a non-empty list of "<id>: <statement>"')
        elif not all(isinstance(i, str) and CONDITION.match(i) for i in items):
            bad(f'each of {key} must read "<id>: <checkable statement>"')
    progress = meta.get('progress')
    if progress is not None and not (isinstance(progress, list) and progress
                                     and all(isinstance(s, str) and s for s in progress)):
        bad('progress must be a non-empty list of stage names')


def checkpoint_path(folder, meta):
    """The skill's requires.checkpoint as a Path (relative paths are from the skill folder; $VARS and ~ expand),
    or None when it names none or a URL."""
    raw = (meta.get('requires') or {}).get('checkpoint') if isinstance(meta.get('requires'), dict) else None
    if _empty(raw) or not isinstance(raw, str) or '://' in raw:
        return None
    path = Path(os.path.expandvars(os.path.expanduser(raw.strip())))
    return path if path.is_absolute() else Path(folder) / path


def _check_checkpoint(folder, meta, bad):
    """A checkpoint named relative to the skill must stay inside it (normally checkpoints/<file>); it may arrive
    later (a draft trained elsewhere), but a verified variant's must be present."""
    raw = (meta.get('requires') or {}).get('checkpoint') if isinstance(meta.get('requires'), dict) else None
    if _empty(raw) or not isinstance(raw, str) or '://' in raw or raw.strip().startswith(('/', '$', '~')):
        return
    path = (Path(folder) / raw.strip()).resolve()
    if not path.is_relative_to(Path(folder).resolve()):
        bad(f'requires.checkpoint {raw!r} must be inside the skill folder (e.g. {CHECKPOINT_DIR}/policy.onnx)')
    elif meta.get('status') == 'verified' and not path.is_file():
        bad(f'verified needs its checkpoint {raw!r} in the skill folder')


def _check_runs(folder, meta, bad):
    runs_file = folder / 'evidence' / 'runs.jsonl'
    images = folder / 'evidence' / 'images'
    runs = []
    if not runs_file.is_file():
        bad('evidence/runs.jsonl missing (an empty file until the first run)')
        return runs
    for n, line in enumerate(runs_file.read_text(errors='replace').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            run = json.loads(line)
        except json.JSONDecodeError:
            bad(f'evidence/runs.jsonl line {n} is not JSON')
            continue
        if not isinstance(run, dict):
            bad(f'evidence/runs.jsonl line {n} must be an object')
            continue
        missing = [k for k in RUN_KEYS if k not in run]
        if missing:
            bad(f'evidence/runs.jsonl line {n} lacks {missing}')
            continue
        if run['backend'] not in BACKENDS:
            bad(f'evidence/runs.jsonl line {n}: backend must be one of {", ".join(BACKENDS)}')
        if run['status'] not in RESULT_STATUSES:
            bad(f'evidence/runs.jsonl line {n}: status must be one of {", ".join(RESULT_STATUSES)}')
        for key in ('success', 'unattended'):
            if not isinstance(run[key], bool):
                bad(f'evidence/runs.jsonl line {n}: {key} must be true or false')
        if run['success'] is True and run['status'] != 'success':
            bad(f'evidence/runs.jsonl line {n}: success true needs status success')
        if not isinstance(run['trial'], str) or not run['trial'].strip():
            bad(f'evidence/runs.jsonl line {n}: trial must name the trial directory')
        if not isinstance(run['params'], dict):
            bad(f'evidence/runs.jsonl line {n}: params must be an object')
        if run['smoothness'] is not None and not isinstance(run['smoothness'], dict):
            bad(f'evidence/runs.jsonl line {n}: smoothness must be an object or null')
        for image in run.get('images') or []:
            path = (folder / image).resolve()
            if not path.is_relative_to(images.resolve()) or not path.is_file():
                bad(f'evidence/runs.jsonl line {n}: image {image} is not a file under evidence/images/')
        runs.append(run)
    if meta.get('status') == 'verified':
        good = [r for r in runs if r.get('success') is True and r.get('unattended') is True
                and r.get('backend') == 'real' and r.get('version') == meta.get('version')]
        if len(good) < VERIFIED_MIN_RUNS:
            bad(f'verified needs at least {VERIFIED_MIN_RUNS} unattended successful real runs of version '
                f'{meta.get("version")} in evidence/runs.jsonl (has {len(good)}); keep it draft until then')
    return runs


def validate_skill(folder, manifest=None):
    """Problems of one variant folder (a list of strings; empty when it follows the format)."""
    folder = Path(folder)
    slug = folder.name
    problems = []

    def bad(message):
        problems.append(f'skills/{slug}: {message}')
    links = [p for p in [folder, *folder.rglob('*')] if p.is_symlink()]
    if links:  # refuse before reading anything through a link
        bad(f'symlinks are not allowed: {[str(p.relative_to(folder.parent)) for p in links][:5]}')
        return problems
    files = [p for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    checkpoints = folder / CHECKPOINT_DIR
    weights = [p for p in files if p.is_relative_to(checkpoints)]
    total = sum(p.stat().st_size for p in files if p not in weights)
    if total > MAX_SKILL_BYTES:
        bad(f'{total / 1e6:.1f} MB is over the {MAX_SKILL_BYTES / 1e6:g} MB per skill (checkpoints/ excluded)')
    for p in files:
        limit = MAX_CHECKPOINT_BYTES if p in weights else MAX_FILE_BYTES
        if p.stat().st_size > limit:
            bad(f'{p.relative_to(folder)} is over {limit / 1e6:g} MB')
    skill = folder / 'SKILL.md'
    if not skill.is_file():
        bad('SKILL.md missing')
        return problems
    text = skill.read_text(errors='replace')
    meta = parse_frontmatter(text)
    if meta is None:
        bad('SKILL.md must begin with --- YAML frontmatter ---')
        return problems
    _check_frontmatter(meta, slug, manifest if manifest is not None else _manifest(folder.parents[1]), bad)
    _check_checkpoint(folder, meta, bad)
    headings = set(re.findall(r'^##\s+(.+?)\s*$', text, re.M))
    for section in SECTIONS:
        if section not in headings:
            bad(f'SKILL.md needs a "## {section}" section')
    if not re.search(r'^###\s+Perception checkpoints\s*$', text, re.M):
        bad('SKILL.md needs a "### Perception checkpoints" table under "## Checks" (or "None")')
    entry = meta.get('entry')
    if isinstance(entry, str) and entry:
        code = (folder / entry).resolve()
        if not code.is_relative_to(folder.resolve()) or not code.is_file() or code.suffix != '.py':
            bad(f'entry {entry} must be a .py file inside the skill folder')
        else:
            source = code.read_text(errors='replace')
            try:
                compile(source, entry, 'exec')  # syntax only: nothing runs
            except (SyntaxError, ValueError) as exc:
                bad(f'entry {entry} does not compile: {str(exc)[:200]}')
            if not re.search(r'^def main\(\s*ctx\b', source, re.M):
                bad(f'entry {entry} must define main(ctx, ...)')
            for optional in ('check_preconditions', 'success'):
                if re.search(rf'^def {optional}\(', source, re.M) and not re.search(
                        rf'^def {optional}\(\s*ctx\b', source, re.M):
                    bad(f'entry {entry}: {optional} must take ctx first')
            if code.parent != (folder / 'code').resolve():
                bad(f'entry {entry} must be a file directly in code/ (it imports the package next to it)')
    package = slug.replace('-', '_')
    if not (folder / 'code' / package / '__init__.py').is_file():
        bad(f'code/{package}/__init__.py missing: the modules go in a package named after the skill')
    params_file = folder / 'code' / package / 'params.json'
    if not params_file.is_file():
        bad(f'code/{package}/params.json missing: every tunable value is data')
    else:
        try:
            data = json.loads(params_file.read_text())
            if not isinstance(data, dict):
                raise ValueError('not an object')
            for name, entry_ in data.items():
                missing = [k for k in PARAM_KEYS if not isinstance(entry_, dict) or k not in entry_]
                if missing:
                    bad(f'params.json {name!r} lacks {missing}')
        except ValueError as exc:
            bad(f'params.json is not a JSON object: {str(exc)[:120]}')
    for p in folder.rglob('*.py'):
        if '__pycache__' in p.parts or (isinstance(entry, str) and p.resolve() == (folder / entry).resolve()):
            continue
        try:
            compile(p.read_text(errors='replace'), str(p.relative_to(folder)), 'exec')
        except (SyntaxError, ValueError) as exc:
            bad(f'{p.relative_to(folder)} does not compile: {str(exc)[:200]}')
    images = folder / 'evidence' / 'images'
    for p in (images.glob('*') if images.is_dir() else ()):
        if p.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
            bad(f'{p.relative_to(folder)}: evidence images are jpg or png')
        elif p.stat().st_size > MAX_IMAGE_BYTES:
            bad(f'{p.relative_to(folder)} is {p.stat().st_size // 1000} kB (at most {MAX_IMAGE_BYTES // 1000} kB)')
    for target in re.findall(r'!\[[^\]]*\]\(([^)\s]+)\)', text):
        if '://' not in target and not (folder / target).is_file():
            bad(f'SKILL.md shows {target}, which is not a file in the skill')
    _check_runs(folder, meta, bad)
    return problems


def _skill_folders(tree):
    skills = Path(tree) / 'skills'
    return sorted(p for p in skills.iterdir() if p.is_dir() and not p.name.startswith(('_', '.'))) \
        if skills.is_dir() else []


def _index(tree):
    path = Path(tree) / 'INDEX.md'
    if path.is_symlink():
        return None
    return path.read_text(errors='replace') if path.is_file() else ''


def _check_links(tree, slug, index, problems):
    folder = Path(tree) / 'skills' / slug
    if f'(skills/{slug}/SKILL.md)' not in (index or ''):
        problems.append(f'INDEX.md needs a row linking skills/{slug}/SKILL.md')
    meta = parse_frontmatter((folder / 'SKILL.md').read_text(errors='replace')) if (folder / 'SKILL.md').is_file() \
        else None
    for used in (meta or {}).get('uses') or []:
        if isinstance(used, str) and not (Path(tree) / 'skills' / used / 'SKILL.md').is_file():
            problems.append(f'skills/{slug}: uses {used!r}, which is not a skill in the library')


def validate_tree(tree, changed):
    """Problems of a library tree, for the files a session branch changed: (problems, slugs)."""
    tree = Path(tree)
    problems = []
    outside = [p for p in changed if not (p in SESSION_EDITABLE or p.startswith('skills/'))]
    template = [p for p in changed if p.startswith('skills/_')]
    if outside or template:
        problems.append(f'a session branch may only change {", ".join(SESSION_EDITABLE)} and skills/ (not '
                        f'skills/_template/); it changed {sorted(outside + template)[:10]}')
    slugs = sorted({p.split('/')[1] for p in changed
                    if p.startswith('skills/') and p.count('/') >= 2 and not p.split('/')[1].startswith('_')})
    index = _index(tree)
    if index is None:
        return problems + ['INDEX.md must be a file, not a symlink'], []
    manifest = _manifest(tree)
    for slug in slugs:
        folder = tree / 'skills' / slug
        if not folder.is_dir():
            problems.append(f'skills/{slug} was deleted; set its status to deprecated instead')
            continue
        problems += validate_skill(folder, manifest)
        _check_links(tree, slug, index, problems)
    return problems, slugs


def validate_library(tree):
    """Problems of a whole library checkout (every variant folder), for maintainers and tests."""
    tree = Path(tree)
    problems = [] if (tree / 'library.json').is_file() else ['library.json missing']
    index = _index(tree)
    manifest = _manifest(tree)
    for folder in _skill_folders(tree):
        problems += validate_skill(folder, manifest)
        _check_links(tree, folder.name, index, problems)
    return problems


# ----- merge (the harness tool) ------------------------------------------------------------------------------------
def _copy_objects(source, target):
    """Copy a git object store without symlinks and without alternates (no reads outside the clone)."""
    source = Path(source)
    for directory, names, files in os.walk(source, followlinks=False):
        here = Path(directory)
        out = Path(target) / here.relative_to(source)
        out.mkdir(parents=True, exist_ok=True)
        names[:] = [n for n in names if not (here / n).is_symlink()]
        for name in files:
            path = here / name
            if path.is_symlink() or (here.name == 'info' and name in ('alternates', 'http-alternates')):
                continue
            shutil.copyfile(path, out / name)


def merge(config, workspace, message):
    """Validate the session branch of the workspace's clone and merge it into the host library's main."""
    cfg = library_config(config.backend_options)
    if cfg is None:
        raise LibraryError('this session has no skill_library')
    if not cfg['merge']:
        raise LibraryError(f"skill_library {cfg['name']} is configured read-only (merge: false)")
    library = ensure_library(cfg)
    clone = clone_path(workspace, cfg['name'])
    branch = branch_name(config)
    if not (clone / '.git').is_dir():
        raise LibraryError(f'the workspace has no {LIBRARY_DIR}/{cfg["name"]} clone')
    lock = open(library / '.git' / 'rax-merge.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    scratch = Path(tempfile.mkdtemp(prefix='skill-merge-'))
    try:
        objects = scratch / 'agent.git'  # the agent's objects only: no config, hooks or info from its .git
        objects.mkdir()
        _copy_objects(clone / '.git' / 'objects', objects / 'objects')
        refs = clone / '.git' / 'refs' / 'heads' / Path(branch)
        packed = clone / '.git' / 'packed-refs'
        head = refs.read_text().strip() if refs.is_file() and not refs.is_symlink() else None
        if head is None and packed.is_file() and not packed.is_symlink():
            for line in packed.read_text().splitlines():
                if line.endswith(f' refs/heads/{branch}'):
                    head = line.split()[0]
        if not head or not re.fullmatch(r'[0-9a-f]{40}', head):
            raise LibraryError(f'no commit on {branch}: commit your skill first (git add, git commit)')
        (objects / 'refs' / 'heads').mkdir(parents=True)
        (objects / 'HEAD').write_text('ref: refs/heads/main\n')
        (objects / 'config').write_text('[core]\n\trepositoryformatversion = 0\n\tbare = true\n')
        _run('fetch', '-q', '--no-tags', str(objects), head, cwd=library)
        base = _run('merge-base', 'main', head, cwd=library).stdout.strip()
        changed = [p for p in _run('diff', '--name-only', base, head, cwd=library).stdout.splitlines() if p]
        if not changed:
            return {'merged': False, 'reason': 'no committed changes on the session branch since its base',
                    'branch': branch, 'head': head}
        tree = scratch / 'tree'
        tree.mkdir()
        archive = subprocess.run(['git', 'archive', '--format=tar', head], cwd=library, capture_output=True,
                                 timeout=60, env={**os.environ, **GIT_ENV})
        if archive.returncode:
            raise LibraryError('git archive failed: ' + archive.stderr.decode(errors='replace')[:300])
        subprocess.run(['tar', '-x', '-C', str(tree)], input=archive.stdout, check=True, timeout=60)
        problems, slugs = validate_tree(tree, changed)
        if problems:
            return {'merged': False, 'branch': branch, 'head': head, 'problems': problems,
                    'do': f'fix these in {LIBRARY_DIR}/{cfg["name"]}/, commit, and call skill_library_merge again'}
        before = _run('rev-parse', 'main', cwd=library).stdout.strip()
        result = _run('merge', '--no-ff', '-m', f'Merge {branch}: {message.strip()[:200]}', head, cwd=library,
                      check=False)
        if result.returncode:
            conflicts = _run('diff', '--name-only', '--diff-filter=U', cwd=library, check=False).stdout.split()
            _run('merge', '--abort', cwd=library, check=False)
            return {'merged': False, 'branch': branch, 'head': head, 'conflicts': conflicts,
                    'do': f'in {LIBRARY_DIR}/{cfg["name"]}/: git fetch origin && git merge origin/main, resolve the '
                          "conflicts (keep both sessions' evidence lines), commit, and call skill_library_merge again"}
        after = _run('rev-parse', 'main', cwd=library).stdout.strip()
        return {'merged': True, 'branch': branch, 'head': head, 'library_main': after, 'previous_main': before,
                'skills': slugs, 'changed_files': len(changed),
                'next': f'git -C {LIBRARY_DIR}/{cfg["name"]} fetch origin to see the merged main'}
    finally:
        shutil.rmtree(scratch, ignore_errors=True)
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()


def sync(name, *, repository=None, ref=None):
    """Merge the source ref's new commits into the host library's main (operator command; never pushes). Returns
    what happened and the push command that would publish the host main to the source ref."""
    library = library_path(name)
    if not (library / '.git').is_dir():
        raise LibraryError(f'no host library at {library}')
    record = json.loads((library / '.git' / 'rax-library.json').read_text())
    repository = _mooncake_repository(repository or record['source_repository'])
    ref = ref or record['source_ref']
    with open(library / '.git' / 'rax-merge.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        _run('fetch', '-q', '--no-tags', repository, ref, cwd=library, timeout=SLOW_GIT_S)
        incoming = _run('rev-parse', 'FETCH_HEAD', cwd=library).stdout.strip()
        before = _run('rev-parse', 'main', cwd=library).stdout.strip()
        if _run('merge-base', '--is-ancestor', incoming, 'main', cwd=library, check=False).returncode == 0:
            outcome = {'merged': False, 'reason': 'up to date'}
        else:
            result = _run('merge', '--no-ff', '-m', f'Merge {ref} from the source', incoming, cwd=library,
                          check=False)
            if result.returncode:
                conflicts = _run('diff', '--name-only', '--diff-filter=U', cwd=library, check=False).stdout.split()
                _run('merge', '--abort', cwd=library, check=False)
                outcome = {'merged': False, 'conflicts': conflicts,
                           'do': f'merge by hand in {library} (git merge FETCH_HEAD after fetching {ref})'}
            else:
                outcome = {'merged': True}
    return {**outcome, 'library': str(library), 'source': repository, 'ref': ref, 'incoming': incoming,
            'previous_main': before, 'main': _run('rev-parse', 'main', cwd=library).stdout.strip(),
            'publish': f'git -C {library} push {repository} main:{ref}   # not run: publishing is the operator\'s'}


def fork(source, ref, destination):
    """A library of its own whose main is `ref` of the library `source`, history kept: a session that must start
    from an earlier state (a head-to-head of variants) runs with RAX_SKILL_LIBRARY on the folder above
    `destination`; its merges stay there and the shared library is never reset."""
    source, destination = Path(source), Path(destination)
    if destination.exists():
        raise LibraryError(f'{destination} exists: a fork is made once, into a new folder')
    commit = _run('rev-parse', '--verify', f'{ref}^{{commit}}', cwd=source).stdout.strip()
    destination.parent.mkdir(parents=True, exist_ok=True)
    _run('clone', '-q', '--no-hardlinks', '--no-checkout', str(source), str(destination), timeout=SLOW_GIT_S)
    _run('checkout', '-q', '-B', 'main', commit, cwd=destination)
    _run('config', 'user.name', 'rax harness', cwd=destination)
    _run('config', 'user.email', 'harness@rax.local', cwd=destination)
    _run('remote', 'remove', 'origin', cwd=destination)
    origin = source / '.git' / 'rax-library.json'
    record = {**(json.loads(origin.read_text()) if origin.is_file() else {}), 'forked_from': str(source),
              'ref': ref, 'commit': commit,
              'skills': sorted(p.parent.name for p in (destination / 'skills').glob('*/SKILL.md')
                               if not p.parent.name.startswith('_'))}
    (destination / '.git' / 'rax-library.json').write_text(json.dumps(record, indent=2) + '\n')
    return {'library': str(destination), **record}


# ===== 3. the host-side API ========================================================================================
@dataclass
class Skill:
    """A loaded variant: its frontmatter, parameter defaults and entry functions (not yet called)."""
    name: str
    path: Path
    library: Path
    meta: dict
    params: dict                     # params.json as written (controls, value, unit, tested, measured, tune)
    module: object = field(repr=False)
    main: object = field(repr=False)
    check_preconditions: object = field(default=None, repr=False)
    success: object = field(default=None, repr=False)

    @property
    def defaults(self):
        return {name: entry.get('value') for name, entry in self.params.items()}

    @property
    def version(self):
        return self.meta.get('version')


def default_library():
    """The host API's library checkout: RAX_SKILL_LIBRARY_CHECKOUT, else the host library of DEFAULT_NAME."""
    configured = os.environ.get(CHECKOUT_ENVIRONMENT)
    return Path(configured) if configured else library_path(DEFAULT_NAME)


def list_skills(library=None):
    """[{name, primitive, representation, hands, status, version, description}] of a library checkout."""
    library = Path(library) if library is not None else default_library()
    out = []
    for folder in _skill_folders(library):
        meta = parse_frontmatter((folder / 'SKILL.md').read_text(errors='replace')) \
            if (folder / 'SKILL.md').is_file() else None
        if meta:
            out.append({k: meta.get(k) for k in ('name', 'primitive', 'representation', 'hands', 'status',
                                                  'version', 'description')})
    return out


def resolve_name(name, library=None):
    """The variant folder a name refers to, in a library checkout. Accepts the folder name
    (`in-hand-rotate--rl`), `<primitive>/<variant>` where the variant is the folder name or its part after `--`
    (`in-hand-rotate/rl`; services/dex_eval tries this form), and a bare primitive when it has exactly one
    variant. Raises SkillNotFound (a LookupError) otherwise."""
    library = Path(library) if library is not None else default_library()
    folders = {p.name for p in _skill_folders(library) if (p / 'SKILL.md').is_file()}
    name = str(name).strip()
    if name in folders:
        return name
    if name.count('/') == 1:
        primitive, variant = name.split('/')
        for candidate in (variant, f'{primitive}--{variant}'):
            if candidate in folders and candidate.startswith(f'{primitive}--'):
                return candidate
    siblings = sorted(f for f in folders if f.startswith(f'{name}--'))
    if len(siblings) == 1:
        return siblings[0]
    detail = f'; variants of {name}: {siblings}' if siblings else ''
    raise SkillNotFound(f'no skill {name!r} in {library}{detail}')


def load_skill_main(name, library_root=None):
    """The skill's entry `main(ctx, **params)` as a callable (services/dex_eval's loader). `name` as resolve_name
    accepts; `library_root`: a library checkout (default: default_library()). Unknown names raise SkillNotFound,
    a LookupError. The callable is the skill's own main: it checks its preconditions itself and returns
    {ok, measured: {status, ...}, why}; use run_skill for the optional check_preconditions/success wrapping."""
    return load_skill(name, library_root).main


def load_skill(name, library=None):
    """Import `skills/<name>/code/run.py` of a library checkout (default: default_library()) without running it.

    The skill's run.py puts its own code/ and the library's lib/ on sys.path, so its imports resolve in this
    process. Python keeps one module per name: load skills from ONE library checkout per process (a second
    checkout's same-named packages would resolve to the first's). Raises LibraryError for an unknown or
    malformed skill."""
    library = Path(library) if library is not None else default_library()
    name = resolve_name(name, library)
    folder = library / 'skills' / name
    meta = parse_frontmatter((folder / 'SKILL.md').read_text(errors='replace'))
    if not meta or not isinstance(meta.get('entry'), str):
        raise LibraryError(f'skills/{name}/SKILL.md has no frontmatter entry')
    entry = (folder / meta['entry']).resolve()
    if not entry.is_relative_to(folder.resolve()) or not entry.is_file():
        raise LibraryError(f'skills/{name}: entry {meta["entry"]} is not a file in the skill')
    params_file = folder / 'code' / name.replace('-', '_') / 'params.json'
    params = json.loads(params_file.read_text()) if params_file.is_file() else {}
    module_name = f'rax_skill_{name.replace("-", "_")}'
    spec = importlib.util.spec_from_file_location(module_name, entry)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    main = getattr(module, 'main', None)
    if not callable(main):
        raise LibraryError(f'skills/{name}: {meta["entry"]} defines no main(ctx, ...)')
    return Skill(name=name, path=folder, library=library, meta=meta, params=params, module=module, main=main,
                 check_preconditions=getattr(module, 'check_preconditions', None),
                 success=getattr(module, 'success', None))


def _contract(value):
    """Any entry return value coerced to {ok, measured: {status, ...}, why} (mirrors dexlib.contract.normalize)."""
    if not isinstance(value, dict) or 'ok' not in value:
        return {'ok': False, 'measured': {'status': 'aborted'},
                'why': f'entry returned {type(value).__name__}, not the {{ok, measured, why}} contract'}
    measured = dict(value.get('measured') or {})
    status = measured.get('status')
    if status not in RESULT_STATUSES:
        status = 'success' if value['ok'] else 'failed'
    if bool(value['ok']) and status != 'success':
        value = {**value, 'ok': False}
    if not value['ok'] and status == 'success':
        status = 'failed'
    return {**value, 'ok': bool(value['ok']), 'measured': {**measured, 'status': status},
            'why': str(value.get('why', ''))}


def run_skill(ctx, name, params=None, *, library=None, check=True, judge=True, progress=None, skill=None):
    """Load (or take `skill`) and run one variant with `ctx`; always returns the entry contract.

    1. `check_preconditions(ctx, **params)` when the skill has one and `check`: a refusal returns status
       failed_precondition without calling main.
    2. `main(ctx, **params)`, with `progress=` when main names it; an exception becomes status aborted.
    3. `success(ctx, measured)` when the skill has one and `judge` and main reported success: False demotes the
       result to status failed.
    Adds `skill` (name, version, primitive, representation, library, commit) and `duration_s`.
    """
    skill = skill or load_skill(name, library)
    params = dict(params or {})
    started = time.monotonic()
    code_paths = [f'skills/{skill.name}', 'lib', *(f'skills/{u}' for u in skill.meta.get('uses') or [])]
    info = {'name': skill.name, 'version': skill.version, 'primitive': skill.meta.get('primitive'),
            'representation': skill.meta.get('representation'), 'library': str(skill.library),
            'commit': _library_commit(skill.library, code_paths)}

    def finish(result):
        return {**result, 'skill': info, 'duration_s': round(time.monotonic() - started, 3)}
    if check and skill.check_preconditions is not None:
        try:
            pre = skill.check_preconditions(ctx, **params)
        except Exception as exc:  # noqa: BLE001 -- a refusal of the check itself is a precondition failure
            pre = {'ok': False, 'why': f'check_preconditions raised {type(exc).__name__}: {exc}', 'failed': []}
        if not (isinstance(pre, dict) and pre.get('ok')):
            pre = pre if isinstance(pre, dict) else {'ok': False, 'why': repr(pre)}
            return finish({'ok': False, 'measured': {'status': 'failed_precondition', 'preconditions': pre},
                           'why': str(pre.get('why', 'preconditions do not hold'))})
    arguments = dict(params)
    if progress is not None and 'progress' in inspect.signature(skill.main).parameters:
        arguments['progress'] = progress
    try:
        result = _contract(skill.main(ctx, **arguments))
    except KeyboardInterrupt:
        raise
    except Exception as exc:  # noqa: BLE001 -- the trial runner gets a typed result, not a traceback
        result = {'ok': False, 'measured': {'status': 'aborted', 'exception': f'{type(exc).__name__}: {exc}'},
                  'why': f'main raised {type(exc).__name__}: {exc}'}
    if judge and skill.success is not None and result['ok']:
        try:
            verdict = bool(skill.success(ctx, result['measured']))
        except Exception as exc:  # noqa: BLE001
            verdict, result['success_error'] = False, f'{type(exc).__name__}: {exc}'
        result['success_check'] = verdict
        if not verdict:
            result = {**result, 'ok': False, 'measured': {**result['measured'], 'status': 'failed'},
                      'why': f"{result['why']} (the skill's success check did not hold)"}
    return finish(result)


def _library_commit(library, paths=()):
    """HEAD of a library checkout, with '-dirty' when `paths` (the skill, its uses, lib/) have uncommitted changes:
    evidence must name the code that actually ran."""
    def git(*args):
        return subprocess.run(['git', '-C', str(library), *args], capture_output=True, text=True, timeout=30,
                              env={**os.environ, **GIT_ENV})
    head = git('rev-parse', 'HEAD')
    if head.returncode:
        return None
    changes = git('status', '--porcelain', '--', *paths).stdout.splitlines() if paths else []
    dirty = [line for line in changes if '__pycache__' not in line and not line.endswith('.pyc')]
    return head.stdout.strip() + ('-dirty' if dirty else '')


def run_record(result, *, trial, backend, params=None, session, unattended, conditions, smoothness=None,
               date=None, **extra):
    """An evidence/runs.jsonl line for a run_skill result (validated keys; pass to append_run)."""
    if backend not in BACKENDS:
        raise ValueError(f'backend must be one of {BACKENDS}')
    status = result['measured']['status']
    return {'date': date or time.strftime('%Y-%m-%d'), 'session': str(session), 'commit': result['skill']['commit'],
            'version': result['skill']['version'], 'backend': backend, 'trial': str(trial),
            'params': dict(params or {}), 'success': status == 'success', 'status': status,
            'unattended': bool(unattended), 'conditions': str(conditions), 'smoothness': smoothness, **extra}


def append_run(name, record, *, library=None):
    """Append one evidence line to skills/<name>/evidence/runs.jsonl of a library checkout (not committed)."""
    library = Path(library) if library is not None else default_library()
    folder = library / 'skills' / resolve_name(name, library)
    missing = [k for k in RUN_KEYS if k not in record]
    if missing:
        raise ValueError(f'evidence record lacks {missing}')
    path = folder / 'evidence' / 'runs.jsonl'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as handle:
        handle.write(json.dumps(record, default=str) + '\n')
    return path


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Skill libraries (services/skill_library.py)')
    commands = parser.add_subparsers(dest='command', required=True)
    command = commands.add_parser('validate', help='check every variant of a library checkout')
    command.add_argument('library', type=Path)
    command = commands.add_parser('list', help='the variants of a library checkout')
    command.add_argument('library', type=Path, nargs='?')
    command = commands.add_parser('sync', help='merge the source ref into the host library main (never pushes)')
    command.add_argument('name')
    command.add_argument('--repository')
    command.add_argument('--ref')
    command = commands.add_parser('fork', help='a library of its own whose main is REF of the library FROM')
    command.add_argument('--from', dest='source', type=Path, required=True)
    command.add_argument('--ref', required=True)
    command.add_argument('--to', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == 'validate':
        problems = validate_library(args.library)
        print(json.dumps({'library': str(args.library), 'problems': problems}, indent=2))
        return 1 if problems else 0
    if args.command == 'list':
        print(json.dumps(list_skills(args.library), indent=2))
    elif args.command == 'sync':
        print(json.dumps(sync(args.name, repository=args.repository, ref=args.ref), indent=2))
    else:
        print(json.dumps(fork(args.source, args.ref, args.to), indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
