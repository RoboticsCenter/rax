"""Vendor model assets for the simulation twin, pinned and fetched as data.

Two upstream description packages supply the geometry. Nothing in them is ever
executed: the fetch is a sparse `git checkout` of fixed files at a pinned commit,
the repository metadata is then deleted, and the models are read as XML, YAML
and meshes only.

- Wuji Hand 2.2: `wuji-technology/wuji-description` (MIT), left and right MJCF
  plus their STL meshes.
- xArm 6: `xArm-Developer/xarm_ros2` `xarm_description` (BSD-3-Clause), the
  xArm 6 kinematics and inertial YAML plus the model-1305 meshes. MuJoCo
  Menagerie has no xArm 6 (only `ufactory_xarm7` and `ufactory_lite6`), so the
  arm's MJCF is generated from these data files (`model.py`).

    python -m services.sim_xarm_wuji.assets fetch     # once per checkout
    python -m services.sim_xarm_wuji.assets verify

The root is `$RAX_SIM_ASSETS`, else `<checkout>/runtime/sim-assets` (machine-local, not
tracked), laid out as mooncake's sim2real lines and services/dex_eval/fingertip.py expect:
`robots/wuji/wuji-description-<commit8>`, `robots/ufactory/xarm_ros2-<commit8>`.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

PROJECT = Path(__file__).resolve().parents[2]
ENVIRONMENT = 'RAX_SIM_ASSETS'

SOURCES = {
    'wuji-description': {
        'vendor': 'wuji',
        'url': 'https://github.com/wuji-technology/wuji-description',
        'commit': '0f02495611a5e5f71ebac50da7efd4f646cc0a75',
        'release': 'v2026.11.9 (committed 2026-11-10)',
        'license': 'MIT',
        'paths': ['/LICENSE', '/README.md',
                  '/hand2/hand2_2/body/mjcf/left.xml', '/hand2/hand2_2/body/mjcf/right.xml',
                  '/hand2/hand2_2/body/meshes/left/*.STL', '/hand2/hand2_2/body/meshes/right/*.STL'],
    },
    'xarm_ros2': {
        'vendor': 'ufactory',
        'url': 'https://github.com/xArm-Developer/xarm_ros2',
        'commit': 'ac736c92b02de7ca7e0be6f737d77d46707464d8',
        'release': 'branch humble (committed 2026-10-20)',
        'license': 'BSD-3-Clause',
        'paths': ['/LICENSE', '/xarm_description/package.xml',
                  '/xarm_description/config/kinematics/default/xarm6_default_kinematics.yaml',
                  '/xarm_description/config/link_inertial/xarm6_type6_HT_BR2.yaml',
                  '/xarm_description/config/xarm6/joint_limits_1305.yaml',
                  '/xarm_description/urdf/xarm6/xarm6.urdf.xacro',
                  '/xarm_description/meshes/xarm6_1305/visual/*.stl',
                  '/xarm_description/meshes/xarm6_1305/collision/*.obj'],
    },
}


class AssetsMissing(FileNotFoundError):
    pass


def root():
    configured = os.environ.get(ENVIRONMENT)
    return Path(configured).expanduser().resolve() if configured else PROJECT/'runtime'/'sim-assets'


def folder(name):
    source = SOURCES[name]
    return root()/'robots'/source['vendor']/f"{name}-{source['commit'][:8]}"


def wuji_hand_mjcf(side):
    return folder('wuji-description')/'hand2'/'hand2_2'/'body'/'mjcf'/f'{side}.xml'


def xarm_description():
    return folder('xarm_ros2')/'xarm_description'


def require():
    """Raise a clear error naming the fetch command when an asset is absent."""
    missing = [name for name in SOURCES if not (folder(name)/'SOURCE.json').is_file()]
    if missing:
        raise AssetsMissing(f'simulation assets {missing} are missing under {root()}: '
                            'run `python -m services.sim_xarm_wuji.assets fetch`')
    return root()


def available():
    try:
        require()
        return True
    except AssetsMissing:
        return False


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _git(*args, cwd=None):
    env = {**os.environ, 'GIT_TERMINAL_PROMPT': '0', 'GIT_LFS_SKIP_SMUDGE': '1'}
    # No hooks, no filters from the remote: the checkout only writes files.
    return subprocess.run(['git', '-c', 'core.hooksPath=/dev/null', '-c', 'protocol.file.allow=never', *args],
                          cwd=cwd, env=env, check=True, text=True, capture_output=True).stdout.strip()


def fetch(name, *, force=False):
    source = SOURCES[name]
    dest = folder(name)
    if dest.exists() and not force:
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f'.fetch-{name}-', dir=dest.parent) as tmp:
        checkout = Path(tmp)/'repo'
        _git('clone', '-q', '--filter=blob:none', '--no-checkout', source['url'], str(checkout))
        _git('sparse-checkout', 'set', '--no-cone', *source['paths'], cwd=checkout)
        _git('checkout', '-q', source['commit'], cwd=checkout)
        date = _git('log', '-1', '--format=%cI', cwd=checkout)
        if _git('rev-parse', 'HEAD', cwd=checkout) != source['commit']:
            raise RuntimeError(f'{name}: checkout is not the pinned commit')
        shutil.rmtree(checkout/'.git')
        files = sorted(p for p in checkout.rglob('*') if p.is_file())
        record = {'name': name, 'url': source['url'], 'commit': source['commit'], 'commit_date': date,
                  'release': source['release'], 'license': source['license'], 'paths': source['paths'],
                  'fetched_unix': time.time(),
                  'files': {str(p.relative_to(checkout)): _digest(p) for p in files}}
        (checkout/'SOURCE.json').write_text(json.dumps(record, indent=1) + '\n')
        if dest.exists():
            shutil.rmtree(dest)
        os.replace(checkout, dest)
    return dest


def verify():
    """Recompute each recorded file digest; returns {name: problems}."""
    report = {}
    for name in SOURCES:
        path = folder(name)/'SOURCE.json'
        if not path.is_file():
            report[name] = ['missing']
            continue
        record = json.loads(path.read_text())
        problems = [] if record.get('commit') == SOURCES[name]['commit'] else ['commit differs from the pin']
        for relative, digest in record['files'].items():
            file = folder(name)/relative
            if not file.is_file() or _digest(file) != digest:
                problems.append(f'{relative} changed or missing')
        report[name] = problems
    return report


def provenance():
    """What docs and logs cite: url, commit, date and license of each asset."""
    out = {}
    for name, source in SOURCES.items():
        path = folder(name)/'SOURCE.json'
        record = json.loads(path.read_text()) if path.is_file() else {}
        out[name] = {'url': source['url'], 'commit': source['commit'], 'release': source['release'],
                     'license': source['license'], 'commit_date': record.get('commit_date'),
                     'present': path.is_file()}
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('fetch', 'verify', 'show'))
    parser.add_argument('--force', action='store_true', help='re-fetch even if present')
    args = parser.parse_args(argv)
    if args.action == 'fetch':
        for name in SOURCES:
            print(f'{name}: {fetch(name, force=args.force)}', flush=True)
    if args.action in ('fetch', 'verify'):
        report = verify()
        print(json.dumps(report, indent=1))
        return 1 if any(report.values()) else 0
    print(json.dumps(provenance(), indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
