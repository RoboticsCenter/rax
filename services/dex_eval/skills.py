"""Resolve a condition's skill to a callable ``main(ctx, **params)``.

Resolution order for one condition:

1. ``entry``: an explicit target, ``package.module:function`` or ``path/to/file.py[:function]``
   (function defaults to ``main``).
2. The host-side skill library. ``services.skill_library`` is probed for a loader
   function (``LIBRARY_LOADERS``); another change adds it there. It is called with each
   candidate name in turn (``library_name``, ``skill/variant``, ``skill_variant``,
   ``variant``, ``skill``) and may return a callable, a module or object with ``main``,
   or a path to a Python file. A different loader can be given as ``module:function``.
3. Search paths: ``<dir>/<skill>/<variant>.py``, ``<dir>/<skill>/<variant>/main.py``,
   ``<dir>/<skill>_<variant>.py``, ``<dir>/<variant>.py``, ``<dir>/<variant>/main.py``,
   ``<dir>/<skill>.py``.
"""
from __future__ import annotations

import hashlib
import importlib
import importlib.util
import inspect
from pathlib import Path
import sys

LIBRARY_LOADERS = ('load_skill_main', 'load_skill', 'resolve_skill', 'get_skill', 'skill_main')


class SkillLoadError(RuntimeError):
    pass


def _import_file(path, function='main'):
    path = Path(path).resolve()
    if not path.is_file():
        raise SkillLoadError(f'skill file not found: {path}')
    name = 'dex_eval_skill_' + hashlib.sha256(str(path).encode()).hexdigest()[:12]
    module = sys.modules.get(name)
    if module is None:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.path.insert(0, str(path.parent))  # Skills may import sibling helpers.
        try:
            spec.loader.exec_module(module)
        finally:
            try:
                sys.path.remove(str(path.parent))
            except ValueError:
                pass
        sys.modules[name] = module
    return _callable(getattr(module, function, None), f'{path}:{function}')


def _callable(value, label):
    if value is None:
        raise SkillLoadError(f'{label} has no main')
    if isinstance(value, (str, Path)) and str(value).endswith('.py'):
        return _import_file(value)
    if not callable(value) or inspect.isclass(value) or inspect.ismodule(value):
        main = getattr(value, 'main', None)
        if callable(main):
            return main
        raise SkillLoadError(f'{label} is not callable and has no main')
    return value


def import_target(target):
    """Callable from 'package.module:function' or 'path/file.py[:function]'."""
    target = str(target)
    path, function = target.rsplit(':', 1) if ':' in target else (target, 'main')
    function = function or 'main'
    if path.endswith('.py') or '/' in path:
        return _import_file(path, function)
    module = importlib.import_module(path)
    return _callable(getattr(module, function, None), target)


def library_loader(spec=None):
    """The skill library's loader function, or None if it does not exist yet."""
    if spec:
        module, _, name = spec.partition(':')
        return getattr(importlib.import_module(module), name)
    try:
        module = importlib.import_module('services.skill_library')
    except Exception:
        return None
    for name in LIBRARY_LOADERS:
        loader = getattr(module, name, None)
        if callable(loader):
            return loader
    return None


def candidate_names(skill, variant=None, library_name=None):
    names = [library_name] if library_name else []
    if variant:
        names += [f'{skill}/{variant}', f'{skill}_{variant}', variant]
    names.append(skill)
    return list(dict.fromkeys(n for n in names if n))


def candidate_files(directory, skill, variant=None):
    d = Path(directory)
    files = []
    if variant:
        files += [d / skill / f'{variant}.py', d / skill / variant / 'main.py', d / f'{skill}_{variant}.py',
                  d / f'{variant}.py', d / variant / 'main.py']
    files += [d / f'{skill}.py', d / skill / 'main.py']
    return files


def resolve(skill, variant=None, *, entry=None, library_name=None, search_paths=(), loader=None):
    """(main callable, description of where it came from)."""
    if entry:
        return import_target(entry), f'entry {entry}'
    loader = library_loader() if loader is None else (loader or None)  # False: no library loader.
    errors = []
    if loader is not None:
        for name in candidate_names(skill, variant, library_name):
            try:
                value = loader(name)
            except (KeyError, LookupError, FileNotFoundError, ModuleNotFoundError, SkillLoadError) as exc:
                errors.append(f'library {name}: {type(exc).__name__}: {exc}')
                continue
            if value is None:
                continue
            return _callable(value, f'library {name}'), f'skill library {getattr(loader, "__name__", loader)}({name!r})'
    for directory in search_paths:
        for path in candidate_files(directory, skill, variant):
            if path.is_file():
                return _import_file(path), str(path)
    detail = '; '.join(errors) if errors else 'no loader in services.skill_library and no matching file'
    raise SkillLoadError(f'cannot resolve skill {skill!r} variant {variant!r}: {detail} '
                         f'(give "entry" or --skill-path)')
