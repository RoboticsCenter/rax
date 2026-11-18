"""Task outcome from a skill result and the trial meta.

Skill entry contract: ``main(ctx, **params) -> {"ok": bool, "measured": dict, "why": str}``
with ``measured["status"]`` in success | failed_precondition | dropped | timeout | aborted.
"""
from __future__ import annotations

from .trace import STATUSES


def normalize_result(value):
    """(result dict, problems) with the contract's keys; invalid shapes become an aborted result."""
    problems = []
    if not isinstance(value, dict):
        return ({'ok': False, 'measured': {'status': 'aborted'},
                 'why': f'skill returned {type(value).__name__}, not a result dict'},
                ['result is not a dict'])
    result = dict(value)
    if not isinstance(result.get('ok'), bool):
        problems.append('ok is not a bool')
        result['ok'] = bool(result.get('ok'))
    if not isinstance(result.get('measured'), dict):
        problems.append('measured is not a dict')
        result['measured'] = {}
    if not isinstance(result.get('why'), str):
        if 'why' in result:
            problems.append('why is not a string')
        result['why'] = '' if result.get('why') is None else str(result['why'])
    status = result['measured'].get('status')
    if status is None:
        problems.append('measured.status missing')
    elif status not in STATUSES:
        problems.append(f'measured.status {status!r} is not one of {STATUSES}')
    return result, problems


def outcome(result=None, meta=None):
    """{'ok', 'status', 'why', 'success', 'success_source', 'problems'}.

    success precedence: a boolean meta.success (operator or external verifier) wins;
    otherwise success = ok and measured.status == 'success' (ok alone when status is
    absent); with no result and no meta verdict success is None.
    """
    meta = meta or {}
    out = {'ok': None, 'status': None, 'why': None, 'success': None, 'success_source': None, 'problems': []}
    if result is not None:
        normalized, problems = normalize_result(result)
        status = normalized['measured'].get('status')
        out.update(ok=normalized['ok'], status=status, why=normalized['why'], problems=problems)
        if status is None:
            out['success'] = normalized['ok']
        else:
            out['success'] = bool(normalized['ok'] and status == 'success')
        out['success_source'] = 'result'
    if isinstance(meta.get('success'), bool):
        out['success'] = meta['success']
        out['success_source'] = 'meta'
    return out
