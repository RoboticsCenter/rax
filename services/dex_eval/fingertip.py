"""Optional fingertip-speed SPARC through Wuji Hand 2 forward kinematics.

Needs the ``mujoco`` package and the official wuji-description model
(https://github.com/wuji-technology/wuji-description, hand2/hand2_2/body/mjcf/{left,right}.xml).
When either is missing every function returns {'available': False, 'reason': ...}.

Joint mapping: the SDK's flat order thumb/index/middle/ring/pinky x S1..S4 matches the
MJCF's per-finger order (thumb cmc_flex, cmc_abd, mcp, ip; fingers mcp_flex, mcp_abd,
pip, dip) with an l_/r_ side prefix. Fingertip sites are the MJCF sites ending in "_tip".
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np

MJCF = Path('hand2/hand2_2/body/mjcf')
FINGERS = ('thumb', 'index_finger', 'middle_finger', 'ring_finger', 'pinky')
MJCF_JOINTS = ('thumb_cmc_flex', 'thumb_cmc_abd', 'thumb_mcp', 'thumb_ip') + tuple(
    f'{finger}_{joint}' for finger in ('index_finger', 'middle_finger', 'ring_finger', 'pinky')
    for joint in ('mcp_flex', 'mcp_abd', 'pip', 'dip'))
ENVIRONMENT = 'RAX_WUJI_DESCRIPTION'
_MODELS = {}


def candidate_roots():
    """Places searched for a wuji-description checkout, most specific first."""
    roots = []
    if os.environ.get(ENVIRONMENT):
        roots.append(Path(os.environ[ENVIRONMENT]))
    try:
        from services.storage_root import PROJECT, artifact_root
        for base in (artifact_root(), PROJECT / 'runtime'):
            roots += sorted((base / 'sim-assets/robots/wuji').glob('wuji-description*'))
    except Exception:
        pass
    return roots


def find_model(side, root=None):
    """Path of the side's MJCF, or None."""
    for base in ([Path(root)] if root else candidate_roots()):
        path = base / MJCF / f'{side}.xml'
        if path.is_file():
            return path
    return None


def _model(path):
    import mujoco
    key = str(path)
    if key not in _MODELS:
        model = mujoco.MjModel.from_xml_path(key)
        prefix = 'l_' if path.stem.startswith('left') else 'r_'
        qpos = []
        for name in MJCF_JOINTS:
            ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefix + name)
            if ident < 0:
                raise ValueError(f'{path}: joint {prefix + name} not found')
            qpos.append(int(model.jnt_qposadr[ident]))
        sites = {}
        for finger in FINGERS:
            ident = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f'{prefix}{finger}_tip')
            if ident >= 0:
                sites[finger] = ident
        if not sites:
            raise ValueError(f'{path}: no fingertip sites')
        _MODELS[key] = (model, np.asarray(qpos), sites)
    return _MODELS[key]


def fingertip_positions(q, side, root=None):
    """(T, n_tips, 3) fingertip positions in the hand frame for joint trajectories q (T, 20)."""
    import mujoco
    path = find_model(side, root)
    if path is None:
        raise FileNotFoundError(f'wuji-description model for the {side} hand not found '
                                f'(set {ENVIRONMENT} to the checkout)')
    model, qpos, sites = _model(path)
    data = mujoco.MjData(model)
    ids = list(sites.values())
    out = np.empty((q.shape[0], len(ids), 3))
    for i, row in enumerate(np.asarray(q, dtype=float)):
        data.qpos[qpos] = row
        mujoco.mj_kinematics(model, data)
        out[i] = data.site_xpos[ids]
    return out, list(sites)


def fingertip_sparc(q_ref, q_meas, fs, side, cfg, *, window=None, root=None, max_rate_hz=200.0):
    """SPARC of each fingertip's speed, measured and reference, on uniform (T, 20) trajectories.

    NaN (disabled) measured joints take their reference value. Returns per-finger values
    and their means, or {'available': False, 'reason'} when FK is unavailable.
    """
    try:
        import mujoco  # noqa: F401
    except Exception as exc:
        return {'available': False, 'reason': f'mujoco not importable ({type(exc).__name__})'}
    if find_model(side, root) is None:
        return {'available': False, 'reason': f'wuji-description not found (set {ENVIRONMENT})'}
    from .metrics import sparc
    from .signals import derivative, lowpass
    q_ref = np.asarray(q_ref, dtype=float)
    q_meas = np.where(np.isfinite(q_meas), q_meas, q_ref)
    q_ref = np.nan_to_num(q_ref)
    q_meas = np.nan_to_num(q_meas)
    if window is not None:
        q_ref, q_meas = q_ref[window[0]:window[1]], q_meas[window[0]:window[1]]
    step = max(1, int(round(fs / max_rate_hz)))
    fs_fk = fs / step
    result = {'available': True, 'fs': fs_fk, 'per_finger': {}}
    try:
        speeds = {}
        for label, q in (('meas', q_meas), ('ref', q_ref)):
            q = lowpass(q, fs, cfg.smooth_cutoff_hz)[::step]
            tips, names = fingertip_positions(q, side, root)
            speeds[label] = (np.linalg.norm(derivative(tips, fs_fk), axis=2), names)
    except Exception as exc:
        return {'available': False, 'reason': f'{type(exc).__name__}: {exc}'}
    kwargs = dict(fc=cfg.sparc_fc_hz, amp_th=cfg.sparc_amp_th, padlevel=cfg.sparc_padlevel)
    for label, (speed, names) in speeds.items():
        for k, name in enumerate(names):
            moving = np.max(speed[:, k]) > 1e-3  # m/s
            value = sparc(speed[:, k], fs_fk, **kwargs) if moving else float('nan')
            result['per_finger'].setdefault(name, {})[label] = value
    for label in ('meas', 'ref'):
        values = [v[label] for v in result['per_finger'].values() if np.isfinite(v.get(label, np.nan))]
        result[f'sparc_{label}'] = float(np.mean(values)) if values else float('nan')
    return result
