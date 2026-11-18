"""Simulation session options: completion from the example rig and safety validation.

A simulated session's backend options are the bimanual rig's (arms, cameras,
calibration manifest) plus a `simulation` block (spec.SIM_DEFAULTS). Two ways to select
the simulator (services/backends.py):
- robot `xarm_wuji_bimanual_sim`: options may be partial or empty; missing rig options
  come from examples/xarm-wuji-bimanual-sim.json (the evaluation harness's route);
- robot `xarm_bimanual` with a `simulation` block: the agent sees exactly the real
  robot's workspace, skills and tools (agent sessions on the twin).

Safety: a simulated rig must not name real hardware, so no real-robot code path that
reads the same options can reach a device. Arm addresses are `sim-*` names (not IP
addresses), hands connect by `SIM*` serials without an address, camera serials are
`sim-*`.
"""
from __future__ import annotations

from copy import deepcopy
import ipaddress
import json
from pathlib import Path

from .scenes import SCENES
from .spec import SIDES, options as sim_options

PROJECT = Path(__file__).resolve().parents[2]
EXAMPLE = PROJECT/'examples'/'xarm-wuji-bimanual-sim.json'
SIM_KEY = 'xarm_wuji_bimanual_sim'


def complete(backend_options):
    """The example rig's options under any given ones; `simulation` merged key by key."""
    options = deepcopy(backend_options or {})
    base = json.loads(EXAMPLE.read_text())
    simulation = {**base.get('simulation', {}), **(options.pop('simulation', None) or {})}
    if 'arms' not in options:
        options = {**base, **options}
    options['simulation'] = simulation
    return options


def _is_ip(value):
    try:
        ipaddress.ip_address(str(value).split(':')[0])
        return True
    except ValueError:
        return False


def validate_simulation(options):
    """Refuse simulated options that could address real hardware; check the sim block."""
    if not isinstance(options.get('simulation'), dict):
        raise ValueError('simulation must be an object')
    sim = sim_options(options['simulation'])
    if sim['scene'] not in SCENES:
        raise ValueError(f"simulation.scene must be one of {', '.join(SCENES)}")
    for key in ('realtime_factor',):
        if not isinstance(sim[key], (int, float)) or sim[key] < 0:
            raise ValueError(f'simulation.{key} must be a nonnegative number')
    for side in SIDES:
        arm = options['arms'][side]
        ip = arm.get('xarm_ip', '')
        if _is_ip(ip) or not str(ip).startswith('sim'):
            raise ValueError(f'simulated arms.{side}.xarm_ip must be a sim-* name, not a network address ({ip!r})')
        hand = arm.get('end_effector') or {}
        if hand.get('address'):
            raise ValueError(f'simulated arms.{side}.end_effector must not carry a network address')
        if not str(hand.get('serial', '')).startswith('SIM'):
            raise ValueError(f'simulated arms.{side}.end_effector.serial must start with SIM')
        for code in sim['hands'][side]['faulted_joints'].values():
            if not isinstance(code, int):
                raise ValueError('simulation.hands.<side>.faulted_joints maps joint names to integer codes')
    for camera in options.get('cameras', []):
        if not str(camera.get('serial', '')).startswith('sim'):
            raise ValueError(f"simulated camera {camera.get('name')} must use a sim-* serial")
    return sim


def validate_backend_options(backend_options):
    """For robot `xarm_wuji_bimanual_sim`: complete, then validate as the bimanual rig."""
    from services.xarm_bimanual.config import validate_options
    options = complete(backend_options)
    validate_options(options)
    return options
