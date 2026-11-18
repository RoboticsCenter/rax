"""Derive the simulated rig's calibration manifest from a verified real one.

The twin places its arms and cameras exactly where a verified RAX calibration says the
real ones are, and RAX's own calibration loader (services/xarm_bimanual/calibration.py)
then reports the same transforms in simulation as on the rig. The loader checks arm
addresses and camera serials against the manifest, so the simulated manifest is the
real one with the hardware identities replaced by simulation names (`sim-left`,
`sim-<camera>`) and its provenance recorded:

    python -m services.sim_xarm_wuji.calibration \
        calibration/xarm-wuji-bimanual-four-camera-20261117/manifest.json \
        calibration/xarm-wuji-bimanual-sim/manifest.json
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
ARM_ADDRESS = {'left': 'sim-left', 'right': 'sim-right'}


def camera_serial(name):
    return f'sim-{name}'


def derive(source, destination):
    source, destination = Path(source), Path(destination)
    raw = source.read_bytes()
    data = json.loads(raw)
    out = deepcopy(data)
    for side, address in ARM_ADDRESS.items():
        out['arm_identity'][side]['ip'] = address
    for spec in out.get('camera_specs', []):
        spec['serial'] = camera_serial(spec['name'])
    for name, model in out.get('camera_models', {}).items():
        model['serial'] = camera_serial(name)
        if isinstance(model.get('factory_profile'), dict) and 'serial' in model['factory_profile']:
            model['factory_profile']['serial'] = camera_serial(name)
    try:
        source_name = str(source.resolve().relative_to(PROJECT))
    except ValueError:
        source_name = str(source)
    out['simulation'] = {
        'derived_from': source_name, 'derived_from_sha256': hashlib.sha256(raw).hexdigest(),
        'changes': 'arm_identity.*.ip -> sim-<side>; camera serials -> sim-<camera>; geometry and intrinsics '
                   'unchanged', 'generator': 'python -m services.sim_xarm_wuji.calibration'}
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(out, indent=1) + '\n')
    return destination


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args(argv)
    print(derive(args.source, args.destination))


if __name__ == '__main__':
    main()
