# xArm + Wuji bring-up checklist

Started 2026-10-27 on `real/xarmwuji`. Checkmarks describe observed results;
unchecked items still need commissioning. The earlier
[live MCP report](wuji-live-test-2026-10-25.md) covers initial hand-only tests.

**Camera calibration is installed and verified.** The new accurate print was
used for run06 RGB calibration and run07 infrared/RGB stereo alignment. See the
[calibration report](../calibration/xarm-wuji/README.md) for measurements, files,
coordinate conventions and repeat commands. Final installed MCP median errors
were 3.60 mm (wrist), 1.63 mm (side), and 4.34 mm between cameras. All 102
targeted regression tests passed. The arm returned to start and is disarmed;
the hand is disabled with fresh, fault-free feedback.

## Configuration and hardware

- [x] Python environment: `.venv/bin/python`, xArm SDK 1.18.4, Wuji SDK
  2026.10.12, RealSense 2.59.1.11053, OpenCV 5.0.0.
- [x] Read arm identity/state at `192.168.100.16`; controller firmware 2.7.1.
- [x] Configure right Wuji `WH2KA01260823017`, endpoint `192.168.1.111:7447`.
- [x] Identify wrist D435 `262422074331` and fixed side D435 `348522070593`;
  both report firmware 5.15.1.55 and USB 2.1. Runtime is 640×480 at 6 fps;
  calibration RGB capture is 1280×720 at 6 fps.
- [x] Select `wrist`/`side` camera names in generated workspace instructions.
- [ ] Verify the mounted hand's mass, center of gravity, contact reference and
  collision clearance. Readback was 0.82 kg, CoG [0, 0, 48] mm, TCP offset
  [0, 0, 172, 0, 0, 0] mm/rad. These are existing controller settings, **not
  measured Wuji geometry**. Calibration uses the recorded TCP reference.
- [ ] Review workspace limits and finger sweep for actual tasks.
- [ ] Qualify gains/current, all joint directions and full-hand postures. The
  example retains 0.5 A, kp 1.5, kd 0.1 and an empty posture map.
- [ ] Define and test a named episode-reset posture. `prepare` holds measured
  angles; it does not home. `scripts/xarm home` resets the arm only. A hand
  reset can use `wuji_pose` after a measured posture and its path are qualified.

## Camera calibration

- [x] Record each camera's factory color intrinsics for both used resolutions,
  distortion model/coefficients, native depth intrinsics, depth units and
  depth-to-color transform in `calibration/xarm-wuji/*_intrinsics.json`.
- [x] Check configured serial/profile/intrinsics against live camera readback.
  Depth is aligned to color and uses the color pixel geometry.
- [x] Confirm the new accurately printed board dimensions with operator: 10×13 squares,
  20 mm square, 15 mm marker. Images match DICT_4X4_100, the legacy 10×14
  layout with its last row removed. `board.json` preserves those marker IDs
  and coordinates and excludes missing-row corners; generating a fresh odd-row
  board produces a different pattern.
- [x] Camera-only detection and read-only arm/target preflight. Initial HD
  wrist/side reprojection means in the new-board preflight were 0.453/0.344 px
  with 108/61 corners.
- [x] Collect 18 arm poses while keeping the board and mounts fixed, holding
  the hand at its measured posture. Planned offsets: XY within ±25 mm,
  Z 0–50 mm, orientation change at most 21.2°, speed 15 mm/s.
- [x] Fit wrist-to-TCP and board-to-base using training poses; validate all
  three reserved poses. Require ≤5 mm and ≤1° residuals. Estimate the fixed
  side transform through the same stationary board and validate consistency.
- [x] Install `wrist_T_cam_tcp.yaml`, `wrist_T_cam_flange.yaml`, and
  `side_T_cam_base.yaml` only after those checks pass. Config already points
  to the installed files; normal observation loads them successfully.
- [x] Independently check RGB-D metric accuracy and cross-camera base-frame
  points. Factory depth scale readback alone does not establish metric accuracy.
- [x] Verify normal 640×480 RAX observations, streaming and pixel-to-base
  conversion through the MCP frontend.

## Network and operator tools

The hand's Ethernet route must select `enx6c1ff7618956`, source
`192.168.1.11`. Discovery alone can succeed while the control connection uses
an incorrect route. Verify both destinations after adapter reconnection:

```bash
ip route get 192.168.1.111
ip route get 192.168.100.16
```

The confirmed local topology currently uses this temporary NetworkManager route:

```bash
nmcli device modify enx6c1ff7618956 \
  +ipv4.routes '192.168.1.111/32 0.0.0.0 50 src=192.168.1.11'
```

It has not been persisted in the connection profile. Existing arm routes and
Wi-Fi settings are unchanged. The camera-only viewer is at
<http://localhost:8765/>; UFactory Studio is at
<http://192.168.100.16:18333/>. Stop the viewer before starting a backend camera
session because both use the same devices.

Calibration is complete. Final evidence and repeat-collection instructions are
in the [calibration report](../calibration/xarm-wuji/README.md). Rejected runs,
raw captures and the commissioning audit remain under `.local-runtime/`.

## Real backend MCP qualification

- [x] Historical 2026-10-25 hand-only MCP discovery, status, invalid-input
  rejection, bounded index-joint moves, named recorded posture, holding and
  feedback-confirmed disable. See the linked report for failed precision trials
  and the exact scope; it does not qualify all fingers or manipulation.
- [x] Repeat connection/status and camera observations with the current setup.
- [x] Verify the bounded calibration arm motions with the hand holding and
  return to the recorded start pose, followed by disarm and confirmed hand disable.
- [ ] Qualify a repeatable episode reset, including a named hand posture and
  the intended task motion envelope.
- [ ] Exercise independent operator disarm/stop during motion and inspect both
  arm and hand outcomes. MCP requests serialize; MCP hold cannot preempt an
  active MCP motion request. Use the private operator stop interface.
- [ ] Qualify communication loss, process loss and physical stop behavior with
  an appropriate controlled test setup.
- [ ] Qualify grasp, retention, transport, release and repeated episodes with
  the intended objects. No tactile feedback or full-hand collision model has
  been established by the current integration.
