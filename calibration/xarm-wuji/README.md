# xArm + Wuji camera calibration

Camera extrinsics were refitted, installed and verified on 2026-10-29 PDT
(2026-10-30 UTC). The wrist optical calibration from the earlier run that day
was retained. The board is 10×13 ChArUco, with 20 mm squares, 15 mm markers and
DICT_4X4_100. `board.json` preserves its legacy 14-row ID layout with the last
physical row absent.

`examples/xarm-wuji.json` and the host runtime config reference these files.
Runtime RGB-D is 640×480 at 6 fps; calibration RGB captures use 1280×720 at 6 fps.

| Camera | Serial | Current calibration |
| --- | --- | --- |
| Wrist D435 | 262422074331 | New camera-to-TCP/flange extrinsics; retained fitted RGB intrinsics, stereo alignment and depth correction |
| Fixed side D435 | 348522070593 | New camera-to-base extrinsics; factory RGB/depth intrinsics retained |

Both intrinsics files are unchanged and retain factory device/profile guards.
Wrist RGB is rectified with the fitted Brown5 model. Native depth receives the
software factor **1.0048860716612322** before projection through the fitted stereo
transform and nearest-surface Z buffer. Output depth is color-camera Z. Factory
depth units remain 0.0010000000474974513 m; firmware settings are unchanged.

## Current validation

All 18 capture poses were accepted: 15 training and three reserved poses. The
hand held its measured posture. Three fresh candidate observations and three
fresh installed observations passed through `xarm_observe` and
`xarm_pixel_to_position` using the corresponding calibration files.

| Check | Result |
| --- | --- |
| Wrist hand-eye maximum training residual | 1.52 mm / 0.63° |
| Wrist hand-eye maximum reserved residual | 1.44 mm / 0.38° |
| Side-camera maximum stationary-board consistency residual | 3.37 mm / 0.41° |
| Installed wrist board-point median range / worst 95th percentile | 1.39–1.45 / 3.47 mm |
| Installed side board-point median range / worst 95th percentile | 1.43–2.42 / 6.33 mm |
| Installed cross-camera median range / worst 95th percentile | 2.60–2.81 / 5.36 mm |

Installed checks used 15–19 common corners per frame.
Gates remain: hand-eye ≤5 mm / 1°; RGB-D median ≤5 mm and 95th percentile ≤10 mm;
cross-camera 95th percentile ≤10 mm; at least 12 valid points per view and
≤0.2 mm arm translation during each capture. The 18 camera commissioning tests passed.

The first fresh verification had only 4–8 decoded side corners and failed the
point-count gate. Small-marker detection now tries larger decoding images and
threshold windows when needed, choosing by decoded corner count. Every result
is mapped back to native pixel centers before geometry calculations. Acceptance
limits were unchanged, and new fresh observations passed afterward.

The arm returned to [0.2531, -0.00134, 0.31412] m. Final readback showed state 4,
zero arm error, disarmed motion, and fresh disabled hand feedback without faults.

## Retained optical calibration

The earlier native collection supplied 15 training and three reserved RGB poses.
Wrist RGB reprojection RMS was 0.227 px. Twelve views supported IR/RGB stereo
fitting with RMS 0.168 px; 255 training measurements supplied the depth factor.
All 18 saved RGB-D views passed the original limits. These are historical
optical-calibration measurements, not a new fit in this extrinsics-only run.
The complete previous report is preserved in this run's backup.

## Coordinate conventions and scope

Matrices use column vectors and metres; `T_A_B` maps B coordinates into A:

- `wrist_T_cam_tcp.yaml`: TCP → wrist color camera.
- `wrist_T_cam_flange.yaml`: flange → wrist color camera.
- `side_T_cam_base.yaml`: arm base → side color camera.
- `calibrated_depth_to_color.T_color_depth`: corrected native depth → wrist color camera.

Wrist: `T_base_cam = T_base_tcp @ inverse(T_cam_tcp)`.
Side: `T_base_cam = inverse(T_cam_base)`.

TCP readback was [0, 0, 172, 0, 0, 0] mm/rad; payload was 0.82 kg with CoG
[0, 0, 48] mm. These are controller settings, not measured palm geometry.
Recompute the TCP-relative transform if TCP settings change and recalibrate
after moving either mount. Accuracy is qualified around the observed tabletop
and calibration poses: ±25 mm sideways, 0–50 mm upward and rotations up to 21.2°.
Full-workspace accuracy and manipulation remain unqualified.

## Evidence and repeatability

Current evidence: `/home/jack-xiang/rax-runtime/camera-recalibration-20261030T041535Z/`

- `collection/`: raw images, poses, fit and reserved-pose residuals.
- `verification-candidate-multiscale/checks/`: fresh candidate RGB-D checks.
- `verification-installed/checks/`: fresh checks through the active config.
- `backup/calibration/`: complete previous calibration and optical-fit report.
- `installed-manifest.json`: active extrinsics and retained-intrinsics hashes.

The earlier RGB/stereo/depth evidence remains under
`/home/jack-xiang/rax-runtime/camera-recalibration-20261030T003223Z/`.

Stop camera viewers/frontends before collection; each owns the devices. Load
`/home/jack-xiang/rax-runtime/environment.sh` and pass `--backend-options
"$RAX_BACKEND_OPTIONS"`. Keep the board flat and fixed throughout.

1. Run `scripts/xarm-charuco-calibrate --preflight --use-installed-intrinsics
   --root NEW_PREFLIGHT_ROOT`.
2. After operator clearance and arm readiness, collect with
   `--use-installed-intrinsics --yes --prepare-hand --root NEW_COLLECTION_ROOT
   --install --calibration-dir CANDIDATE_DIR`. These flags authorize real motion.
   The hand holds its posture; no finger homing occurs.
3. Start a disarmed RGB-D MCP frontend using candidate calibration paths. Run
   `scripts/xarm-charuco-verify --socket SOCKET --session SESSION --fit
   COLLECTION/result.json --output NEW_CHECK_DIRECTORY`.
4. Install only after fresh checks pass, preserving a backup, then repeat the
   same verification through the active runtime config.

For an optical refit, collect with `--capture-native`, then use
`scripts/xarm-charuco-fit` and `scripts/xarm-charuco-stereo-fit`. The latter's
`--fit-depth-scale` uses training measurements only. Fresh MCP checks still
apply; image-fit residuals alone do not establish metric depth accuracy.
