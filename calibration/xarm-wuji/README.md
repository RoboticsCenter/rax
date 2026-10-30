# xArm + Wuji camera calibration

Both camera extrinsics were refitted and verified on 2026-10-29 America/Los_Angeles,
following movement of the side camera. The saved full 18-pose procedure was used.
The board is 10×13 ChArUco, 20 mm squares and 15 mm markers, DICT_4X4_100.
`board.json` preserves its legacy 14-row ID layout with the final physical row absent.

The fit accepted 15 training and 3 reserved wrist poses. Side-camera
consistency was checked across repeated observations of the same fixed board.
The hand held its measured posture. The arm returned to its starting pose.
The side fit selects native or multiscale detections by corner count, using the
existing PnP solver and pose-consistency limits; its last three frames are reserved.
A separate no-motion collection fitted the side camera's aligned-depth scale
from 12 training frames and verified it on six reserved frames before fresh MCP checks.
Small-marker detection uses denser threshold windows only when the original
fallback still has fewer than 12 corners. It selects by corner count and retains
native-pixel coordinates; point-count and geometric acceptance limits are unchanged.

| Check | Result |
| --- | --- |
| Wrist maximum training residual | 1.72 mm / 1.00° |
| Wrist maximum reserved residual | 1.54 mm / 0.53° |
| Side maximum board consistency residual | 1.41 mm / 0.27° |
| Installed wrist RGB-D | 1.58–1.60 mm median; 4.07 mm worst 95th percentile |
| Installed side RGB-D | 1.18–1.71 mm median; 3.51 mm worst 95th percentile |
| Installed cross-camera agreement | 2.11–2.47 mm median; 6.14 mm worst 95th percentile |

Three fresh candidate and three fresh installed observations passed through
`xarm_observe` and `xarm_pixel_to_position` at 640×480, 6 FPS. Collection RGB used
1280×720, 6 FPS. Acceptance limits: pose residual ≤5 mm / 1°; RGB-D median ≤5 mm,
95th percentile ≤10 mm; cross-camera 95th percentile ≤10 mm; at least 12 valid
points per view and shared across views; arm translation during capture ≤0.2 mm.

Wrist fitted RGB intrinsics, stereo depth alignment and metric depth correction
are unchanged. The side camera retains its factory RGB/depth intrinsics.
The side SDK-aligned depth receives a software factor of 0.9927937879,
qualified at 640×480, 6 FPS over the measured board range [0.768, 0.923] m.
Other RGB-D profiles require their own metric qualification. The correction is
guarded by camera serial, factory intrinsics, profile, depth units and a ±2% bound.
It cannot be combined with fitted stereo/depth registration. Both files retain
their device/profile checks; only the wrist intrinsics file has an unchanged hash.
The wrist software depth factor remains 1.0048860716612322; firmware is unchanged.
Earlier optical-calibration evidence is under
`/home/jack-xiang/rax-runtime/camera-recalibration-20261030T003223Z/`.

Matrices use metres and column vectors: `T_A_B` maps B coordinates into A.
`wrist_T_cam_tcp.yaml` maps TCP to wrist camera; `wrist_T_cam_flange.yaml` maps
flange to wrist camera; `side_T_cam_base.yaml` maps base to side camera.
Wrist base pose is `T_base_tcp @ inverse(T_cam_tcp)`; side pose is `inverse(T_cam_base)`.
TCP offset is [0.0, 0.0, 172.0, 0.0, 0.0, 0.0] mm/rad. Payload/CoG readbacks are
controller settings, not physical verification of the assembly.

Observed board/tabletop region and calibration poses: +/-25 mm sideways, 0–50 mm upward, rotations up to 21.2 degrees. Full-workspace accuracy and manipulation are not established by these measurements.
Recalibrate after mount movement; account for any TCP-setting changes.

Evidence: `/home/jack-xiang/rax-runtime/camera-recalibration-20261030T052107Z/` contains collection, candidate/installed verification,
the previous calibration backup and the installed file-hash manifest.
Repeat with `scripts/xarm-charuco-calibrate --preflight --use-installed-intrinsics`,
then the operator-authorized collection using `--yes --prepare-hand` into a candidate
directory. Run `scripts/xarm-charuco-verify` before and after installation.
The motion flags authorize real arm motion; keep the board fixed throughout.
