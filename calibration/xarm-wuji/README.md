# xArm + Wuji camera calibration

Installed and verified on 2026-10-27 with a new, correctly sized 10×13
ChArUco print: 20 mm squares, 15 mm markers, DICT_4X4_100. The observed marker
IDs use the legacy 14-row layout with the last row absent. `board.json` records
that mapping; it does not add a physical row to the target.

`examples/xarm-wuji.json` references these files. Runtime RGB-D is 640×480 at
6 fps; color-only calibration and preview also support 1280×720 at 6 fps.

| Camera | Serial | Calibrated quantities |
| --- | --- | --- |
| Wrist D435 | 262422074331 | RGB intrinsics/distortion, depth-to-color alignment, camera-to-TCP/flange |
| Fixed side D435 | 348522070593 | Camera-to-base extrinsics; factory RGB/depth intrinsics retained |

Both intrinsics JSON files retain the factory readback as a device/profile
guard. Wrist RGB is rectified with the fitted Brown5 model. Native depth is
projected through the fitted stereo transform into that rectified color model,
using a nearest-surface Z buffer. Depth values represent color-camera Z.
Factory depth units remain 0.0010000000474974513 m; no firmware calibration,
depth-unit setting, TCP offset, or payload was changed.

## Validation

RGB intrinsics and hand-eye fitting used 15 training poses from run06; the last
three poses were reserved. The additional run07 supplied 12 usable paired
infrared/RGB training views for stereo alignment. All three reserved RGB/depth
views were checked independently, including one without enough infrared
corners for a separate IR pose estimate. No reserved view entered either fit.

| Check | Result |
| --- | --- |
| Wrist RGB training reprojection RMS | 0.260 px |
| Wrist hand-eye maximum training residual | 1.22 mm / 0.70° |
| Wrist hand-eye maximum reserved residual | 1.43 mm / 0.42° |
| Fixed side-camera maximum consistency residual | 0.80 mm / 0.18° |
| Infrared/RGB stereo reprojection RMS | 0.368 px |
| Reserved wrist RGB-D errors, per-view median | 1.73–3.59 mm |
| Reserved wrist RGB-D errors, worst per-view 95th percentile | 5.04 mm |
| Installed MCP wrist board-point error, median / 95th percentile | 3.60 / 5.46 mm |
| Installed MCP side board-point error, median / 95th percentile | 1.63 / 3.61 mm |
| Installed MCP cross-camera difference, median / 95th percentile | 4.34 / 6.17 mm |

The final MCP comparison used 16 common board corners at runtime resolution.
`xarm_status`, `wuji_status`, `xarm_observe`, `xarm_stream`, and
`xarm_pixel_to_position` succeeded. Gates were unchanged: hand-eye ≤5 mm / 1°;
RGB-D median ≤5 mm and 95th percentile ≤10 mm; cross-camera 95th percentile
≤10 mm. The camera/workspace/arm/hand regression suite passed 102 tests.

The arm returned to [0.25310, -0.00134, 0.31412] m. Final readback showed
controller state 4, no arm error, disarmed session, and fresh, disabled hand
feedback with no faults.

## Coordinate conventions and scope

All matrices use column vectors and metres. `T_A_B` maps coordinates from B
into A:

- `wrist_T_cam_tcp.yaml`: TCP coordinates → wrist color-camera coordinates.
- `wrist_T_cam_flange.yaml`: flange coordinates → wrist color-camera coordinates.
- `side_T_cam_base.yaml`: arm-base coordinates → side color-camera coordinates.
- `calibrated_depth_to_color.T_color_depth`: native depth coordinates → fitted
  wrist color-camera coordinates.

For the wrist, `T_base_cam = T_base_tcp @ inverse(T_cam_tcp)`. For the fixed
side camera, `T_base_cam = inverse(T_cam_base)`.

The TCP readback during collection was [0, 0, 172, 0, 0, 0] mm/rad; payload was
0.82 kg with CoG [0, 0, 48] mm. These are existing controller settings, not
measured Wuji palm geometry. Recompute the TCP-relative transform if that TCP
changes. Recalibrate after moving either camera mount. Accuracy is qualified
around the observed tabletop and bounded calibration poses; full-workspace
accuracy and manipulation remain unqualified.

The initial run06 RGB-only model passed hand-eye checks but failed wrist depth
validation by approximately 15 mm. Raw IR/depth inspection and the paired run07
showed that the fitted RGB model also needed its own depth-to-color alignment.
The installed stereo model passed both reserved-frame and fresh MCP checks;
the rejected factory-alignment candidate was never installed.

## Evidence and repeatability

`calibration-report.json` contains residuals, validation limits, board definition,
runtime measurements and final assembly state. Raw evidence remains local:

- `.local-runtime/sessions/charuco-wuji-20261027-run06/`: 18 raw RGB poses.
- `.local-runtime/sessions/charuco-wuji-20261027-run06-fit/`: RGB fit and reserved checks.
- `.local-runtime/sessions/charuco-wuji-20261027-run07/`: synchronized native IR/depth/RGB.
- `.local-runtime/sessions/charuco-wuji-20261027-run07-stereo-reproduced/`: reproducible stereo fit and depth checks.
- `.local-runtime/sessions/camera-mcp-check-20261027-installed/`: final production MCP audit and frames.

Stop the camera viewer before calibration or an MCP session; each owns the
camera devices. The camera-only viewer is available at <http://localhost:8765/>.
UFactory Studio is hosted by the arm at <http://192.168.100.16:18333/>.

For a future collection, use a new artifact directory and confirm a fixed,
flat board and fixed mounts. Preflight is read-only:

```bash
export RAX_ARTIFACT_ROOT=/home/jerry/rax/.local-runtime
.venv/bin/python scripts/xarm-charuco-calibrate --capture-native \
  --preflight --root "$RAX_ARTIFACT_ROOT/sessions/new-preflight"
```

After operator clearance and arm readiness, collection uses `--capture-native
--yes --prepare-hand` and a new root. These flags authorize real robot movement.
The hand holds its measured posture; this is not finger homing. Collections
save raw factory-profile RGB even when a fitted model is already installed.

Offline fitting uses:

```bash
.venv/bin/python scripts/xarm-charuco-fit \
  --collection "$RAX_ARTIFACT_ROOT/sessions/new-collection" \
  --root "$RAX_ARTIFACT_ROOT/sessions/new-rgb-fit"
.venv/bin/python scripts/xarm-charuco-stereo-fit \
  --collection "$RAX_ARTIFACT_ROOT/sessions/new-collection" \
  --rgb-fit "$RAX_ARTIFACT_ROOT/sessions/new-rgb-fit" \
  --root "$RAX_ARTIFACT_ROOT/sessions/new-stereo-fit"
```

Fitting writes candidates only. Before installation, keep the board fixed and
verify fresh `xarm_observe` and `xarm_pixel_to_position` results through the
[normal MCP frontend](../../docs/xarm-wuji.md#start-a-session-for-mcp-testing).
Compare board points and common corners between cameras against the limits
above. Passing image-fit residuals alone is insufficient for RGB-D validation.
The one-off MCP verifier used for the recorded commissioning check is archived
locally at `.local-runtime/calibration-session-cleanup/scripts/xarm-camera-verify`.
The stereo fitter can use a separate collection if the board, robot base and camera mounts remain
fixed, as in this calibration.
