# Bimanual ChArUco extrinsics

`scripts/xarm-bimanual-charuco-calibrate` adapts `real/xarmwuji`'s original
18-pose workflow for two xArm bases, two wrist cameras and two fixed cameras.
It uses the existing bimanual MCP owner for motion and separate read-only SDK
connections for precise pose snapshots. It never opens another Wuji owner or
changes finger targets. Only one arm moves at a time.

The board must remain fixed across both passes. This installation uses
`calibration/xarm-wuji/board.json`: 10×13 physical squares, 20 mm squares,
15 mm markers, `DICT_4X4_100`, with the verified legacy 14-row marker layout
and the final physical row absent. A standard 13-row layout does not match
the observed IDs. The print must lie flat.

Camera identities come from the session's configuration: `wrist_left` belongs
to the left arm, `wrist_right` to the right arm, and `side_a`/`side_b` are fixed.
All four are captured at 1280×720, 6 FPS, with live factory RGB intrinsics.
The default procedure fits extrinsics only. The optional
`--refine-wrist-intrinsics` applies the original script's training-only Brown5
RGB lens refinement; held-out images do not participate in that fit. It saves
the resulting color models with the candidate transforms. Those transforms
must be used with their matching color models, not the factory models. Neither
mode changes depth scale, firmware, joint origins or TCP offsets. Old camera
extrinsics are not reused.

For a sweep with limited optical diversity, `--refine-wrist-distortion` fits
only radial coefficients k1/k2, keeping factory focal lengths and principal
point fixed and the other distortion terms zero. This avoids the large focal
drift observed with a free intrinsic fit on this rig's inward-facing sweep.
It uses OpenCV's [fixed-intrinsic-parameter calibration flags](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
Both refinement modes use training images only; neither changes acceptance
limits. After choosing a model from diagnostic fits, verify it on fresh images.
The optical training images must cover the parts of the image used at runtime.
`--optical-training-collection EARLIER_COLLECTION` includes only the earlier
collection's non-reserved images in the lens fit, while retaining the current
collection's pose-training set for hand-eye fitting. This allows edge views to
constrain the lens model without substituting sparse edge views for well-framed
hand-eye observations. Every optical training frame is recorded in the model.

Stop the standalone camera dashboard before collection to release its USB
streams. The collector owns one camera service and exposes a read-only live
preview on port 8765 while running. Restore the normal dashboard afterward.

```bash
export RAX_ARTIFACT_ROOT=/home/jerry/rax-runtime
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --preflight --session SESSION --root PREFLIGHT_ARTIFACTS

# After the board/paths are checked and the existing session is prepared:
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --collect --yes --session SESSION --mcp-socket MCP_SOCKET \
  --root COLLECTION_ARTIFACTS

# Refit saved real captures without hardware access:
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --fit COLLECTION_ARTIFACTS --root FIT_ARTIFACTS

# Use the original wrist RGB refinement when factory intrinsics are insufficient:
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --fit COLLECTION_ARTIFACTS --root REFINED_ARTIFACTS --refine-wrist-intrinsics

# Smaller lens correction, retaining factory K:
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --fit COLLECTION_ARTIFACTS --root RADIAL_ARTIFACTS --refine-wrist-distortion

# Fresh physical verification of a frozen passing result; no refitting:
.venv/bin/python scripts/xarm-bimanual-charuco-verify \
  --fit RADIAL_ARTIFACTS/result.json --planning-collection INITIAL_COLLECTION \
  --session SESSION --mcp-socket MCP_SOCKET --root VERIFICATION_ARTIFACTS --yes
```

Use 15 mm/s Cartesian speed and 50 mm/s² acceleration in each arm's session
profile for collection. The MCP calls also request 15 mm/s. The original
rotation sweep is retained; translations are shortened where necessary to stay
inside the current workspace guard. Every capture records both measured arm
poses and checks that neither arm moved during image acquisition.

If the original sweep clips the board, `--coverage-from COLLECTION_ARTIFACTS`
uses that collection's preliminary geometry solely to plan better-framed views.
It selects 18 diverse poses with the entire projected board at least 80 pixels
inside the image, within the original translation and rotation envelope. The
board, mounts, identities, TCP offsets and starting arm poses must be unchanged.
The new collection declares its three reserved poses before moving and refits
all transforms independently. SDK reachability and normal backend checks still
apply; a preliminary estimate is never installed as calibration.

The verifier checks a fresh home observation and three newly seeded poses per
arm, with three separate captures per pose. It checks all four cameras in each
capture, including the stationary wrist. Its plan and source-fit hash are saved
before moving. It returns both arms to their starting poses and records every
residual without fitting or discarding observations.

Each wrist fit uses at least nine training views and all three reserved views.
The existing 5 mm / 1° pose-residual limits and 1-pixel mean reprojection limit
apply. Fixed cameras use training observations of the stationary board, with
the six observations from reserved arm poses checked separately. The collector
returns each arm to its starting pose after its pass; a motion failure stops the
rig instead of issuing an unconditional return.

`T_A_B` maps column-vector points from B to A, in metres. The two independent
hand-eye fits yield `T_left_base_board` and `T_right_base_board`; their shared
board gives `T_left_base_right_base = T_left_base_board @ inverse(T_right_base_board)`.
Successful fits write a candidate containing:

- Each wrist camera's `T_cam_tcp` and `T_cam_flange`.
- Each fixed camera's transforms from both arm bases.
- The transform between the two arm bases.
- A manifest with camera serials, arm serials/IPs, TCP offsets, board geometry,
  training/reserved residuals and source capture paths.
- Matching wrist RGB models when intrinsic refinement was requested. These
  describe the captured resolution; runtime profile conversion and depth
  alignment still require validation.

Candidates are written only when all four camera checks pass. This script does
not install them into the running backend or claim depth/pixel-to-base support;
those runtime capabilities require an explicit configuration integration step.

## Recalibrating one bumped fixed camera

When only one fixed mount moved, preserve the other three camera transforms,
both wrist models and the inter-base transform. The board can be in a new
position. Place it flat and stationary with at least twelve noncollinear
ChArUco corners visible in each camera. Use the existing board definition,
including its legacy 14-row ID layout with the final physical row absent.

With a running bimanual MCP backend and stationary arms:

```bash
.venv/bin/python scripts/xarm-bimanual-fixed-camera-calibrate \
  --session SESSION --mcp-socket MCP_SOCKET --target side_a \
  --output NEW_CALIBRATION_DIRECTORY --install
```

This command only calls `xarm_observe`; it does not prepare or move any robot.
It fits the selected fixed camera from twelve captures using the three unchanged
camera poses with equal weights. It freezes the fit before eight fresh captures
and checks each against all three reference cameras. It retains the 5 mm / 1°
consistency limits and 1-pixel mean reprojection limit. Detector selection uses
decoded corner count, never geometric residual. No failed observation is silently
discarded. Omit `--install` to retain a candidate without updating the bundle.

Installation replaces only the selected fixed camera's two base-transform files
and its manifest entries, and adds a dedicated verification report. The original
verification report remains historical evidence for the unchanged calibration.
Restart the backend after installation: an already running backend retains its
previous manifest. Fresh stationary observations establish local board-pose
consistency; they do not replace a wrist hand-eye sweep if a wrist mount or arm
base has moved. See the [installed bundle](../calibration/xarm-wuji-bimanual/README.md)
for the latest fixed-camera update.
