# Bimanual ChArUco extrinsics

`scripts/xarm-bimanual-charuco-calibrate` adapts `real/xarmwuji`'s original
18-pose workflow for two xArm bases, two wrist cameras and all configured fixed cameras
(currently the single fixed `front` view).
It uses the existing bimanual MCP owner for motion and separate read-only SDK
connections for precise pose snapshots. It never opens another Wuji owner or
changes finger targets. Only one arm moves at a time.

## Three cameras: two wrists and front

Use `examples/xarm-wuji-bimanual-three-camera.json` for `wrist_left`
(`108322071838`), `wrist_right` (`752112070450`), and fixed `front`
(`348522070593`). No side cameras are required. This example preserves the
arm/hand settings and uses RGB at 15 FPS: 640×480 wrists and 1280×720 front.
It now selects the verified November 10 three-camera bundle in
`calibration/xarm-wuji-bimanual-three-camera/manifest.json`. Before a new full
calibration, use a temporary operator configuration without `calibration_manifest`
so the camera owner supplies raw factory RGB images.

The board is unchanged: 10×13 physical squares, 20 mm squares, 15 mm markers,
with the legacy 14-row ID layout and its final row absent. Keep it stationary
through both sweeps and verification, visible to all three cameras.

Create the session using this backend-options file. Collection now inherits
the session camera resolutions and `stream_fps`; `--width`, `--height`, and
`--fps` can explicitly override them. Verification uses the frozen fit's profile,
including legacy 6 FPS fits. The eventual run must use that same profile.

```bash
export RAX_BACKEND_OPTIONS="$PWD/examples/xarm-wuji-bimanual-three-camera.json"

# SESSION is a configured bimanual session. Preflight does not run a sweep.
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --preflight --session SESSION --root PREFLIGHT_ARTIFACTS \
  --sweep-z-direction down

# After checking preflight and preparing the session:
.venv/bin/python scripts/xarm-bimanual-charuco-calibrate \
  --collect --yes --session SESSION --mcp-socket MCP_SOCKET \
  --root COLLECTION_ARTIFACTS --sweep-z-direction down
```

Release the standalone live viewer before capture so there is only one camera
owner. Follow the fit and fresh-verification steps below. Full collection fits
both wrists, front-to-base transforms, and the inter-base alignment. Install
only a bundle that passes fresh verification, then point `calibration_manifest`
at it. Selecting this example alone does not validate any geometry.

Use `--wrists-only` for a separate wrist stage when the fixed cameras cannot
see the wrist calibration board. Keep that board stationary through both arm
sweeps and fresh verification; the stationary peer wrist monitors board motion.
This fits the two wrists and inter-base transform only. Verification inherits
the two-camera scope from the fit. Fixed views remain unverified when installing
this partial bundle and can be calibrated later against the verified wrists
using the fixed-camera workflow below, with the board at a different height.

Start from the operator's current per-arm saved homes, reported as
`arms.<side>.home_joints` by `xarm_status`; the left and right homes can differ.
The sweep is relative to each arm's measured starting TCP pose and returns to
that same pose. Updating a home does not change the rigid wrist-to-TCP transform,
but it changes board visibility and the sweep's reachability, so run preflight
at the new homes before collection. Do not substitute the old single-arm preset.

For higher homes, use `--sweep-z-direction down` in preflight and collection.
This reverses the original 15–50 mm vertical offsets while retaining all 18
rotations and three reserved views per arm. Coverage-guided follow-up and fresh
verification inherit that vertical envelope from the planning collection.
Both tools screen every Cartesian segment and the return path at intervals of
at most 5 mm and 2°, checking controller IK, joint-limit margins and continuity.
A finite-difference FK Jacobian, with translation normalized by 0.3 m, must have
minimum singular value at least 0.05 and condition number at most 50 at every
sample. These sampled kinematic checks do not check collisions; the existing
workspace and controller protections remain active. Reports are saved in
`kinematic-checks.json` before collection or verification motion.

When the board sits near the inner edges of the wrist images,
`--frame-wrists-inward` changes the left roll range to −15°..0° and the right
range to 0°..15°, keeping pitch/yaw within the original limits and retaining
18 poses and three reserved views. Pose zero retains the actual starting home
orientation as a training observation. Use fresh captures after changing this plan.
`--coverage-margin-px 20` permits subsequent coverage planning to use a smaller
full-board image border than the default 80 pixels; 20 pixels is the minimum.
It changes only pose selection, not the corner-count, reprojection, pose-error,
workspace or singularity checks. Follow-up planning inherits these settings.
Coverage-guided collections also retain the actual home as pose zero, even
when that view is cropped; the remaining 17 poses use the full-board border.

The board may be placed differently from an earlier calibration. This full
workflow solves its new pose and the alignment between the arm bases; it does
not compare current observations to the old board placement. Keep the board
fixed across both passes, any coverage-guided follow-up, and fresh verification.
This installation uses
`calibration/xarm-wuji/board.json`: 10×13 physical squares, 20 mm squares,
15 mm markers, `DICT_4X4_100`, with the verified legacy 14-row marker layout
and the final physical row absent. A standard 13-row layout does not match
the observed IDs. The print must lie flat.

Camera identities come from the session's configuration: `wrist_left` belongs
to the left arm, `wrist_right` to the right arm, and `front` is fixed. Legacy
side cameras are optional. The current profiles are 640×480 for both wrists and
1280×720 for front, all at 15 FPS, with live factory RGB intrinsics for collection.
The default procedure fits extrinsics only. The optional
`--refine-wrist-intrinsics` applies the original script's training-only Brown5
RGB lens refinement; held-out images do not participate in that fit. It saves
the resulting color models with the candidate transforms. Those transforms
must be used with their matching color models, not the factory models. Neither
mode changes depth scale, firmware, joint origins or TCP offsets. Old camera
extrinsics are not reused.

For a sweep with limited optical diversity, `--refine-wrist-distortion` fits
only radial coefficients k1/k2, keeping factory focal lengths and principal
point fixed and the other distortion terms zero. This limits focal drift when
the sweep cannot constrain a fuller model. It does not guarantee an adequate
fit: the November 3 recalibration needed Brown5/K refinement to pass fresh home
and sweep verification. Select the model using training data and verify the
frozen result with new captures before installation.
It uses OpenCV's [fixed-intrinsic-parameter calibration flags](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).
`--refine-wrist-brown-distortion` instead fits all five Brown distortion terms
while retaining the factory focal lengths and principal point. These refinement
modes are mutually exclusive and use training images only; none changes acceptance
limits. After choosing a model from diagnostic fits, verify it on fresh images.
The optical training images must cover the parts of the image used at runtime.
`--optical-training-collection EARLIER_COLLECTION` includes only the earlier
collection's non-reserved images in the lens fit, while retaining the current
collection's pose-training set for hand-eye fitting. This allows edge views to
constrain the lens model without substituting sparse edge views for well-framed
hand-eye observations. Every optical training frame is recorded in the model.

The collector can own capture or borrow the existing session camera owner.
For borrowed capture, load a temporary operator backend configuration with no
`calibration_manifest`, raw factory RGB capture, and the collection speed settings
below. Pass `--camera-socket SOCKET` to both collection and verification; the
existing live viewer stays on port 8765. The collector checks each frame's serial,
profile and raw factory source, stages captures through the owner's frames root,
and copies them into its calibration artifacts. It never opens a second camera
owner or applies the old calibrated optical models.

Without `--camera-socket`, release the existing camera owner and live viewer first.
The collector then owns capture and a temporary preview on port 8765. Restore the
normal calibrated backend afterward in either mode.

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
arm, with three separate captures per pose. It checks every configured camera in each
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

Candidates are written only when every configured camera check passes. This script does
not install them into the running backend or claim depth/pixel-to-base support;
those runtime capabilities require an explicit configuration integration step.

## Calibrating individual fixed cameras

The current setup uses `--target front --reference wrist_left wrist_right`.
The side-camera commands below apply only to legacy or expanded configurations
that actually include those views.

When both reference cameras are fixed, `--camera-socket CAMERA_SOCKET` can replace
`--mcp-socket MCP_SOCKET` to capture directly from an existing camera server.
This mode never opens arm or hand connections, uses the frozen fixed-camera
transforms, and rejects wrist-camera references because they require measured arm
poses. It remains usable while the robot service and hand power are off.

When fixed mounts move or a fixed view is added, preserve the unchanged camera transforms,
both wrist models and the inter-base transform. The board can be in a new
position. Place it flat and stationary with at least twelve noncollinear
ChArUco corners visible in each camera. Use the existing board definition,
including its legacy 14-row ID layout with the final physical row absent.

With a running bimanual MCP backend and stationary arms:

```bash
.venv/bin/python scripts/xarm-bimanual-fixed-camera-calibrate \
  --session SESSION --mcp-socket MCP_SOCKET --target side_a \
  --output NEW_CALIBRATION_DIRECTORY --install

# If both side mounts moved, neither side camera may be an anchor:
.venv/bin/python scripts/xarm-bimanual-fixed-camera-calibrate \
  --session SESSION --mcp-socket MCP_SOCKET --target side_a side_b \
  --output NEW_CALIBRATION_DIRECTORY --install

# A new front view can use the two unchanged wrists as references:
.venv/bin/python scripts/xarm-bimanual-fixed-camera-calibrate \
  --session SESSION --mcp-socket MCP_SOCKET --target front \
  --reference wrist_left wrist_right --output NEW_CALIBRATION_DIRECTORY --install

# For cropped planar views, use shared corners to constrain the reference pose:
.venv/bin/python scripts/xarm-bimanual-fixed-camera-calibrate \
  --session SESSION --mcp-socket MCP_SOCKET --target front \
  --reference wrist_left wrist_right --reference-method stereo \
  --output NEW_CALIBRATION_DIRECTORY --install
```

This command only calls `xarm_observe`; it does not prepare or move any robot.
It fits the selected fixed cameras from twelve captures using unchanged camera
poses with equal weights. By default, references are all calibrated, non-target
views. `--reference` selects at least two unchanged calibrated views when other
views cannot see the board. Targets and unverified views cannot be references.
New targets must already be configured as fixed cameras and explicitly marked
unverified; their live factory RGB profile is frozen on the first capture and
checked throughout collection. Existing optical models are never refitted here.
It freezes the fits before eight fresh captures and checks
each target against every reference camera. It retains the 5 mm / 1°
consistency limits and 1-pixel mean reprojection limit. Detector selection uses
decoded corner count, never geometric residual. No failed observation is silently
discarded. Omit `--install` to retain a candidate without updating the bundle.

The optional stereo method uses exactly two frozen reference cameras. It requires
at least twelve shared, noncollinear ChArUco corners spanning 40 mm in both board
axes, with converging rays between 10° and 170°. It triangulates those corners
and fits the known metric board by a rigid transform, without rescaling the board
or discarding outliers. This avoids averaging independently estimated planar
poses when cropped views poorly constrain their tilt. Each capture must pass
5 mm limits on viewing-ray distance and rigid-board residual. The original
1-pixel per-image board reprojection check remains in place. Fresh verification
checks the frozen target transform against the stereo board pose at 5 mm / 1°,
and also checks every triangulated corner against the target's inferred board
at 5 mm. Reports distinguish this method from independent-pose averaging and
retain all reference geometry checks. Existing reference calibration uncertainty
still applies; neither method independently measures absolute robot accuracy.
The projection and pose conventions follow
[OpenCV's calibration documentation](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html).

Installation requires every selected target to pass and replaces only their
base-transform files and manifest entries, with a dedicated report for each.
New targets also add their camera specification and measured factory color model.
All target entries are included in one atomic manifest replacement. The original
verification report remains historical evidence for the unchanged calibration.
After a passing installation, remove only the fitted targets from
`unverified_camera_extrinsics` in the session and resume configurations.
Restart the backend after installation: an already running backend retains its
previous manifest. Fresh stationary observations establish local board-pose
consistency; they do not replace a wrist hand-eye sweep if a wrist mount or arm
base has moved. See the [installed bundle](../calibration/xarm-wuji-bimanual/README.md)
for the latest fixed-camera update.
