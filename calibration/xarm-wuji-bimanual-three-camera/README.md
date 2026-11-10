# Three-camera RGB calibration — November 10, 2026

Verified for `wrist_left` (108322071838), `wrist_right` (752112070450),
and fixed `front` (348522070593). Wrists use 640×480; front uses 1280×720;
all use 15 FPS. The original ChArUco board definition is preserved in `board.json`.

Both sequential 18-pose sweeps accepted all views, including three reserved
views per wrist. The downward, inward-roll sweep used the lowered stationary
board. The frozen fit then passed 57 fresh camera checks: a home observation
and three fresh poses per arm with three captures per pose. Verification used
the documented 20-pixel coverage margin, with unchanged motion and acceptance
limits. Both arms returned to their starting poses, and all four devices were
confirmed stopped and disarmed.

| Camera | Maximum position residual | Maximum rotation residual |
| --- | ---: | ---: |
| Left wrist | 2.049 mm | 0.692° |
| Right wrist | 4.636 mm | 0.766° |
| Front | 1.917 mm | 0.916° |

All checks passed 5 mm / 1° and mean reprojection below 1 pixel. These are
board-pose consistency checks, not independent absolute positioning accuracy.
Depth calibration remains unqualified.

`fit-result.json` preserves the frozen fit. `verification-report.json` contains
all fresh checks and its fit hash. `manifest.json` contains runtime identities,
models, transforms and verification summaries. No older five-camera files were
replaced. The three-camera example and calibration session select this bundle;
an already running backend must restart to load it.

Collection and motion evidence:
`/home/jerry/rax-runtime/sessions/20261110T074606Z_three_camera_calibration_12827c/operator/collection-lowered`.
Fresh verification:
`/home/jerry/rax-runtime/sessions/20261110T074606Z_three_camera_calibration_12827c/operator/verification-lowered-margin20`.

The board can move after calibration. A moved camera mount, arm base, changed
camera identity or TCP offset requires checking the affected calibration.
