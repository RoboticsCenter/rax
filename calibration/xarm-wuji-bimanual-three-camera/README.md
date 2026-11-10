# Three-camera RGB calibration — November 10, 2026

Verified for `wrist_left` (108322071838), `wrist_right` (752112070450),
and fixed `front` (348522070593). Wrists use 640×480; front uses 1280×720;
all use 15 FPS. The original ChArUco board definition is preserved in `board.json`.

Recalibrated after securing the cameras, replacing the board with an identical
flatter print, and improving the lighting. Only the new captures under the
improved lighting contribute to this bundle. Both sequential 18-pose sweeps
completed: the left accepted 14 training and all three reserved views, and
the right accepted all 15 training and three reserved views. One cropped left
training view was rejected for insufficient corners. The downward, inward-roll
sweep used the stationary board. The frozen fit then passed 57 fresh camera checks: a home observation
and three fresh poses per arm with three captures per pose. Verification used
the documented 20-pixel coverage margin, with unchanged motion and acceptance
limits. The planner searched 48,000 candidates within the same motion envelope
to find fully framed verification poses. Both arms returned to their saved homes, and all four devices were
confirmed stopped and disarmed.

| Camera | Maximum position residual | Maximum rotation residual |
| --- | ---: | ---: |
| Left wrist | 2.924 mm | 0.594° |
| Right wrist | 3.409 mm | 0.974° |
| Front | 1.301 mm | 0.178° |

All checks passed 5 mm / 1° and mean reprojection below 1 pixel. These are
board-pose consistency checks, not independent absolute positioning accuracy.
Depth scale and registration remain factory-provided and independently unqualified.
RGB-D remains enabled for all three cameras in the active session.

`fit-result.json` preserves the frozen fit. `verification-report.json` contains
all fresh checks and its fit hash. `manifest.json` contains runtime identities,
models, transforms and verification summaries. No older five-camera files were
replaced. Both bimanual examples and the active lightbulb session select this bundle;
an already running backend must restart to load it.

Collection and motion evidence:
`/home/jerry/rax-runtime/sessions/20261110T091931Z_screw_lightbulb_fable52/operator/all-camera-improved-lighting-20261110T211032Z/collection`.
Fresh verification:
`/home/jerry/rax-runtime/sessions/20261110T091931Z_screw_lightbulb_fable52/operator/all-camera-improved-lighting-20261110T211032Z/verification`.

The previous installed bundle and both session configurations were backed up
under that attempt's `original-bundle/` and operator directory before installation.

The board can move after calibration. A moved camera mount, arm base, changed
camera identity or TCP offset requires checking the affected calibration.
