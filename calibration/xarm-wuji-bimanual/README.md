# Verified bimanual RGB extrinsics

On November 4, the operator accepted the unchanged wrist transforms and lens
models at the higher saved homes with limits of **10 mm / 1° per wrist** and
**20 mm / 1.5° between wrists in the shared base frame**. Today's maximum
residuals were 8.58 mm / 0.94° left, 5.72 mm / 0.56° right, and
17.62 mm / 1.18° between wrists. These limits were selected after reviewing
the measurements. The original stricter results remain in
[`wrist-validation-20261104.json`](wrist-validation-20261104.json); the manifest's
`operator_acceptance` records the approved limits and is exposed in MCP status.
No transforms, lens models or robot motion limits changed. The verification
figures below describe the earlier November 3 validation.

## November 4 fixed-camera update

All three fixed cameras were recalibrated against shared board corners
triangulated by the unchanged wrist cameras. Twelve training captures and eight
fresh verification captures passed the original 5 mm / 1° and 1-pixel limits:

| Camera | Maximum position residual | Maximum angle residual |
| --- | ---: | ---: |
| Side A | 0.49 mm | 0.137° |
| Side B | 0.45 mm | 0.109° |
| Front | 1.48 mm | 0.110° |

At least 46 shared corners were available. Maximum stereo ray error was
4.63 mm, board-rigidity error 2.57 mm, and target-to-triangulated-corner error
3.49 mm. Independent single-camera wrist PnP estimates disagreed by up to
33.04 mm / 1.425° at this raised placement; stereo transfer did not average
those poses. These are consistency residuals relative to the retained wrist
geometry, not independent absolute accuracy. Both wrist transforms, every lens
model, inter-base alignment and the operator's wrist acceptance are unchanged.
All five cameras are enabled for calibrated RGB geometry in the saved configs.
See the current `side_a-recalibration-report.json`,
`side_b-recalibration-report.json` and `front-recalibration-report.json`.

## November 3 history

Full calibration captured and verified on November 3, 2026, after both side
camera mounts were adjusted. That full pass established all four camera extrinsics,
both wrist RGB models, and the alignment between the two arm bases. Both side
views were subsequently refitted on November 3 at 16:54 UTC; Side A was secured and refitted at 16:58 UTC;
both wrist calibrations, all RGB models and the inter-base alignment were preserved.
The front view was added on November 3 using stationary stereo observations from
the unchanged wrist cameras. No earlier camera model or transform was refitted.
`examples/xarm-wuji-bimanual.json` selects this bundle. Front serial
`348522070593` was subsequently reconnected and its placement was recalibrated
in the November 4 update above; the table below records the earlier results.

| Role | Serial | Fresh maximum position residual | Fresh maximum angle residual |
| --- | --- | ---: | ---: |
| Left wrist | 108322071838 | 4.06 mm | 0.402° |
| Right wrist | 752112070450 | 4.48 mm | 0.430° |
| Side A | 263322070113 | 3.99 mm | 0.504° |
| Side B | 262422074331 | 3.26 mm | 0.451° |
| Front | 348522070593 | 0.43 mm | 0.205° |

The original full sweep's 76 fresh checks passed the existing 5 mm / 1° pose-consistency limits and
1-pixel mean reprojection limit. The largest mean reprojection error was
0.743 pixels. These are board-pose consistency residuals, not independently
measured absolute robot positioning accuracy. Depth scale and RGB-depth
registration remain unqualified; no depth pixel-to-position endpoint is exposed.

## Front-camera addition

The front mount was bumped after the initial calibration and refitted on
November 3 at 20:16 UTC using the same stereo method. Twelve new fitting captures
and eight fresh verification captures passed: maximum pose residual 0.43 mm /
0.205°, maximum triangulated-corner residual 0.79 mm, and maximum mean front
reprojection 0.468 pixels. Two additional captures passed through the restarted
MCP backend. Only the front extrinsics changed; all five RGB models, both wrists,
both side extrinsics and the inter-base transform were preserved exactly.
The November 3 evidence is retained in the hashed historical front report and
`/home/jerry/rax-runtime/sessions/20261103T073444Z_unscrew_bottle_b1d812/operator/front-recalibration-20261103T201605Z/`.
The suffixed front report preserves the initial calibration described below.

The final front mount was calibrated with the board lying flat on the wooden
work surface. Earlier captures with the board on a box failed image or pose
consistency checks and were not installed. Independent planar-pose averaging
also failed the 5 mm / 1° check on flat-board captures (6.19 mm / 1.15°).
The accepted method instead triangulated shared corners using the fixed wrist
models, then fitted a rigid metric board pose without scale changes or outlier
removal. Reference viewing-ray and board-rigidity residuals were checked in
metres; the per-image 1-pixel board reprojection requirement remained unchanged.

Twelve new captures fitted only the front transform, followed by eight fresh
verification captures after freezing it. The fresh maxima were 0.39 mm / 0.318°
for board-pose consistency, 0.99 mm for triangulated-corner consistency,
0.455 pixels for mean front reprojection, 2.01 mm for reference-ray distance,
and 0.72 mm for reference-board rigidity. At least 24 shared wrist corners were
available in every verification frame. These residuals inherit the existing
wrist-calibration uncertainty and do not establish submillimetre absolute accuracy.

`front-recalibration-report.json` records the fit, frozen reference models,
individual captures and verification checks. Raw captures and failed diagnostic
attempts are retained under
`/home/jerry/rax-runtime/sessions/20261103T073444Z_unscrew_bottle_b1d812/operator/front-calibration-20261103T190044Z/`.
The accepted candidate is `fit-stereo/result.json`. Both side transforms, both
wrist transforms, all four existing color models and the inter-base transform
are unchanged from the preceding manifest.

## Latest side-camera update

The operator first requested Side B alone. A fit using Side A and both wrists
failed because Side A disagreed with the wrists by 21–24 mm. Matching fixed
background features also found about 24 pixels of Side A image displacement
since the verified full sweep. The operator then requested both side views be
recalibrated while preserving the wrists.

Both hands were commanded home, followed by sequential arm home commands.
The right hand reached its saved pose; the partial left hand completed its
reference with an accepted 0.01783-rad shortfall. All four stops were confirmed.
Calibration then used stationary arms and the unchanged wrist cameras as the
two equally weighted references. Neither side camera was used as an anchor.

Twelve four-camera captures fitted the fixed transforms. The fit was frozen
before eight fresh captures, yielding sixteen reference comparisons per side
camera. All 32 fresh comparisons passed the unchanged 5 mm / 1° limits.
Side A shifted again by about 50 pixels before the post-reload check, which
rejected its new geometry. Side B passed. After the operator secured Side A
and its cable, a separate twelve-capture fit used verified Side B and both
unchanged wrists. Eight fresh frames supplied 24 passing comparisons. Only
Side A was replaced in this final step; Side B was preserved byte-for-byte.
The side-camera table entries describe these latest passing fits; wrist entries
retain their full-sweep results. Stationary checks establish local board-pose
consistency, not a new wrist sweep or independent world-coordinate accuracy.

Current reports are `side_a-recalibration-report.json` and
`side_b-recalibration-report.json`. Captures, the failed first fit, successful
candidates, home records and prior bundle are preserved under
`/home/jerry/rax-runtime/sessions/20261103T073444Z_unscrew_bottle_b1d812/operator/side-b-recalibration-20261103T164953Z`.
After the final reload, fresh MCP observations in both base frames passed all
eight side-to-wrist comparisons (maximum 3.47 mm / 0.581°). All four views
reported verified geometry, and equivalent points projected to the same pixels
from either base. Evidence is in `final-runtime-verification.json` there.

## Collection and verification

The board stayed fixed throughout two sequential 18-pose sweeps per arm and
fresh verification. The initial sweep had clipped wrist views. Its usable
non-reserved images supplied additional optical training and preliminary
geometry for planning the second sweep. All 36 second-sweep observations were
accepted: 15 training poses and three reserved poses per wrist.

The constrained k1/k2 model passed the collection checks but failed fresh home
verification and was not installed. A fuller Brown5/K refinement used only
non-reserved optical training images from both sweeps. Hand-eye fitting used
only the second sweep's 15 training poses per arm. The resulting fit was frozen
before a new home capture and six newly seeded verification poses, three per
arm. Three fresh captures per pose checked all four cameras, including the
stationary wrist. No verification image was fitted or discarded. Both arms
returned to their starting home poses; finger targets were unchanged.

The installed wrists require their matching fitted RGB models. The backend
rectifies their images and supplies the corresponding K and zero distortion
for those saved images. Side cameras retain their factory RGB models. Use each
observation's saved metadata rather than substituting factory wrist intrinsics.

After the full calibration's installation, fresh MCP observations in both arm base frames passed all
eight camera checks with the normal runtime image pipeline. Equivalent points
projected to the same reported pixels from either base. All four views reported
`rgb_extrinsics_verified`; the resumed session has no unverified-camera overrides.
That evidence is saved as `runtime-verification.json` in the capture directory.

## Files and conventions

`T_A_B` maps column-vector points from B to A, with translations in metres.

- `wrist_*_T_cam_tcp.yaml` and `wrist_*_T_cam_flange.yaml`: each wrist camera
  relative to its owning arm's TCP and flange.
- `side_*_T_cam_{left,right}_base.yaml`: each side camera relative to both bases.
- `front_T_cam_{left,right}_base.yaml`: the front camera relative to both bases.
- `T_left_base_right_base.yaml`: right base into left base.
- `*_color_model.json`: matching 1280×720 @ 6 FPS RGB models.
- `board.json`: 10×13 physical squares, 20 mm squares and 15 mm markers,
  retaining the verified legacy 14-row ID layout with the final row absent.
- `fit-result.json`: exact frozen fit used for fresh verification.
- `verification-report.json`: all 76 fresh checks, fit hash and return poses.
- `manifest.json`: identities, models, geometry, fit and verification summaries.

Left arm: `192.168.2.205`, serial `XI130510J56A06`.
Right arm: `192.168.1.241`, serial `XI130505F56A0A`.
Both TCP offsets were `[0, 0, 172, 0, 0, 0]` in mm/radians.

The board may be removed or moved after calibration. Its saved reference plane
is historical and is not automatically the task's support surface. Arm motion
updates wrist poses using telemetry; changing a camera mount, camera identity,
TCP offset or arm base requires checking the affected geometry.

## Provenance and earlier calibrations

Current captures, diagnostic failures, the passing fit and fresh verification:
`/home/jerry/rax-runtime/sessions/20261103T073444Z_unscrew_bottle_b1d812/operator/full-recalibration-20261103T100920Z`.
The accepted fit is `full-intrinsic-diagnostic/result.json`; verification is
`verification-full-optics/verification.json`. The frozen fit SHA-256 is
`b4099e8d08dbe4ef43029088bbb425562bc3594f6e3c2b2e305e45fbd92041df`.

The prior full-calibration bundle and README are preserved under `bundle-before/`
and `README-before.md` in that directory. Suffixed repository
`side_*-recalibration-report-*.json` files preserve earlier fixed-camera reports;
the unsuffixed reports describe the latest update above. The original full-sweep
`verification-report.json` remains unchanged as evidence for the preserved wrists.

See [the operator workflow](../../docs/xarm-wuji-bimanual-calibration.md).
