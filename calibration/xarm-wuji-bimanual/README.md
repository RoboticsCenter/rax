# Verified bimanual RGB extrinsics

Full calibration captured and verified on November 3, 2026, after both side
camera mounts were adjusted. This bundle replaces all four camera extrinsics,
both wrist RGB models, and the alignment between the two arm bases.
`examples/xarm-wuji-bimanual.json` selects it with every camera verified.

| Role | Serial | Fresh maximum position residual | Fresh maximum angle residual |
| --- | --- | ---: | ---: |
| Left wrist | 108322071838 | 4.06 mm | 0.402° |
| Right wrist | 752112070450 | 4.48 mm | 0.430° |
| Side A | 263322070113 | 1.81 mm | 0.432° |
| Side B | 262422074331 | 0.62 mm | 0.153° |

All 76 fresh checks passed the existing 5 mm / 1° pose-consistency limits and
1-pixel mean reprojection limit. The largest mean reprojection error was
0.743 pixels. These are board-pose consistency residuals, not independently
measured absolute robot positioning accuracy. Depth scale and RGB-depth
registration remain unqualified; no depth pixel-to-position endpoint is exposed.

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

After installation, fresh MCP observations in both arm base frames passed all
eight camera checks with the normal runtime image pipeline. Equivalent points
projected to the same reported pixels from either base. All four views reported
`rgb_extrinsics_verified`; the resumed session has no unverified-camera overrides.
That evidence is saved as `runtime-verification.json` in the capture directory.

## Files and conventions

`T_A_B` maps column-vector points from B to A, with translations in metres.

- `wrist_*_T_cam_tcp.yaml` and `wrist_*_T_cam_flange.yaml`: each wrist camera
  relative to its owning arm's TCP and flange.
- `side_*_T_cam_{left,right}_base.yaml`: each side camera relative to both bases.
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

The prior bundle and README are preserved under `bundle-before/` and
`README-before.md` in that directory. Repository `side_a-recalibration-report*.json`
and `side_b-recalibration-report.json` are historical evidence for earlier
side-only adjustments; they do not describe this new full calibration.

See [the operator workflow](../../docs/xarm-wuji-bimanual-calibration.md).
