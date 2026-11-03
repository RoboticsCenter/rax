# Verified bimanual RGB extrinsics

Captured and verified 2026-11-02 Pacific time (2026-11-03 UTC).
Status: **verified candidate; not yet loaded by the bimanual backend**.
Depth scale, RGB-depth registration and runtime pixel-to-base integration remain
separate work. The backend still reports calibration pending.

| Role | Serial | Fresh maximum position residual | Fresh maximum angle residual |
| --- | --- | ---: | ---: |
| Left wrist | 108322071838 | 4.75 mm | 0.444° |
| Right wrist | 752112070450 | 3.49 mm | 0.518° |
| Side A | 263322070113 | 1.30 mm | 0.131° |
| Side B | 262422074331 | 2.22 mm | 0.268° |

These are board-pose consistency residuals, not independently measured absolute
robot positioning accuracy. All 76 fresh checks passed the unchanged 5 mm / 1°
limits and the 1-pixel mean reprojection limit. Maximum mean reprojection error
was 0.772 pixels. Verification used a frozen fit, one home capture, and three
new poses per arm with three captures per pose. All four cameras were checked
in every capture, including the stationary wrist. Both arms returned to their
starting poses. Finger targets were unchanged throughout calibration.

## Files and conventions

`T_A_B` maps column-vector points from B to A, with translations in metres.

- `wrist_*_T_cam_tcp.yaml`: each wrist camera from its owning arm's TCP.
- `wrist_*_T_cam_flange.yaml`: equivalent transforms from each flange.
- `side_*_T_cam_{left,right}_base.yaml`: each fixed camera from either arm base.
- `T_left_base_right_base.yaml`: right base into left base.
- `*_color_model.json`: the exact 1280×720 @ 6 FPS RGB models used by these fits.
  Wrist files include the recorded factory profile and fitted radial distortion;
  side views retain factory intrinsics. Runtime profile conversion must be
  validated before applying them at a different resolution.
- `board.json`: 10×13 physical squares, 20 mm squares, 15 mm markers, with the
  verified legacy 14-row ID layout and its final physical row absent.
- `manifest.json`: camera/arm identities, TCP offsets, models, fit residuals,
  source images and fresh-verification summary.
- `verification-report.json`: every fresh observation, residual, source-fit hash
  and measured return pose.

Left arm: `192.168.2.205`, serial `XI130510J56A06`.
Right arm: `192.168.1.241`, serial `XI130505F56A0A`.
Both controller TCP offsets were `[0, 0, 172, 0, 0, 0]` in mm/radians.
Moving a mount, replacing a camera, changing TCP offsets or moving an arm base
requires checking the corresponding transforms again.

## Method and provenance

The original single-arm 18-pose workflow was adapted to sequential bimanual
sweeps against one stationary board. The initial sweep clipped the board in
some wrist views. A second sweep selected fully framed views within the same
translation/rotation bounds; all 18 views per arm were accepted. Each hand-eye
fit used 15 training poses, reserving three poses per arm.

Free RGB intrinsic refinement shifted focal lengths substantially. A constrained
radial k1/k2 fit retained factory focal lengths and principal points. Lens
training combined non-reserved images from the initial and well-framed sweeps
to cover image edges as well as the center; no reserved or verification images
were included. All other distortion coefficients are zero. Hand-eye fitting
used only the second sweep's training poses. Fresh verification followed model
selection and did not alter the fitted parameters or discard observations.

Artifacts, including unsuccessful diagnostic fits, remain under:
`/home/jerry/rax-runtime/bimanual-extrinsics-20261103T045627Z`.
The accepted fit is `pooled-distortion-fit/result.json`; independent verification
is `verification-pooled/verification.json`.

See [the operator workflow](../../docs/xarm-wuji-bimanual-calibration.md) for
collection, fitting, optical-training and verification commands.
