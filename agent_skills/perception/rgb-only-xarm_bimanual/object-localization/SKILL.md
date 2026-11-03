---
name: object-localization
description: Locate objects using two side and two wrist RealSense views, with explicit left/right base frames and the session's RGB calibration. Use before choosing bimanual approach, grasp or placement targets.
---

# Bimanual RGB perception

The views are `{{ robot.cameras | join('`, `') }}`. `xarm_observe` returns images in
`images_in_order` and saves `runtime/frames/<frame_id>/meta.json`. Each camera entry
has its serial, mount, image dimensions and capture timestamp; a wrist entry also
identifies its `arm`. Both arms' capture-adjacent states are recorded separately.

{% if robot.rgb_calibrated %}
Verified RGB extrinsics and matching color models are loaded. Wrist images are
rectified; their saved `K` describes the saved image. Metadata contains
`T_left_base_cam` and `T_right_base_cam` for every view, and `T_base_cam` in the
selected observation base (`xarm_observe(arm="left" | "right")`, default left).
Each wrist transform uses its owning arm's measured pose immediately after capture.
Camera exposures and arm telemetry are not hardware synchronized. Use settled
observations for triangulation; `arms_before`, `arms` and
`arm_moved_during_capture_mm` help identify motion during the capture interval.
`calibration.T_left_base_right_base` maps right-base points into the left base.
Arm motion commands always take coordinates in the selected arm's own base.

Triangulate the same visible object point across two views using their saved
intrinsics and transforms in the same base. Alternatively intersect a camera
ray with a known support plane. The supplied `reference_board_planes` describe
the original calibration board; confirm that the current support surface matches
before using them. A bottle's cap is elevated above its supporting surface.

Check estimates with fresh `xarm_observe(arm=..., project=[[x,y,z], ...])` views;
compare the red crosses against the object in all useful cameras. Coordinates
in `projected_pixels` and saved images are native pixels. Divide displayed
preview coordinates by that view's `preview_scale` before using saved `K`.
Avoid triangulating mismatched features or nearly parallel rays. The calibration
has millimetre-scale residuals; do not treat an estimated surface as exact contact.

Depth scale and RGB-depth alignment have not been qualified. The backend does
not expose `xarm_pixel_to_position`; use RGB geometry for this session.
{% else %}
Calibration is pending. No `T_base_cam`, inter-arm transform or calibrated table
plane is supplied. `xarm_pixel_to_position` and the observe `project` parameter are
not exposed. Do not reuse single-arm calibration files or assume an identity or
mirror transform. Even an rgbd session's depth is in camera coordinates, not either
robot's base frame. Invalid depth does not imply free space.

Use the side views to inspect both arms and the shared scene, and each wrist view
for its own hand's local geometry. Compare fresh views to assess occlusion,
retention, release and clearance. Do not convert a pixel into a metric arm target
without a measured transform. A clearer image alone does not establish that
transform. If the next action requires metric localization unavailable here,
request calibration or a specific operator-provided measurement.
{% endif %}

`xarm_status.calibration_status` must report `rgb_extrinsics_verified` before
using calibrated transforms. Its calibration summary identifies the loaded bundle.
The operator owns recalibration and depth-alignment validation; calibration files
outside the workspace are not agent tools.

Keep object/hand observations separate by side and serial in memory. Moving one
wrist invalidates that view's previous pose relationship; a different camera or
mount also invalidates calibration. See [wrist-camera-inspection](../wrist-camera-inspection/SKILL.md)
for bounded viewpoint changes and [robot-interface](../robot-interface/SKILL.md)
for the two independent arm base frames.
