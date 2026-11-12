---
name: object-localization
description: Locate objects using configured fixed and wrist RealSense views, available RGB-D observations, and explicit left/right base frames. Use before choosing bimanual approach, grasp or placement targets.
---

# Bimanual perception

The views are `{{ robot.cameras | join('`, `') }}`. `xarm_observe` returns images in
`images_in_order` and saves `runtime/frames/<frame_id>/meta.json`. Each camera entry
has its serial, mount, image dimensions and capture timestamp; a wrist entry also
identifies its `arm`. Both arms' capture-adjacent states are recorded separately.
Fixed views: `{{ robot.fixed_cameras | join('`, `') }}`; wrist views:
`{{ robot.wrist_cameras | join('`, `') }}`. Use only configured view names.
Image dimensions may differ between cameras; read each image's metadata.

{% if observation_profile == 'rgbd' %}
## Depth from every camera

`xarm_observe` saves a full-resolution RGB image and aligned depth PNG for each
requested camera. Read the filename in `meta.json` → `cameras[name].depth` with
`cv2.imread(path, cv2.IMREAD_UNCHANGED)`. Values are **uint16 millimetres**; divide
by 1000 for metres. Zero is invalid, not free space. Do not apply `depth_scale_m`
again: the camera service already converted raw sensor units before saving.

Depth pixels share the saved RGB image's grid, dimensions and `K`, including
rectification where configured. Use native coordinates, not resized preview
coordinates. Read `depth_alignment`, `depth_registration_source`, and
`depth_scale_factor` to identify the alignment/scale actually used. Inspect valid
depth in a small patch on the object, excluding background, edges and holes;
reflective metal and translucent bulb surfaces can yield missing or false depth.

For an undistorted pixel `(u,v)` and valid depth `z` metres, camera coordinates are
`[(u-cx)*z/fx, (v-cy)*z/fy, z]`; otherwise undistort the ray using the saved model
first. A verified `T_base_cam` maps that point into the selected arm base. The
depth scale/registration currently uses factory calibration, separately from the
verified RGB camera-to-arm calibration. Cross-check depth estimates against
settled multi-view geometry or known surfaces before precise contact motions.
There is no `xarm_pixel_to_position` convenience tool in this bimanual backend.
{% else %}
This session captures RGB only; depth files are not available.
{% endif %}

{% if robot.rgb_calibrated %}
RGB extrinsics and matching color models are loaded. Read each view's
`calibration_status`: only `rgb_extrinsics_verified` supplies calibrated geometry.
Use the saved image's `K` and `distortion_coeffs`; the backend applies the loaded
color model and may rectify wrist images. Do not substitute factory intrinsics
for a rectified image. Metadata contains
`T_left_base_cam` and `T_right_base_cam` for each verified view, and `T_base_cam` in the
selected observation base (`xarm_observe(arm="left" | "right")`, default left).
Each wrist transform uses its owning arm's measured pose immediately after capture.
Camera exposures and arm telemetry are not hardware synchronized. Use settled
observations for triangulation; `arms_before`, `arms` and
`arm_moved_during_capture_mm` help identify motion during the capture interval.
`calibration.T_left_base_right_base` maps right-base points into the left base.
Arm motion commands always take coordinates in the selected arm's own base.

Triangulate the same visible object point across two views using their saved
intrinsics and transforms in the same base. {% if robot.fixed_cameras | length >= 2 %}
The configured fixed views `{{ robot.fixed_cameras | join('`, `') }}` can form a
stationary stereo pair when both have verified geometry and see the same feature.
Use wrist views for closer inspection and additional viewing angles.
{% else %}With one fixed view, use it with a
wrist view or use both wrist views when the same feature is visible; do not
assume a second fixed camera exists.{% endif %} Alternatively intersect a camera
ray with a known support plane. The supplied `reference_board_planes` describe
the original calibration board; confirm that the current support surface matches
before using them. A tilted calibration board does not define the tabletop.
Moving or removing the board after calibration does not change the camera-to-arm
transforms. A bottle's cap is elevated above its supporting surface.

Check estimates with fresh `xarm_observe(arm=..., project=[[x,y,z], ...])` views;
compare the red crosses against the object in all useful cameras. Coordinates
in `projected_pixels` and saved images are native pixels. Divide displayed
preview coordinates by that view's `preview_scale` before using saved `K`.
Avoid triangulating mismatched features or nearly parallel rays. Read the loaded
bundle's verification summary for its measured residuals; these are consistency
checks, not a guarantee of absolute positioning accuracy or exact contact.

After operator recalibration, read fresh status and observations and record the
new `calibration.sha256`. Re-localize task objects with the newly supplied camera
models and base alignment. Previous frame metadata remains valid for its original
capture; do not mix it with images or targets computed under the new bundle.

The verification above qualifies RGB geometry. It does not independently qualify
depth scale or RGB-depth registration; use the observation profile's depth guidance
above when depth is enabled.
{% else %}
Calibration is pending. No `T_base_cam`, inter-arm transform or calibrated table
plane is supplied. `xarm_pixel_to_position` and the observe `project` parameter are
not exposed. Do not reuse single-arm calibration files or assume an identity or
mirror transform. Even an rgbd session's depth is in camera coordinates, not either
robot's base frame. Invalid depth does not imply free space.

Use the configured fixed view(s) to inspect both arms and the shared scene, and each wrist view
for its own hand's local geometry. Compare fresh views to assess occlusion,
retention, release and clearance. Do not convert a pixel into a metric arm target
without a measured transform. A clearer image alone does not establish that
transform. If the next action requires metric localization unavailable here,
request calibration or a specific operator-provided measurement.
{% endif %}

`xarm_status.calibration_status` reports `rgb_extrinsics_verified` when all views
are verified, or `rgb_extrinsics_partial` when specific views have moved or an
additional or replacement camera has not yet been calibrated. Its calibration summary lists
per-camera status and the bundle.
An `extrinsics_unverified` view remains useful for visual inspection but supplies
no base transform or projected marks. Use two suitable verified views for metric
localization; an unverified fixed view does not require stopping otherwise supported
work. Do not recover its old transform from earlier captures after the mount moved.
A replacement camera also has its own optical model. Matching a view name such
as `front` is insufficient: compare serial, image profile and calibration identifier
before reusing observations or geometry.
The operator owns recalibration and depth-alignment validation; calibration files
outside the workspace are not agent tools.

Keep object/hand observations separate by side and serial in memory. Moving an
arm changes its wrist camera's pose in the base, while the rigid camera-to-TCP
calibration remains applicable. Read that wrist's fresh capture-adjacent transform.
A physical camera-mount or robot-base shift requires the affected geometry to be
checked. See [wrist-camera-inspection](../wrist-camera-inspection/SKILL.md)
for bounded viewpoint changes and [robot-interface](../robot-interface/SKILL.md)
for the two independent arm base frames.
