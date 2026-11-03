---
name: object-localization
description: Inspect two side and two wrist RealSense views while bimanual camera calibration is pending. Distinguish image evidence and camera depth from unavailable robot-base coordinates.
---

# Bimanual perception before calibration

The views are `{{ robot.cameras | join('`, `') }}`. `xarm_observe` returns images in
`images_in_order` and saves `runtime/frames/<frame_id>/meta.json`. Each camera entry
has its serial, mount, image dimensions and capture timestamp; a wrist entry also
identifies its `arm`. Both arms' capture-adjacent states are recorded separately.

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

Keep object/hand observations separate by side and serial in memory. Moving one
wrist invalidates that view's previous pose relationship; a different camera or
mount also invalidates calibration. See [wrist-camera-inspection](../wrist-camera-inspection/SKILL.md)
for bounded viewpoint changes and [robot-interface](../robot-interface/SKILL.md)
for the two independent arm base frames.
