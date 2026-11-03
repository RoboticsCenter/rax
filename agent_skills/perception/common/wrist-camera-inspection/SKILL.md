---
name: wrist-camera-inspection
description: Obtain a clearer close-up of small or occluded task details, or a second viewpoint, by positioning the wrist camera with an empty end effector when the {{ robot.overview }} is insufficient. Use object-localization on the new observation.
---

# Wrist-camera inspection

Use the arm as a movable inspection camera. Bringing the wrist camera closer and
adjusting the viewing angle can reveal details that are unclear in
`{{ robot.overview_camera }}`. This is physical camera repositioning, not digital zoom.

- Do this with an empty end effector. Do not release a held object just to obtain a view.
- Use the exterior cameras to choose a clear approach and keep the end effector well clear
  of the table and objects. Use a complete supported move through verified clear
  space; use smaller steps to resolve uncertain geometry near objects.
- Inspect the fresh `{{ robot.wrist_cameras | join('` or `') }}` image after each move. Adjust distance and angle until the
  relevant detail is clear, avoiding end-effector occlusion.
{% if robot.bimanual %}
- Select `arm="left"` or `arm="right"` explicitly, and inspect that arm's wrist
  camera. Check the other arm, hand, attached camera and any object spanning the
  hands before repositioning.
{% if robot.rgb_calibrated %}
  The four-camera RGB calibration is loaded; each wrist uses its owning arm's
  measured pose. Obtain a fresh observation after repositioning.
{% else %}
  Extrinsics are pending; a clearer view alone does not establish metric geometry.
{% endif %}
{% endif %}
{% if camera_geometry or robot.rgb_calibrated %}
- Use [object-localization](../object-localization/SKILL.md) on the new observation.
  Use the new frame's pixels and capture-adjacent `T_base_cam`. Moving the arm changes
  the camera's base-frame pose, not its fixed mount calibration. Intrinsics and
  rigid mounting extrinsics remain applicable while the physical mounts and robot
  base are unchanged.
{% else %}
- Use [object-localization](../object-localization/SKILL.md) on the new observation;
  do not reuse old wrist images after moving the camera.
{% endif %}
{% if robot.has_hand %}
- Check the palm, every finger and the attached camera's sweep when changing view;
  clear TCP motion alone does not establish whole-hand clearance.
- At close range the hand can clip the wrist image or lie below valid depth range.
  Use a fresh exterior view, RGB silhouettes or a small clear-space viewpoint change
  to resolve the relevant uncertainty. Invalid depth is missing evidence, not free space.
- The operator can improve illumination without moving calibrated camera mounts.
  A mount/base change requires the calibration relationship to be checked again.
{% endif %}

Move only when the clearer view resolves a task-relevant uncertainty, then return to a
clear pose if the arm obstructs the next manipulation.
