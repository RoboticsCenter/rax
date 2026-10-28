---
name: object-localization
description: Locate visible object points in the xArm base frame from calibrated RGB cameras without depth. Use before choosing any approach, grasp or placement target.
---

# Object localization (RGB only)

This session has no depth. Each `{{ robot.observe }}` frame saves RGB images and a
`meta.json` with each camera's intrinsics `K` and pose `T_base_cam`.

- Triangulate: find the same object point in two configured camera images
  (`{{ robot.cameras | join('`, `') }}`) and
  intersect the two camera rays (from `K` and `T_base_cam`) in your own Python.
- Or intersect one ray with a known plane, such as the table top for a point on the
  table, or the table plus a known object height.
- Check every estimate by projecting it back with `project: [[x, y, z]]` on a new
  `{{ robot.observe }}` and comparing the red cross with the object.
{% if robot.has_hand %}

Contact offsets depend on finger posture and wrist orientation. A table-plane
intersection is valid for a point on the table, not an elevated finger or object
surface. Compare nominal geometry with fresh multi-view evidence and use small
clear-space probes from [hand-control](../hand-control/SKILL.md) to refine uncertain
offsets. Track an uncertainty margin and the whole-hand sweep before approaching.
{% endif %}
