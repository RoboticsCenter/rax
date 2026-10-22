---
name: object-localization
description: Metric 3-D positions of visible object points in the xArm base frame from calibrated RealSense RGB-D cameras. Use before choosing any approach, grasp or placement target.
---

# Object localization (RGB-D)

Three calibrated Intel RealSense D435i cameras: `left` and `right` are fixed side views,
`wrist` rides on the hand. Every `{{ robot.observe }}` call saves a frame under
`{{ vocab.frames }}<frame_id>/`: `<camera>.jpg`, `<camera>_depth.png` (uint16
millimetres, 0 = invalid) and `meta.json` with each camera's intrinsics `K` and pose
`T_base_cam` (4x4, camera to base) at capture time.

## Procedure

1. Observe now; never reuse pixels from an earlier frame after anything moved.
2. Find the object in the full-resolution image. Tool previews are downscaled by
   `preview_scale`; divide preview pixels by it.
3. Call `{{ robot.pixel }} {frame_id, camera, pixels: [[u, v], ...]}` for several pixels
   on the object's visible top surface. It returns base-frame points from the median
   depth around each pixel.
4. Cross-check: the same object from another camera should agree within about 1-2 cm.
   A point on the table should match the measured table height in the current setup.
   Disagreement usually means a bad pixel on an edge, shadow, reflection or occluder.
5. Object centre: average interior top-surface points in x, y; height = their z. For a
   grasp, place the TCP at the centre and choose a z that puts the fingers around the
   object's middle, not below the table.

Depth is unreliable on object edges, shiny or transparent surfaces and closer than the
configured camera `depth_min_m`. Use the wrist camera for close inspection after moving
above the object.

To check a target before moving, pass it to `{{ robot.observe }}` as
`project: [[x, y, z]]`; it is drawn as a red cross on every image.

For bulk processing in Python, load the PNG with OpenCV (`cv2.IMREAD_UNCHANGED`) and
back-project with `K` and `T_base_cam` from `meta.json`.
