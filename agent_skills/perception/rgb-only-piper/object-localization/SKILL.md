---
name: object-localization
description: Locate objects conservatively from the fixed overview RGB camera without pretending uncalibrated pixels provide metric robot coordinates.
---

# RGB-only Piper localization

Call `piper_observe` for current evidence. Only the fixed `overview` camera is
configured. There is no wrist camera, depth image, calibrated camera-to-base
transform, or pixel-to-position tool in this port. Never borrow Franka/xArm
calibration or infer a safe metric motion from a single uncalibrated pixel.

Ask the operator for calibrated/taught waypoints and clearance information before
planning metric approaches. Use RGB to check scene identity, contact, obstruction,
slip and task outcomes. Refresh observations after movement or a pause. Preserve
decisive frames and distinguish observed success from an untested guess.
