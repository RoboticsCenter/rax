---
name: wrist-camera-inspection
description: Handle requests for close-up views when this single-Piper setup has no commissioned wrist camera.
---

# No commissioned wrist camera

This session has only the fixed overview RGB camera. Moving the arm will not
create a new camera viewpoint. Do not move the arm or release an object to obtain
a nonexistent wrist image. Call `piper_observe`; if the view is insufficient,
ask the operator for a clearer safe view or additional calibrated equipment.
