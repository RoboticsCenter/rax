---
name: hand-control
description: Native BrainCo Revo 2 motor targets, measured feedback, bounded effort/speed and tactile observations.
---

Revo has six motor channels ordered **thumb flex, thumb rotation, index, middle,
ring, pinky**. Position is normalized 0..1000 (larger closes), not radians.
Speed and current are signed normalized vendor units, not rad/s or amperes.
Read `revo_status` for the actual serial, firmware, limits, motor states and postures.
{% if robot.bimanual %}Pass `arm="left"` or `arm="right"` to every Revo call.{% endif %}

`revo_move(positions=[...], duration_ms=...)` uses the native six-motor
position-and-duration API. Alternatively supply six positive `speeds` for native
position-and-speed control. Do not supply both timing forms. Targets outside the
configured envelope or step bound are rejected rather than clamped. Split large
changes into small moves, inspecting feedback and the scene between them.

The default completion requires measured position and stationary feedback.
`completion="contact"` also accepts stationary obstruction and reports
`reached=false, outcome="stationary_obstruction"`. It does not establish contact
location, calibrated force, grasp quality or retention; check images and tactile
feedback where available. Motor state 2 is a stall indication, not a force reading.

`revo_pose(name=...)` uses an operator-defined posture; `revo_home` requires an
explicit `home` posture. Neither runs firmware calibration nor changes protections.
Support and release the object before home. Retreat with sufficient clearance, then
return the appropriate arm home. Repeat for each arm in a bimanual session, accounting
for the other hand. Never force a home move from an exception handler.

`revo_set_speeds` and `revo_set_currents` appear only when enabled by configuration.
They accept six signed native values and a finite `duration_ms`. Positive closes;
negative opens. They end in measured-position hold, including on disarm. Current
does not persist across subsequent arm moves. There is no MIT kp/kd or feed-forward
Wuji effort mapping. `revo_hold` cancels effort/speed and holds measured positions;
this may release a held object. Whole-rig `xarm_hold` additionally stops both arms.

`revo_touch` appears only for a configured Touch device. It reads native fingertip
data and the enabled mask without enabling/calibrating sensors. Disabled or faulted
sensors are not evidence of zero contact. Interpret native units using the device's
commissioned sensor calibration.
