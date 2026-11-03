---
name: robot-interface
description: Control two real xArm 6 arms with left/right Wuji Hand 2, explicit side selectors, paired moves and shared stop handling. Read before moving either assembly or writing robot code.
---

# Bimanual xArm + Wuji interface

Two independent six-joint arms each carry a twenty-joint Wuji Hand 2. Read
`xarm_status` for `arms.left`, `arms.right`, the IPs, profiles, workspace limits and
shared arming state. Cameras are `{{ robot.cameras | join('`, `') }}`.

## Select the device and its frame

- `xarm_move_pose(arm="left" | "right", position=[...], ...)` and
  `xarm_move_joints(arm="left" | "right", positions=[...] | preset="home")`
  require an explicit arm. There is no default side.
- Every `wuji_*` single-hand tool requires `hand="left" | "right"`, including
  `wuji_status`, `wuji_hold` and `wuji_home`. Read [hand-control](../hand-control/SKILL.md).
- Metres, radians and quaternions `[x,y,z,w]`. Each arm uses its **own base frame**:
  x forward, y left, z up. `relative=true` adds a displacement in that arm's base
  axes, not the wrist axes. {% if robot.rgb_calibrated %}Observations include the
  measured `T_left_base_right_base`; use it to convert points between bases.
  {% else %}The arms do not yet have a calibrated shared frame.
  {% endif %}Do not copy an absolute pose from one arm to the other or infer a mirror transform.
- The controller TCP is not a fingertip position. Both hands' posture, palm,
  fingers and wrist-mounted cameras contribute to their swept volume.

## Single and paired moves

Single-arm motion retains the original xArm Cartesian/joint planner, workspace/FK
checks, speed caps, controller error checks, progress detection and measured
settling. Read each side's live limits. Cartesian `speed_mps` selects a lower speed
for that call; `timeout_s` is a maximum wait, not a required duration.
An explicitly configured partial hand does not by itself prohibit arm movement.
Its excluded joints remain passive; use `active_enabled` and the loaded isolation
policy when interpreting hand readiness.

Paired calls accept separate `left` and `right` objects with the ordinary payload:

```python
ctx.value("xarm_move_both",
          left={"position": [0, 0, 0.01], "relative": True},
          right={"position": [0, 0, 0.01], "relative": True})
```

This illustrates the schema, not a scene-qualified movement. `xarm_move_joints_both`
accepts each side's joint payload; `wuji_move_both` accepts each hand's twenty-joint
target, optional duration and contact-completion policy. Both targets are validated
before either starts. Results are under `results.left` and `results.right`.
Each hand still follows its own completion policy: a partial hand requires
position completion even when the full-control hand uses contact completion.

The controllers start concurrently but are not hardware synchronized. A common
arrival time, coupled trajectory, mutual collision check, or object-force balance
is not guaranteed. Check both sweeps, including the other arm and any object or
lace spanning the hands. Use sequential moves when their interaction is uncertain.
Use paired tools for concurrency; separate MCP requests are serialized.

## Readiness, faults and stop

Both assemblies must have fresh readiness under their loaded profiles before a mutating call. Each
hand retains its own limits, gains, current bias, warning policy and postures.
Use each hand's loaded profile. The operator may explicitly share tested gains
and the exact encoder-warning allowance; saved postures remain hand-specific.
Configured passive joints and isolated diagnostic codes do not by themselves
block supported control; a partial hand uses `active_enabled` for readiness.

An operator reply and preparation are separate state changes. The dashboard's
**Ready & send reply** prepares/reuses both assemblies and renews arming before
replying; **Send reply only** does not. Recheck status and fresh observations after
either reply. Preparing a disabled hand seeds measured angles, restores its
profile parameters and zeros current bias; renewing a healthy hold preserves it.

Either controller fault or loss of owned hand control disarms the session and
requests stops across both arms and both hands. A paired execution failure can
leave partial movement; inspect per-side results, measured state and fresh views.
Do not replay both targets just because only one side failed.

`xarm_hold()` requests and verifies four independent stops. `assembly_stop` names
`left_arm`, `left_hand`, `right_arm`, `right_hand`. Uncertainty clears only when all
four stops are confirmed. This disables the hands and may release an object. It
does not clear errors, ready the arms or prepare the hands. Operator direct
stop/disarm bypasses the MCP queue; another MCP hold waits behind the active call.

`wuji_hold(hand=...)` retains that hand's healthy commanded posture and current
bias; it is not an assembly emergency stop. Read its result and current health.

## Four views and coordinate frames

`xarm_observe(cameras=[...])` saves observations and both assemblies' state.
`xarm_stream` reviews the rolling history. Wrist camera metadata identifies its
owning arm. {% if robot.rgb_calibrated %}RGB models and transforms are loaded.
`xarm_observe(arm="left" | "right", project=[[x,y,z], ...])` projects points in
the selected base. Saved metadata supplies camera poses in both bases; arm
motion tools still require targets in their own base. Depth calibration remains
pending and no depth pixel-to-position tool is exposed.
{% else %}Factory intrinsics and optional depth do not provide a camera-to-arm
transform. Pixel-to-base lookup and projected base-frame points are unavailable.
{% endif %}
Use [object-localization](../object-localization/SKILL.md) for the current limits.
