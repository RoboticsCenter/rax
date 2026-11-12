---
name: screw-lightbulb
description: Pick up, orient, hand over, align and screw a lightbulb into a lamp using Mooncake's learned Dex and Dexmate procedures as references. Includes bulb perception, lamp bracing, thread entry, tightening, grasp search and recovery, with adaptation guidance for xArm and Wuji hands.
---

# Screw a lightbulb with xArm and Wuji

Use this skill for a bulb-installation task. It supplies actual Mooncake procedure
code and evidence, plus the translation needed to plan on this robot. Read
[robot-interface](../robot-interface/SKILL.md),
[hand-control](../hand-control/SKILL.md) and
[object-localization](../object-localization/SKILL.md) for the current tools,
hand profiles and calibration. Those contracts govern execution.

## Choose the relevant source procedure

The [catalog](references/catalog.md) points to 52 relevant skills/methods and their
dependencies in the session's external Git libraries, `skill-library/dex/` and
`skill-library/dexmate/`. Read only the packages needed for the current stage.
Each retains its original `SKILL.md`, `code/` and evidence. These are reference
implementations, not directly runnable RAX controllers. Each library starts at a
pinned source commit on a local `session` branch; edits remain in this session.

| Current problem | Read in the catalog |
| --- | --- |
| Bulb lying on a table; two hands available | Dexmate `screw-bulb-pickup-handover-press`, `pickup-bulb-housing-left`, `release-bulb-left-to-right`, `brace-upright-lamp-left` |
| Bulb upright in a cup | Dex `install-bulb-from-cup`, `extract-upright-bulb-from-cup`, `pick-and-screw-bulb` |
| Locate bulb axis, cap and socket using RGB | Dexmate `measure-bulb-terminal-stereo`, `measure-upright-socket-mouth`, `track-held-diffuser-contour`, `fit-sphere-visible-arcs`, `estimate-coaxial-direction-from-image-line` |
| Entry and tightening | Dex `install-bulb-with-short-turns`, `install-bulb-with-short-entry`, `enter-e27-by-wrist-compliance`, `enter-e27-with-observable-backspin`, `enter-e27-with-retained-grasp`, `tighten-upright-bulb` |
| Lost alignment or changing grasp | Dex `level-free-bulb-by-visible-cap`, `recover-supported-bulb-thumb`, `rebalance-free-bulb-index`; Dexmate `recover-held-wrist-joint-margin` |
| A difficult multi-finger grasp needs search | `search-firm-grasp` and `retarget-grasp-from-demo` methods; Dex `grasp-round-object-by-search` |

The verified Dexmate bulb procedure covers pickup, handover and lamp bracing
**only**: it explicitly does not seat or screw the bulb. The Dex installation
variants are drafts with mixed outcomes, including release failures. Read their
failure modes and evidence before choosing a variant; a newer version is not
automatically more successful. Source `verified` does not establish success on
xArm/Wuji. The Dexmate `in-hand-localization` method is a scaffold with a missing
`dexmate-preview-model` dependency; it is not executable even on its source setup.

## Translate before executing

- Use the live camera inventory and per-camera calibration status. The configured
  views are `{{ robot.cameras | join('`, `') }}`. On the four-camera bimanual rig,
  `wrist_left` and `wrist_right` move with their arms; `front` and `side_right`
  are the two fixed side views (`front` retains its existing camera ID).
  Use fixed views for scene context and separated, verified views for cap/socket
  geometry. An unverified view can supply images but not trusted base-frame poses.
  A source `head_camera` is not simply an alias for a RAX camera: intrinsics,
  extrinsics, image dimensions and observation time differ. In RGB-D sessions,
  configured views also supply aligned depth PNGs; follow object-localization
  for their units, validity checks and factory depth-calibration limitations.
- Source Sharpa/Dexmate finger targets, palm offsets, joint counts, effort scales,
  FK/IK models and workspace coordinates do not transfer to Wuji. Recompute the
  grasp for the selected hand, actual bulb and current pose. Read each hand's
  active/passive joints from status. A partial hand cannot use contact completion,
  effort changes or impedance changes; preserve its passive joint targets and
  commissioned settings as specified by hand-control.
- Express poses in the selected xArm base frame using the verified inter-arm
  transform when needed. RAX pose quaternions are `xyzw`; inspect and convert the
  source convention explicitly. Do not replay source poses, neutral positions,
  tabletop heights or bulb dimensions as measurements of this scene.
- Source tactile pad forces and wrist wrench feedback are not supplied by the
  current RAX hand contract. Motor current or tracking error is not a calibrated
  substitute for either. Keep sensor-dependent controllers conditional: port one
  only with its required observations and checks available. Do not remove its
  force/engagement guard merely to make it run. An RGB-guided alternative is a new,
  unvalidated controller, with its own observable acceptance and stopping criteria.
- Preview-based grasp search requires a qualified Wuji model and adapter. The
  bundled Dex model and the Dexmate scaffold are source references, not this
  robot's collision or grasp model. Grasp-family selection and uncertainty-aware
  evaluation remain useful even when that simulator cannot be run here.
- Use current MCP calls with explicit arm/hand selectors on bimanual setups.
  `dex_*`, `dexmate_*`, source `Context`, simulation truth, scoring APIs and
  `skill_library_merge` are not RAX services. Loading these libraries does not add
  those services. Port task-specific code into the workspace's `code/` after
  reviewing its dependencies; retain the source library as provenance.

## Apply the learned sequence to the observed scene

1. Observe bulb and lamp, identify cap/socket type and the available grasp surfaces,
   and estimate both axes and the socket rim. E27/A60 source assumptions apply only
   if the observed parts match. Choose pickup or cup extraction accordingly.
2. Establish a retained grasp and verify object follow with fresh views before
   transport. A commanded closure alone does not prove retention. If a handover is
   needed, verify the receiving hand holds the bulb before releasing the donor.
   Choose hand roles from current reach and usable fingers, not source left/right
   labels. With one arm, use an already stable fixture instead of a two-hand plan.
3. Stabilize the lamp when needed, align the measured cap and socket axes, and
   preserve room for wrist turning and regrasping. Use normal clear-space motions
   and controlled observed increments near contact. Reobserve after a slip, handover
   or regrasp because the bulb-to-hand transform may have changed.
4. Treat first contact, rim support, thread engagement and final seating as distinct
   states. Source entry/backspin strategies are candidates, not proof of capture.
   Check axial advance and alignment during bounded turns; reject visible tilt,
   lamp motion, slip or lack of progress before applying further tightening.
   If the observations needed to distinguish these states are unavailable, stop
   that stage and report the missing evidence instead of declaring success.
5. Release only after support is established. Verify the bulb remains seated and
   aligned with the hands clear; do not equate a turn count, apparent contact or
   increased motor current with fully seated threads. Record what was actually
   demonstrated, including failures, in task memory via
   [experience-memory](../experience-memory/SKILL.md).

The [source references](references/sources.json) pin the two Mooncake library
commits; the launcher records their checked-out revisions in the workspace manifest.
Original statuses, dependencies and evidence remain in each library. No procedure
has been validated on this physical xArm/Wuji setup by this transfer.
