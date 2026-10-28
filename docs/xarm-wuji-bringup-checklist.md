# xArm + Wuji bring-up checklist

Started 2026-10-27 on `real/xarmwuji`. Checkmarks describe observed results;
unchecked items still need commissioning. The earlier
[live MCP report](wuji-live-test-2026-10-25.md) covers initial hand-only tests.
The [October 27 full backend report](xarm-wuji-live-mcp-2026-10-27.md) covers all
eleven tools on the current setup: xArm checks passed after corrections; the
hand's index-joint positioning trial failed settling and needs follow-up.

**October 28 update:** the integrated v13 controller now passes saved home/fist
motion through RAX with both cameras streaming. Five moves, three home arrivals
and a moving-hand software-disarm test passed; the operator reported smooth,
acceptable motion. See the [live qualification](wuji-rax-qualification-2026-10-28.md).
The older hand-profile failures below are historical evidence. The selected next
task is cube into bowl; the operator will start Claude Code using the
[prepared handoff](xarm-wuji-cube-handoff-2026-10-28.md).

## Remaining before cube contact

- [x] Validate v13 saved home/fist paths, repeated returns from fist and independent
  disarm during moving-finger execution, with production camera/recording load.
- [x] Confirm camera mounts/base unchanged with the operator; recheck live RGB-D
  and scene localization. Cube-top estimates agreed within approximately 2.34 mm.
- [x] Prepare the current cube task and a fresh v13 session with 15 mm/s arm speed,
  50 mm translation limit per call and both calibrated cameras.
- [x] Select the inherited `openai` provider and `openai/gpt-6-astra` model for
  the prepared Claude Code session; document CLI PATH and OpenAI API key setup
  for the official `scripts/start_agent.sh`. No Claude experiment login is needed.
- [x] Operator provides the OpenAI API key and starts Claude Code. Astra reached
  MCP status; workspace reads were blocked by the session's location inside the
  protected source checkout. No motion occurred in that attempt.
- [x] Prepare a corrected session and agent Python environment under
  `/home/jerry/rax-runtime`, outside the protected checkout. Bubblewrap read/write
  and import checks passed while source/private files remained hidden. Add an
  early launcher check for protected ancestors; 23 targeted regressions passed.
- [ ] Operator exits the first session and launches the corrected session from
  the handoff. The new session has not been started.
- [ ] Verify mounted payload/CoG, physical contact reference and swept clearance.
- [ ] Establish cube-specific opening, grasp and release; qualify short-lift
  retention, transport, placement and task-pose resets. Saved home/fist alone
  do not establish these behaviors.

**Camera calibration is installed and verified.** The new accurate print was
used for run06 RGB calibration and run07 infrared/RGB stereo alignment. See the
[calibration report](../calibration/xarm-wuji/README.md) for measurements, files,
coordinate conventions and repeat commands. Final installed MCP median errors
were 3.60 mm (wrist), 1.63 mm (side), and 4.34 mm between cameras. All 102
targeted regression tests passed. The arm returned to start and is disarmed;
the hand is disabled with fresh, fault-free feedback.

## Configuration and hardware

- [x] Integrate the approved standalone S-curve controller as
  [v13](wuji-scurve-2026-10-28.md): local Ruckig 0.19.4, 1 kHz position/velocity
  streaming, Kp 8/Kd 0.15 on all twenty joints, 1.5 A caps, and 0.6/3/2
  radian velocity/acceleration/jerk limits. Trajectory, cancellation, feedback
  guards and private-service behavior are tested without physical motion.
- [x] Validate the integrated v13 profile through RAX on hardware, including the
  existing operator-recorded home and fist. Five commands passed; worst endpoint
  error was 0.038747 rad against the unchanged 0.060 rad gate. The operator
  approved smoothness. The known permitted thumb S2 encoder warning recurred;
  encoder accuracy/root cause remains unqualified. See the live report above.
- [x] Python environment: `.venv/bin/python`, xArm SDK 1.18.4, Wuji SDK
  2026.10.12, RealSense 2.59.1.11053, OpenCV 5.0.0.
- [x] Read arm identity/state at `192.168.100.16`; controller firmware 2.7.1.
- [x] Configure right Wuji `WH2KA01260823017`, endpoint `192.168.1.111:50001`
  after the [2.6.0 → 2.13.1 firmware upgrade](wuji-firmware-upgrade-2026-10-27.md).
  All twenty joints returned disabled/error-free feedback. Restore/readback of
  the 0.5 A limit and Kp 1.5/Kd 0.1 passed through the native SDK and RAX driver.
- [x] Requalify the saved home/fist paths, repeated home returns from fist and
  moving-hand software disarm on firmware 2.13.1 with v13. Two fist→home returns
  differed by at most 0.002310 rad; disabled low-speed feedback followed disarm
  within 61.04 ms in this trial. Broader task poses and other stop modes remain
  open. Most earlier motion tests below used 2.6.0. The historical
  [vendor-profile comparison](wuji-vendor-profile-test-2026-10-27.md) on 2.13.1
  failed initial thumb-home settling at 0.05067 rad with Kp 5/Kd 0.1/1.5 A,
  versus 0.06318 rad on the commissioning profile. Repeated returns did not begin.
  Both trials disabled cleanly and restored the configured profile.
- [x] Identify wrist D435 `262422074331` and fixed side D435 `348522070593`;
  both report firmware 5.15.1.55 and USB 2.1. Runtime is 640×480 at 6 fps;
  calibration RGB capture is 1280×720 at 6 fps.
- [x] Select `wrist`/`side` camera names in generated workspace instructions.
- [ ] Verify the mounted hand's mass, center of gravity, contact reference and
  collision clearance. Readback was 0.82 kg, CoG [0, 0, 48] mm, TCP offset
  [0, 0, 172, 0, 0, 0] mm/rad. These are existing controller settings, **not
  measured Wuji geometry**. Calibration uses the recorded TCP reference.
- [ ] Review workspace limits and finger sweep for actual tasks.
- [ ] Qualify gains/current, all joint directions and full-hand postures. The
  previous [v12 example](wuji-feedback-motion-2026-10-28.md) used Kp 6/Kd 0.15
  on the thumb, Kp 5/Kd 0.15 elsewhere, 1.5 A caps and stable 0.060 rad endpoint
  tolerance. Feedback pacing and matching velocity commands allow extra time;
  the 2.1 rad displacement admission limit allows continuous larger paths.
  Small-motion comparisons and one full arbitrary-pose → fist transition passed,
  with fist error 0.059642 rad on thumb S2. The operator still observed jerkiness;
  smoothness and general task operation remain unqualified. All joints ended
  disabled, with zero native warning or active guard event during these trials.
  The preceding
  [v9 damping retest](wuji-ring-damping-2026-10-28.md) completed one broad
  random-home cycle, then stopped during the second initialization on pinky S2
  error of 0.054443 rad. Ring S1 reached 0.048728 rad at the first random pose.
  No ring oscillation event or native encoder warning occurred; the hand ended
  disabled away from home. The earlier
  [final per-joint trial](wuji-per-joint-profile-2026-10-27.md) passed all nine
  commands, with worst error 0.02223 rad. Speed bursts remain. The earlier
  [uniform-profile test](wuji-uniform-profile-2026-10-27.md) passed eleven
  commands and three home arrivals, but the operator reported jerky movement.
  Tracking success does not qualify smoothness or full task operation.
  A [six-run comparison](wuji-smoothness-2026-10-27.md) tested Kp 8/10 and
  Kd 0.15/0.20 with fixed trajectories and caps. Kp 8 suppressed useful small
  thumb motion; Kd 0.20 gave a modest burst-count reduction at Kp 10. All thirty
  commands passed the existing endpoint gate, with no native diagnostic event,
  but the 0.020 rad excursions are smaller than that gate's 0.040 rad tolerance.
  Small-motion fidelity and visible smoothness remain unqualified.
- [x] Test a lower-gain finger profile with joint-specific exceptions. With
  fingers at 5/0.10, only index S1 repeatedly exceeded the 0.030 rad review
  threshold (0.03220 rad twice); 8/0.15 reduced those errors to about 0.020 rad.
  At the operator's request, lower all thumb joints from 10/0.20 to 8/0.15 and
  complete four whole-hand returns from ±0.06 rad excursions. Save v7 and
  independently verify gains/current caps and all-joint disable. Broader
  arbitrary-pose returns and smoothness remain open for this final profile.
  The subsequent [broad random-pose test](wuji-random-home-2026-10-27.md)
  failed while initializing its first starting pose: outbound step 4 had
  thumb S2 error 0.042127 rad. No random-home cycle completed; the hand was
  disabled away from home, with v7 unchanged and no encoder event.
- [x] Save the operator-selected current hand posture as `postures.home` in
  `examples/xarm-wuji.json`, profile `right-hand2-home-20261027-v2`. Ten fresh,
  fault-free disabled-state samples were captured without enabling or moving.
- [ ] Test repeatable returns to the saved hand home from task poses. It differs
  from the earlier tested return posture by up to 1.00035 rad; the 0.35 rad step
  limit applied to the original trials; v13 uses 2.1 rad. V13 now passes the
  initial near-zero→home and two fist→home returns; cube task poses remain open.
  Use `wuji_pose(name="home")` for an admitted return.
  `prepare` holds measured angles; `scripts/xarm home` resets the arm only.
  The first requested home-return trial stopped on intermediate step 2 of 6:
  thumb S2 had 0.07170 rad settling error against a 0.040 rad tolerance. Home was
  not reached; hand disable was confirmed. See the October 27 report.
  The later [selected-profile retest](wuji-home-retest-2026-10-27.md) also stopped
  before home: thumb S2/S3 and pinky S4 exceeded 0.040 rad at an intermediate
  waypoint (worst 0.045738 rad). No encoder event or oscillation guard triggered;
  the hand was disabled. That older selection affected only ring S1/S3/S4.
  The subsequent uniform profile completed one staged home arrival and two
  ±0.02 rad return checks, with worst home-hold errors of 0.020091, 0.009115 and
  0.009162 rad over all joints. Broader starting poses and smoothness remain open.
- [x] Resolve the cached hand-mode/target race between consecutive MCP moves.
  Admission now reads the private owner's current status and validates against
  that same target snapshot. Stale/unavailable/faulted owners still refuse.
  Regression tests and five consecutive live calls passed without admission
  retries or artificial waits. In the historical
  selected-profile retest, a completed move was followed by an admission
  rejection while status still reflected execution. Waiting for confirmed
  holding was added to that test harness; the production check is now corrected.
- [x] Operator inspected thumb clearance with the hand disabled and reported no
  visible obstruction. Read-only firmware inspection found thumb S2 soft limits
  disabled; this does not rule out internal friction or binding.
- [ ] Resolve thumb S2 stalling and qualify repeatable home before promoting gains.
  The [gain trials](wuji-gain-tuning-2026-10-27.md) tested kp 2–5, including
  lower damping at kd 0.05/0.075, with the 0.5 A cap unchanged. At kp 4, kd 0.05,
  two isolated thumb-home approaches passed with about 0.0284 rad error, but the
  negative probe triggered `0x0007: Stall`. Brief SDK-reported thumb velocity reached
  2.09 rad/s; physical output speed was not independently verified. Whole-hand home
  remains unqualified. Original gains were restored
  and verified with the hand disabled and arm stopped; no tuned profile was promoted.
  A subsequent controlled positive-side comparison at kp 4 and kd 0.05/0.06/0.075
  failed its first home-angle return for every candidate (0.06049–0.06072 rad
  error, approximately 0.48 A stationary current). No firmware warning occurred
  in that comparison. The remaining repeats were stopped; original gains were
  restored again. It used a different hand posture from the earlier successes.
- [x] Isolate the thumb-home symptom from RAX's MCP/controller/wrapper using
  [standalone native SDK tests](wuji-sdk-isolation-2026-10-27.md). The same error
  reproduced at 120/200 Hz with quintic/linear interpolation and matched gains,
  cap and targets. This does not distinguish SDK/firmware, profile and mechanical
  causes; whole-hand home qualification remains open.
- [x] Review [Hand 2 controls/firmware evidence](wuji-hand2-controls-research-2026-10-27.md).
  Offline native traces show an apparent current-law scale near 1.98. External
  implementations suggest testing trajectory velocity and bounded feedforward;
  firmware compensation eligibility/calibration and static home bias remain open.

- [x] Complete the [expanded native-SDK parameter study](wuji-parameter-sweep-2026-10-27.md):
  95 thumb trials, three profiles completing six bidirectional thumb-home returns,
  and all twenty joints passing isolated local moves/returns. Kp 15/Kd 0.15
  reduced repeated thumb-home error to 0.00864–0.00984 rad at a 1.0 A cap.
  This exceeds RAX's current Kp admission ceiling and was not installed.
- [ ] Resolve the recurring ring `Enc1BitRate` cause. Coordinated
  flexion passed on thumb/index/middle, then stopped on this encoder warning
  during the ring's final return hold. It auto-cleared while disabled; all twenty
  joints were restored to 1.5/0.1/0.5 A. Full-hand home and manipulation remain
  unqualified. The watched pinky replay was clear, but used a different starting
  pose and did not resolve the cause of earlier held-ring S2 drift.
  A [guarded resumption](wuji-shortlist-validation-2026-10-27.md) then triggered
  the same encoder code on ring S3. The remaining comparisons stopped; original
  settings were restored and all twenty joints remain disabled. The warning
  cannot currently be treated as a one-off occurrence.
  The [encoder investigation](wuji-encoder-isolation-2026-10-27.md) found
  constant-target angle/current oscillation before both warnings. The operator
  clarified that they were not watching closely, so visible trembling is unknown.
  Disabled diagnostics and current bus counters
  are clean. A public-source search found an older firmware false-warning fix,
  already predating the installed version, but no matching current resolution.
  Feedback corruption versus actual internal oscillation remains unresolved.
  A [watched replay](wuji-visual-replay-2026-10-27.md) was steady at Kp 5;
  Kp 15 reproduced 1.03° peak-to-peak ring S3 feedback oscillation on the final
  return. The early guard stopped it with no native encoder warning. All joints
  were disabled with original settings restored. The operator was not watching
  that higher-stiffness event closely, so visible trembling remained unknown.
  A further explicitly requested repeat reproduced a 0.777° ring S3 feedback
  swing, and the operator **confirmed a visible twitch/tremble**. The early guard
  stopped that run without an encoder warning; all twenty joints are disabled
  with original settings restored. The physical movement is corroborated, but
  the encoder-warning root cause remains unresolved.
- [x] Complete the [moderate ring gain sweep](wuji-ring-stability-sweep-2026-10-27.md):
  twelve candidates, a repeated reference and two six-return validations;
  38 returns without detected repeated oscillation or encoder warnings. At the
  operator's request, save and apply Kp 10/Kd 0.15/1.5 A to ring S1/S3/S4 under
  revision `right-hand2-ring-kp10-kd015-20261027-v3`. Hardware readback matched,
  all twenty joints remained disabled, and 58 software checks passed. This
  selection does not resolve the encoder root cause or qualify whole-hand home.

## Camera calibration

- [x] Record each camera's factory color intrinsics for both used resolutions,
  distortion model/coefficients, native depth intrinsics, depth units and
  depth-to-color transform in `calibration/xarm-wuji/*_intrinsics.json`.
- [x] Check configured serial/profile/intrinsics against live camera readback.
  Depth is aligned to color and uses the color pixel geometry.
- [x] Confirm the new accurately printed board dimensions with operator: 10×13 squares,
  20 mm square, 15 mm marker. Images match DICT_4X4_100, the legacy 10×14
  layout with its last row removed. `board.json` preserves those marker IDs
  and coordinates and excludes missing-row corners; generating a fresh odd-row
  board produces a different pattern.
- [x] Camera-only detection and read-only arm/target preflight. Initial HD
  wrist/side reprojection means in the new-board preflight were 0.453/0.344 px
  with 108/61 corners.
- [x] Collect 18 arm poses while keeping the board and mounts fixed, holding
  the hand at its measured posture. Planned offsets: XY within ±25 mm,
  Z 0–50 mm, orientation change at most 21.2°, speed 15 mm/s.
- [x] Fit wrist-to-TCP and board-to-base using training poses; validate all
  three reserved poses. Require ≤5 mm and ≤1° residuals. Estimate the fixed
  side transform through the same stationary board and validate consistency.
- [x] Install `wrist_T_cam_tcp.yaml`, `wrist_T_cam_flange.yaml`, and
  `side_T_cam_base.yaml` only after those checks pass. Config already points
  to the installed files; normal observation loads them successfully.
- [x] Independently check RGB-D metric accuracy and cross-camera base-frame
  points. Factory depth scale readback alone does not establish metric accuracy.
- [x] Verify normal 640×480 RAX observations, streaming and pixel-to-base
  conversion through the MCP frontend.

## Network and operator tools

The hand's Ethernet route must select `enx6c1ff7618956`, source
`192.168.1.11`. Discovery alone can succeed while the control connection uses
an incorrect route. Verify both destinations after adapter reconnection:

```bash
ip route get 192.168.1.111
ip route get 192.168.100.16
```

The confirmed local topology currently uses this temporary NetworkManager route:

```bash
nmcli device modify enx6c1ff7618956 \
  +ipv4.routes '192.168.1.111/32 0.0.0.0 50 src=192.168.1.11'
```

It has not been persisted in the connection profile. Existing arm routes and
Wi-Fi settings are unchanged. The camera-only viewer is at
<http://localhost:8765/>; UFactory Studio is at
<http://192.168.100.16:18333/>. Stop the viewer before starting a backend camera
session because both use the same devices.

Calibration is complete. Final evidence and repeat-collection instructions are
in the [calibration report](../calibration/xarm-wuji/README.md). Rejected runs,
raw captures and the commissioning audit remain under `.local-runtime/`.

## Real backend MCP qualification

- [x] Historical 2026-10-25 hand-only MCP discovery, status, invalid-input
  rejection, bounded index-joint moves, named recorded posture, holding and
  feedback-confirmed disable. See the linked report for failed precision trials
  and the exact scope; it does not qualify all fingers or manipulation.
- [x] Repeat connection/status and camera observations with the current setup.
- [x] Exercise all seven xArm MCP tools: observation, projection, pixel conversion,
  live/history/contact-sheet streaming, relative/absolute/joint motion, home,
  return, hold, input rejection, timeout recovery and motion recording. Corrected
  runs passed 35 read-only/admission and 33 motion/recording checks.
- [x] Repeat all four Wuji MCP interfaces, including 22 input/admission rejections,
  holding, a recorded named return and confirmed disable. The nonzero index S3
  trial failed with 0.04655 rad residual error against a 0.040 rad tolerance.
- [ ] Diagnose the hand's repeatable settling error and resume individual-joint
  qualification. The wider twenty-joint sweep was stopped after the first failure;
  gains, current limits and tolerance were not increased in that initial test.
  Later trials changed gains and tolerance; see the current v12 report above.
- [x] Verify the bounded calibration arm motions with the hand holding and
  return to the recorded start pose, followed by disarm and confirmed hand disable.
- [ ] Qualify a repeatable episode reset, including a named hand posture and
  the intended task motion envelope.
- [x] Exercise independent operator disarm during arm motion and an active hand
  command; inspect both arm and hand outcomes. First stopped/disabled samples
  arrived after 73.85 ms (arm) and 114.59 ms (hand) in these trials. The hand command
  retained its posture, so this did not measure moving-finger braking. MCP requests
  serialize; MCP hold cannot preempt an active MCP motion request. Use the private
  operator stop interface. Immediate hold during emergency stopping can be rejected;
  wait for stop completion and inspect feedback.
- [x] Extend disarm qualification to actual moving fingers under v13. During a
  sixth fist move, disarm stopped and disabled all twenty joints within the
  observed 61.04 ms confirmation interval. Arm stayed in state 4. Final hand
  disable was confirmed; the interrupted posture was not returned to home.
- [ ] Qualify communication loss, process loss and physical stop behavior with
  an appropriate controlled test setup.
- [ ] Qualify grasp, retention, transport, release and repeated episodes with
  the intended objects. No tactile feedback or full-hand collision model has
  been established by the current integration.
