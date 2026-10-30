Pick up the Rubik's cube with the right Wuji Hand 2 on the xArm 6 and place it
inside the bowl. Success means the cube is supported inside the bowl, the hand
has released it, and the arm has retreated with clearance. Verify placement in
both camera views. Begin each training episode from both home positions and
finish each episode with the return-home procedure below.

Use fresh observations to localize the cube, bowl and hand. Cameras are `wrist`
(D435 262422074331, TCP-mounted) and `side` (D435 348522070593, fixed). Runtime
RGB-D is 640×480 at 6 fps. Allow camera warm-up and check valid depth before
using `xarm_pixel_to_position`. Calibration was installed and verified on
October 29: board-point median errors were 0.82–0.95 mm wrist and 1.19–1.39 mm
side, with 1.66–1.71 mm median cross-camera disagreement (3.19 mm worst-frame
95th percentile). The wrist model includes calibrated depth alignment and a
software metric depth correction. Those measurements cover the observed
tabletop and calibration poses. Use both views and small, visible approach steps.

The controller TCP remains 172 mm along flange Z. It is not a measured Wuji palm
or fingertip contact reference. Account for the whole hand when converting an
object position into an arm target; establish the contact offset and finger
sweep before descent. The configured workspace box bounds the TCP only.

The configured hand profile is `right-hand2-v13-observe-strict-20261029`: local
jerk-limited position/velocity references, 1000 Hz requested, Kp 8/Kd 0.15 and
1.5 A caps on all twenty joints. Reference limits are 0.6 rad/s, 3 rad/s² and
2 rad/s³; endpoint tolerance is 0.060 rad with stable feedback required. Keep
these settings unchanged. Encoder warnings are faults under this strict profile;
stop and report them rather than clearing or ignoring them. Inspect command
timing; reference limits do not establish actual contact force or
absolute encoder accuracy. All tactile channels currently report offline.

`home` and `fist` are saved joint postures. Neither is a qualified cube grasp or
guaranteed release shape. Preparation holds the measured posture. Establish an
appropriate opening, opposing finger placement and release in clear space using
bounded visible motions; record successful joint targets. Do not command an
empty-hand fist around an object or treat a settling timeout as proof of grasp.
If the available position tools cannot hold the cube within their existing
tracking/settling guards, request operator help instead of increasing gains,
current caps or tolerances.

The task session limits arm translation to 50 mm per call and Cartesian speed
to 15 mm/s; take smaller steps near the table and objects. Check retention with
a short lift before transport and keep the accepted hand target while carrying.
Stop on faults or uncertain retention, and do not silently retry failed motion.

Between every training episode, support and release any held object, verify that
the hand is empty, and retreat into clear space with clearance for the full hand
and arm return path. Clear any task-applied feed-forward current bias after
release. Call `wuji_home()` through the custom hand motion controller, wait for
measured completion, then call `xarm_move_joints(preset="home")`. Keep the tuned
gains, damping, S-curve limits and current caps unchanged. Verify fresh, fault-free
hand/arm status and both camera views before requesting the next scene reset.
Use this sequence after recoverable failed attempts as well as successful ones.
If home is blocked, motion faults, or state is uncertain, stop and request specific
operator help; do not force a reset or begin another episode. Record both reset
results with the episode, and recheck the scene after the operator's reply.

The October 28 v13 RAX test completed home → fist → home → fist → home with
both cameras streaming. Maximum endpoint error was 0.03875 rad, and the operator
reported smooth, acceptable motion. A moving-hand disarm confirmed disabled,
low-speed feedback within 61 ms. This qualifies those empty-hand paths at the
stationary commissioning arm pose; cube contact and loaded transport remain open.

Only operator controls can ready the arm, arm the session and prepare the hand.
Before contact, review the task's payload, contact geometry and swept clearance
with the operator. The existing 0.82 kg payload and [0, 0, 48] mm center of
gravity are controller readbacks, not verified measurements of this assembly.
If a prerequisite is missing, use `request_operator` with the specific missing
fact. Use only configured MCP tools for hardware access. Save decisive frames,
useful postures and the final result in workspace memory.
