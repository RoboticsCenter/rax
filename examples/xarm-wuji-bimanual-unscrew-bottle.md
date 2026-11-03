# Unscrew and remove the bottle cap

Use the two xArm 6 + Wuji Hand 2 assemblies to stabilize the upright bottle,
unscrew its cap, fully detach the cap, and set the cap down separately while
leaving the bottle upright. Do not pour its contents or crush the bottle.
Verify success visually from fresh camera views and record the procedure.

The bottle currently stands on a raised cardboard box between the arms. Locate
the actual box top, bottle body, neck and cap using calibrated RGB views. The
original calibration-board plane is below this support and is not the bottle's
support plane. At the setup's home poses, the cap is cropped out of the available
views; obtain a clear view before attempting contact. Keep the box and camera
mounts in place.

Prefer the left hand to stabilize the bottle body and the right hand to manipulate
the cap if the observed reach and grasps support that division. The left hand uses
its commissioned 18-joint profile with thumb_S3 and middle_S1 passive; the right
hand controls all 20 joints. Use each hand's own limits and saved home. Confirm
grasp and cap rotation from observation rather than assuming trajectory completion
proves contact or thread release. Regrasp as needed within the measured clearance.

Arm targets use their own left/right base frames. Observe metadata provides both
camera transforms and the inter-base transform. Work from fresh calibrated RGB
images; depth metric calibration is pending. Release supported objects before
returning the hands and arms to their established home poses.

For this first run, complete one bottle opening, leave the scene stable, and
report the result. Do not begin another physical episode without an operator
reset. The operator launches this session manually and prepares the assemblies
through the operator controls; if motion is unarmed, wait for that preparation.
