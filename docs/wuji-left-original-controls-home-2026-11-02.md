# Left-hand arbitrary-to-home test with original control values

All three physical arbitrary-pose-to-home cycles completed on left Hand 2
`WH2JA01260811007`, firmware `2.13.1`, with Kp 8, Kd 0.15 and a 1.5 A cap on
the eighteen active joints. The two excluded joints stayed Stopped with zero
gains/current limits. The right-hand process and command log were unchanged;
both arm joint readbacks were unchanged. No simulated device was used.

## Comparison with real/xarmwuji

Fetched `real/xarmwuji` at commit `899f0c6`. Its `examples/xarm-wuji.json` uses
Kp 8 / Kd 0.15 / 1.5 A on right hand `WH2KA01260823017`. Those three control
values were copied exactly for the active left joints. The excluded-joint zero
override, 200-Hz rate and staged trajectory limits remain specific to this test.

The original branch did not ignore native `Stall (0x0007)`: its SDK boundary
latched the diagnostic and stopped control. It supported reconnecting after the
native warning cleared. Its contact completion accepted stable position shortfalls,
which is distinct from accepting a native Stall warning.

Partial-hand recovery was corrected to permit the two exact, configured isolated
codes to remain on disabled joints while unrelated native Stall codes must clear.
This path was exercised on the physical hand, including two seconds of fresh,
disabled, stationary readback before reconnect. Native faults were not manually
cleared, and reconnect did not replay a trajectory.

## Home and movement results

The operator chose the **current left-hand pose**, replacing an earlier selection
of the right-hand saved angles before any home test began. The measured left home
is saved in the session's `operator/left-current-home.json`. Excluded entries use
the fresh preparation seed as wire placeholders; they are never commanded to a
saved angle.

Three reproducible configurations used seed `2026110307`, with independent active
joint offsets bounded by 0.08, 0.12 and 0.16 rad from that home. Outbound and return
paths were staged into MCP `wuji_move_joints(hand="left")` calls no larger than
0.04 rad per joint. This is a bounded arbitrary-pose test, not a repeat of the old
full-range right-hand random sweep or a test of grasping with passive joints.

| Trial | Maximum sampled offset bound | Largest active home-return error |
| --- | ---: | ---: |
| 1 | 0.08 rad | 0.012369 rad / 0.71° |
| 2 | 0.12 rad | 0.017973 rad / 1.03° |
| 3 | 0.16 rad | 0.018028 rad / 1.03° |

All returns were within the original branch's 0.06-rad position tolerance.
The partial controller retained its 0.015-rad reporting tolerance: eleven calls
reported a position shortfall while remaining enabled and holding, and the next
command proceeded without preparation or a stop. This physically exercised the
new nonfatal shortfall behavior.

The successful run completed 21 MCP commands, including initial alignment to the
saved home. Across 867 status samples there were no new blocking native faults.
Both excluded joints were continuously Stopped, with verified zero gains/current
limits; active current-cap readbacks were 1.5 A. Peak sampled active current was
0.770 A and peak MCU temperature was 59.96 °C. The controller delivered 188.9
commands/s against 200 Hz requested; maximum command interval was 22.13 ms.

Earlier 0.3 A trials stopped on ring S2 and pinky S2 Stall warnings while their
measured current reached about 0.31 A. A 0.5 A retry advanced farther, then ring
S2 stalled near 0.52 A. The 1.5 A run used the same saved home and sampled targets.
It succeeded while allowing current above the earlier caps; the cap is a ceiling,
not a constant current command.

Final readback confirmed fresh feedback, disabled/Ready or Stopped states for all
twenty joints, no controller ownership, no retained blocking fault, and disarmed
motion. The two original isolated native fault codes remain visible.

## Saved evidence and operating state

Session: `/home/jerry/rax-runtime/sessions/20261103T034138Z_left_partial_real_probe/`.

- `original-controls-home-summary.json`: final results, readbacks and peer checks.
- `original-branch-control-values.json`: source branch/commit and copied values.
- `operator/left-current-home.json`: user-selected measured home.
- `arbitrary-home-plan.json`: seed, targets and staged paths.
- `arbitrary-home-result.json` and `arbitrary-home-telemetry.jsonl`: final physical run.
- `ring-s2-cap03-*`, `pinky-s2-cap03-*`, `all-cap05-*`: retained unsuccessful runs.
- `stall-recovery-real-result.json` and `operator/stall-recovery-left-*.json`: real recovery evidence.
- `mcp.jsonl` and `wuji-left-controller.log`: MCP and native SDK audit.

The standalone left-only MCP session has the final profile loaded. The original
bimanual session remains disarmed with its old left child stopped. Its validated
partial-hand configuration draft is updated, with the right configuration unchanged;
a coordinated reload is still needed before ordinary bimanual use.
