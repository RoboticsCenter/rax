# Wuji Hand 2 firmware upgrade — 2026-10-27

**Right hand `WH2KA01260823017` was upgraded from firmware 2.6.0 to 2.13.1.**
Independent CLI and native SDK checks confirmed the installed version and the
new endpoint, `192.168.1.111:50001`. All twenty joints reported Ready/disabled
with no active diagnostic error. The xArm pose was unchanged.

The operator explicitly authorized the upgrade after reviewing the procedure.
The official CLI 2026.10.26 was installed at `/home/jerry/.local/bin/wuji`, using
the vendor installer with optional agent-skill installation skipped. The
installer verified the release archive's SHA-256 digest. No hand controller was
running; the separate camera viewer was left active.

## Upgrade and verification

The serial-specific catalog offered 2.13.1 as latest and also listed 2.12.0 and
2.6.0. The preflight recorded the old firmware, measured posture, diagnostics and
the existing Kp 1.5, Kd 0.1, 0.5 A profile. Motor disable was requested and fresh
Ready feedback was confirmed before flashing. The xArm was in state 5, with no
error/warning; the SDK treats states 4 and 5 as not ready to move. No arm writes
were performed.

With the RAX and teleop advisory device locks held, the actual command was:

```bash
wuji upgrade --sn WH2KA01260823017 --to 2.13.1 --yes --json
```

The [official CLI upgrade procedure](https://docs.wuji.tech/docs/en/wuji-cli/latest/firmware-upgrade)
selects a matching catalog package, verifies it, flashes it and reboots the
device. The recorded result was `status: ok`, `firmware_before: 2.6.0`,
`firmware_expected: 2.13.1`, `firmware_after: 2.13.1`, duration 62 seconds and
exit code 0. The catalog package SHA-256 was
`9009442050fcf9fd34ec6af6e933a95d25080b1f002c15d5124f7808249f4c01`.

After reboot:

- `wuji devices` discovered the same serial at `192.168.1.111:50001`.
- `wuji ping` independently reported firmware 2.13.1 and data port 50001.
- Native SDK subscriptions returned all twenty joint states and diagnostics;
  all joints were disabled and all active error codes were zero.
- The rebooted profile had Kp 1.5/Kd 0.1 but a **1.5 A current limit**. With the
  hand disabled, the maintenance script restored and read back Kp 1.5/Kd 0.1
  and **0.5 A** for every joint. These are runtime settings; no `save_params`
  or factory-reset operation was performed.
- The actual RAX `WujiDriver` connected using the updated configuration, decoded
  fresh complete feedback and successfully applied/read back that profile while
  the joints remained disabled. RAX's pinned SDK remains **2026.10.12**.

Only the address in `examples/xarm-wuji.json` changed, from port 7447 to 50001.
Saved home angles, profile limits and camera calibration were preserved. Use a
new session generated from the updated configuration; historical session
snapshots still record the old endpoint.

The before/after arm pose matched exactly at the reported precision. The largest
hand position difference across the disabled/reboot interval was 0.05117 rad;
this is not a motion-command or accuracy test. The maintenance and validation
scripts never enabled motors or published joint targets.

## Remaining control qualification

The upgraded resource list exposes per-joint constant/velocity and position-table
feedforward setters. Their presence does not establish useful calibration or
effective compensation on this hand. No compensation setter was called, and
calibration eligibility remains unresolved. See the
[controls research](wuji-hand2-controls-research-2026-10-27.md) and
[official release notes](https://docs.wuji.tech/docs/en/release-notes).

The upgrade is complete. The later
[vendor-profile test](wuji-vendor-profile-test-2026-10-27.md) on 2.13.1 failed
initial thumb-home settling even with Kp 5/Kd 0.1/1.5 A. Earlier gain/native-SDK
results describe firmware 2.6.0. Home repeatability and broader motion/stop
qualification remain incomplete.

## Evidence

```text
.local-runtime/wuji-firmware-upgrade-20261027/
  install-cli.sh, install-cli.log
  catalog-check.json, catalog-versions.json
  inspect_hand.py, preflight.json, preflight.log
  upgrade-command.json, upgrade.json, upgrade.log, upgrade-exit.json
  devices-after.json, ping-after.json
  postflight.json, postflight.log
  rax-driver-after.json
```

An initial preflight stopped before connecting to the hand because it expected
xArm state 4. After checking the installed xArm SDK's treatment of state 5,
the maintenance-only check accepted either stopped/not-ready state; the arm's
state was not changed. That aborted read is retained as
`preflight-arm-state5.json`.
