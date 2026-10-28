# Wuji skill audit — October 28, 2026

Audited the provided xArm/Wuji control, perception, manual-operation, automation and
experience-memory skills against the backend, controller, recording implementation
and operator workflow. This is a software/instruction audit, not physical grasp or
force-control qualification.

Corrections made:

- Hand guidance covers S-curve control, all six Wuji MCP tools, current ramping and
  persistence, active versus baseline impedance/current caps, stop behavior, and
  reprepare versus arming renewal. Named home/fist postures are not automatically
  qualified grasps.
- Exploration uses explicit hypotheses, small reversible probes in observed clear
  space, before/after evidence and bounded uncertainty. Complete CAD registration
  is not a prerequisite for every probe; an unbounded next action still requires
  better observations or specific operator assistance.
- Arm limits come from the live tool description. Relative Cartesian translation
  is in base axes. TCP bounds and home presets do not prove whole-hand clearance.
- Camera guidance distinguishes changing capture-time camera pose from fixed mount
  calibration, addresses invalid close-range depth and occlusion, and preserves
  posture/orientation dependence of contact offsets.
- Recording guidance uses configured rates/windows/camera names and distinguishes
  rolling history, saved motion clips and optional continuous archives.
- Automation and learned memory preserve full grasp control state and its evidence,
  and distinguish resumed-session memory from a new session's empty memory.

The existing cube session was inspected read-only. Its copied hand skill still said
gain/current changes were unavailable, and its contract listed the four older Wuji
tools. All eight supplied skills and the experience-memory reference differed from
the newly rendered sources. Its live files and processes were not changed.

`scripts/refresh_wuji_tools.py` now refreshes the complete provided skill set and
references, plus the tool contract, for a stopped session. It backs up overwritten
files and records their new hashes. Custom skills, task text, conversation, memory,
code, reference geometry and recordings are preserved. Resume through the official
launcher to load the updated backend, then reread the refreshed skills.

Workspace rendering, migration/preservation, and Wuji MCP/contract integration checks
passed. A fresh rendering of the cube configuration had no broken relative Markdown
links and advertised all six Wuji tools. The audit did not qualify live effort or
gain changes, refresh a running session, or transplant learned experimental geometry
into general-purpose instructions.
