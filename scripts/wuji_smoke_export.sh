#!/usr/bin/env bash
# One comprehensive Wuji Hand 2 smoke test, then export everything for offline analysis.
#
# Runs the motion battery (docs/wuji-smooth-control.md) on one hand across conditions
# M0..M4 and all free-space items, scores it with dex_eval, and packs trials, report,
# logs, environment and the battery's own session into ~/battery/<name>.tar.gz.
#
# Usage (from the rax checkout, RAX not running, person at the e-stop):
#   bash scripts/wuji_smoke_export.sh <session operator-input.json> [--hand right|left] [--name full1] [--dry-run]
#
# The battery prints the plan and an arming command; arm in a second terminal with
#   python scripts/arm_robot.py <printed battery session> arm --minutes 60
# then type READY. Nothing moves without arming + READY. Arms are never connected.
set -u
RAX="$(cd "$(dirname "$0")/.." && pwd)"
cd "$RAX"

SESSION=""; HAND=right; NAME=""; EXECUTE=--execute
while [ $# -gt 0 ]; do
  case "$1" in
    --hand) HAND="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --dry-run) EXECUTE=""; shift ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    *) SESSION="$1"; shift ;;
  esac
done
[ -n "$SESSION" ] || { sed -n 2,13p "$0"; exit 2; }
[ -e "$SESSION" ] || { echo "no such session: $SESSION"; exit 2; }
NAME="${NAME:-smoke-${HAND}-$(date +%Y%m%dT%H%M%S)}"
PY="${RAX_PYTHON:-$RAX/.venv/bin/python}"

# Artifact root defaults to the one that owns the session (<root>/sessions/<id>/operator-input.json).
SESSION_DIR="$SESSION"; [ -f "$SESSION" ] && SESSION_DIR="$(dirname "$SESSION")"
export RAX_ARTIFACT_ROOT="${RAX_ARTIFACT_ROOT:-$(dirname "$(dirname "$SESSION_DIR")")}"
export PYTHONUNBUFFERED=1

BASE=~/battery; OUT="$BASE/$NAME"
mkdir -p "$OUT"

# Stream limits from the operator tuning order; friction starts low (0.04 A commanded,
# ~0.08 A applied at the measured ~1.98x gain factor). The left hand gets half the limits.
if [ "$HAND" = left ]; then ACC=2.5; JERK=15; else ACC=5; JERK=30; fi
cat > "$OUT/patch.json" <<J
{"all": {"stream_max_acceleration_rad_s2": $ACC, "stream_max_jerk_rad_s3": $JERK, "friction_current_a": 0.04}}
J

echo "== battery: hand=$HAND out=$OUT artifact_root=$RAX_ARTIFACT_ROOT ${EXECUTE:-(dry run)}"
"$PY" scripts/wuji_motion_battery.py --session "$SESSION" --hands "$HAND" \
  --conditions M0_baseline,M1_stream_1khz,M2_stream_friction_ff,M3_soft_gains_cap,M4_online_replan \
  --items single_joint,stick_slip,fist_pinch,tapping,hold,waypoints5 \
  --repeats 1 --shuffle --seed 1 --hand-patch "$OUT/patch.json" --start-temp-c 55 \
  --out "$OUT" $EXECUTE 2>&1 | tee "$OUT/battery.log"
[ -n "$EXECUTE" ] || { echo "dry run only; re-run without --dry-run to move the hand"; exit 0; }

echo "== report"
"$PY" scripts/dex_eval_report.py "$OUT" --group-by controller --baseline M0_baseline \
  --out "$OUT/report" 2>&1 | tee "$OUT/report.log"

{
  echo "rax: $(git rev-parse HEAD) ($(git rev-parse --abbrev-ref HEAD))"
  git status -s
  "$PY" -m pip freeze 2>/dev/null | grep -iE "wuji|ruckig|numpy|scipy"
  uname -a; date -Is
} > "$OUT/env.txt" 2>&1

# Pack the output plus the battery's own session (hand-owner logs and config); sockets are skipped.
BS="$(grep -m1 -oE 'Battery session: .*' "$OUT/battery.log" | sed 's/^Battery session: //')"
ITEMS=(-C "$BASE" "$NAME")
[ -n "$BS" ] && [ -d "$BS" ] && ITEMS+=(-C "$(dirname "$BS")" "$(basename "$BS")")
tar czf "$BASE/$NAME.tar.gz" "${ITEMS[@]}" 2>/dev/null
ls -lh "$BASE/$NAME.tar.gz"
echo "DONE -> copy $BASE/$NAME.tar.gz to the analysis machine"
