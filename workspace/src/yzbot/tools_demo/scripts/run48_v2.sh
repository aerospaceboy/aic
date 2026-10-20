#!/usr/bin/env bash
# Motion-only 48-task coverage. Both color orders are always included.
# Usage: run48_v2.sh [first=1] [last=48] [legacy-variant] [fresh-output-root]
# A checkpoint requires CASE_ACCEPTED, motion_under_290 and a frozen version.
set -uo pipefail
repo=$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
workspace=$(cd "$repo/../.." && pwd)
first=${1:-1}
last=${2:-48}
root=${4:-"$workspace/experiments/coverage-48"}
mkdir -p "$root"
mapfile -t QUESTIONS < <(python3 "$repo/tools_demo/scripts/gen48_questions.py")
if [ "${#QUESTIONS[@]}" -ne 48 ] || [ "$first" -lt 1 ] || [ "$last" -gt 48 ] || [ "$first" -gt "$last" ]; then
  echo QUESTION_RANGE_INVALID >&2
  exit 2
fi
version=$({ git -C "$repo" rev-parse HEAD; git -C "$repo" diff HEAD --binary; } | sha256sum | cut -d' ' -f1)
version_file="$root/campaign-version.sha256"
if [ -f "$version_file" ] && [ "$(cat "$version_file")" != "$version" ]; then
  echo CAMPAIGN_VERSION_CHANGED_USE_NEW_OUTPUT_ROOT >&2
  exit 2
fi
printf '%s\n' "$version" > "$version_file"
export RLC_TIMEOUT_SEC=${RLC_TIMEOUT_SEC:-600.0}
for index in $(seq "$first" "$last"); do
  current=$({ git -C "$repo" rev-parse HEAD; git -C "$repo" diff HEAD --binary; } | sha256sum | cut -d' ' -f1)
  if [ "$current" != "$version" ]; then
    echo CAMPAIGN_SOURCE_CHANGED_STOP >&2
    exit 2
  fi
  line="${QUESTIONS[$((index - 1))]}"
  tag="${line%% *}"
  question="${line#* }"
  case_root="$root/$tag"
  mkdir -p "$case_root"
  marker="$case_root/COMPLETED"
  if [ -f "$marker" ] && grep -q "version=$version" "$marker"; then
    echo "SKIP $tag (accepted on this campaign version)"
    continue
  fi
  log=$(mktemp "$case_root/harness-XXXXXX.log")
  echo "RUN $index/48 $tag"
  rc=0
  bash "$repo/tools_demo/scripts/run_ordered_case.sh" "$question" "$case_root" > "$log" 2>&1 || rc=$?
  out=$(sed -n 's/^EVIDENCE_DIRECTORY=//p' "$log" | tail -1)
  if [ "$rc" -eq 0 ] && [ -n "$out" ] && grep -aq 'CASE_ACCEPTED=true' "$out/result.log" && \
      python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("motion_under_290") is True else 1)' "$out/audit.json"; then
    printf 'version=%s evidence=%s\n' "$version" "$out" > "$marker"
    echo "DONE $tag evidence=$out"
  else
    echo "FAILED $tag rc=$rc evidence=${out:-missing} harness=$log"
  fi
  # Let this run's process group, Gazebo services and DDS participants
  # finish shutting down. 2026-10-06: an 8 s gap was not enough once -
  # the next case started with /gazebo/set_entity_state and
  # /collision_monitor/change_state timing out and the mission never
  # left waiting_for_task (case 3rB_2bA, elapsed 0.0).
  sleep 20
  if pgrep -x gzserver >/dev/null; then
    echo STALE_SIM_STOP_BATCH_WITHOUT_KILLING_OTHER_OWNER >&2
    exit 2
  fi
done
echo BATCH_DONE
