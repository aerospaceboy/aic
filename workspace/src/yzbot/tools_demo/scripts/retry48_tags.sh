#!/usr/bin/env bash
# Re-run selected 48-case tags (sparse retry after a coverage pass).
# Usage: retry48_tags.sh <tag> [tag...]
# A tag is skipped when its most recent evidence directory already holds a
# completed MISSION_TERMINAL; otherwise one fresh run_ordered_case.sh run
# is made and its evidence directory printed.
set -u
repo=$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel 2>/dev/null || echo /home/ros/dev_ws/src/yzbot)
workspace=$(cd "$repo/../.." && pwd)
root=${COVERAGE_ROOT:-"$workspace/experiments/coverage-48"}
mapfile -t QUESTIONS < <(python3 "$repo/tools_demo/scripts/gen48_questions.py")
for tag in "$@"; do
  index=-1
  for i in "${!QUESTIONS[@]}"; do
    if [ "${QUESTIONS[$i]%% *}" = "$tag" ]; then index=$i; break; fi
  done
  if [ "$index" -lt 0 ]; then echo "UNKNOWN_TAG $tag"; continue; fi
  question="${QUESTIONS[$index]#* }"
  # settle so the previous teardown finishes before the stale-sim guard
  sleep 10
  echo "=== RETRY $tag: $question"
  out=$(bash "$repo/tools_demo/scripts/run_ordered_case.sh" "$question" "$root" 2>&1 | grep -a "EVIDENCE_DIRECTORY=" | cut -d= -f2)
  if [ -z "$out" ]; then echo "FAILED_NO_EVIDENCE $tag"; continue; fi
  if grep -aq '"state": *"completed"' "$out/result.log" 2>/dev/null; then
    echo "=== RETRY_DONE $tag (evidence $out)"
  else
    echo "=== RETRY_FAILED $tag (evidence $out)"
  fi
done
echo RETRY_PASS_DONE
