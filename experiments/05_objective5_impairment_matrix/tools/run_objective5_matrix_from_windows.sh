#!/usr/bin/env bash
# Permanent, git-tracked Windows -> WSL launch wrapper for the Objective5
# impairment-matrix orchestrator (run_objective5_impairment_matrix_trial.sh).
#
# This wrapper does exactly two things and nothing else:
#   1. Register + verify WSL interop as root, aborting immediately if the
#      check fails (never silently continues into a doomed Webots launch).
#   2. Switch to the eamon user and invoke the real orchestrator with every
#      argument passed through UNCHANGED.
# It never copies, duplicates, or rewrites the orchestrator, the controller,
# the relay, or the analyzer -- those live only in their one real location
# under tools/ and src/epuck2_comm/epuck2_comm/. This script exists solely
# because registering WSL interop and then invoking anything that depends on
# it must happen inside the SAME wsl.exe process invocation in this
# environment (confirmed empirically earlier this session: interop
# registered in one wsl.exe call does not persist into a later, separate
# wsl.exe call).
#
# Usage:
#   run_objective5_matrix_from_windows.sh CONDITION_ID TRIAL_INDEX [ATTEMPT]
#   run_objective5_matrix_from_windows.sh CONDITION_ID TRIAL_INDEX --pilot LABEL
#   run_objective5_matrix_from_windows.sh --check-only CONDITION_ID TRIAL_INDEX [...]
#
# --check-only verifies interop, orchestrator presence, and the four
# behavioral-code SHA-256 values against the values frozen for the
# Condition A formal batch -- WITHOUT launching Webots. Any other arguments
# are only echoed back to confirm passthrough, never executed.
#
# From Windows PowerShell:
#   wsl.exe -d Ubuntu-22.04 -u root -- bash "<repo>/experiments/05_objective5_impairment_matrix/tools/run_objective5_matrix_from_windows.sh" B 1

set -eo pipefail

REPO="/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm"
ORCHESTRATOR="$REPO/experiments/05_objective5_impairment_matrix/tools/run_objective5_impairment_matrix_trial.sh"
RELAY_PY="$REPO/src/epuck2_comm/epuck2_comm/network_impairment_relay.py"
IMPAIRMENT_PY="$REPO/src/epuck2_comm/epuck2_comm/network_impairment.py"
SEQCOUNTER_PY="$REPO/src/epuck2_comm/epuck2_comm/sequence_counter.py"

# Frozen behavioral-code SHA-256, as verified identical across all 5
# Condition A formal trials (commit 2837d08 / 4882489). Any future
# condition's Trial 01 must match these UNLESS a deliberate, reviewed code
# change was made and this wrapper's expected values were updated alongside
# it in the same commit -- never silently.
EXPECTED_ORCH_SHA256="20d2ef63a152a7d65632e4fd3414c9cd1cdaa2a449f58daf7eac1bd28110913b"
EXPECTED_RELAY_SHA256="f5d408bc3379f79fa70628370b4dfb6d537c4d03a1968fe8dc75a691c3e6d5ff"
EXPECTED_IMPAIRMENT_SHA256="253e0d960e9b587a3c5e60587ce7ac56c167fd6aba1c98f8b7b940e821210561"
EXPECTED_SEQCOUNTER_SHA256="57bb0699a444df644d75c4e834b5fd13b5f15a6283d7b1d276ec0b65674f1fd3"

CHECK_ONLY="false"
ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--check-only" ]]; then
    CHECK_ONLY="true"
  else
    ARGS+=("$arg")
  fi
done

if [[ "$(id -u)" -ne 0 ]]; then
  echo "FATAL: this wrapper must be launched as root (wsl.exe -u root ...) so it can register WSLInterop; it switches to eamon itself before invoking the orchestrator or checking anything eamon-owned." >&2
  exit 1
fi

echo ':WSLInterop:M::MZ::/init:PF' > /proc/sys/fs/binfmt_misc/register 2>/dev/null || true
echo "=== interop registered, verifying ==="
INTEROP_CHECK="$(/mnt/c/Windows/System32/cmd.exe /c echo INTEROP_OK 2>/dev/null || true)"
if [[ "$INTEROP_CHECK" != *INTEROP_OK* ]]; then
  echo "FATAL: WSL interop check failed (got: '$INTEROP_CHECK') -- aborting, NOT starting the trial." >&2
  exit 1
fi
echo "INTEROP_OK confirmed"

if [[ "$CHECK_ONLY" == "true" ]]; then
  echo "=== --check-only: verifying orchestrator presence + behavioral-code SHA-256 + argument passthrough. Webots is NOT launched. ==="

  if [[ ! -f "$ORCHESTRATOR" ]]; then
    echo "FATAL: orchestrator not found at $ORCHESTRATOR" >&2
    exit 1
  fi
  echo "orchestrator found: $ORCHESTRATOR"

  CHECK_FAILED=0
  check_sha() {
    local label="$1" path="$2" expected="$3"
    if [[ ! -f "$path" ]]; then
      echo "FAIL  $label: file not found at $path"
      CHECK_FAILED=1
      return
    fi
    local actual
    actual="$(sha256sum "$path" | awk '{print $1}')"
    if [[ "$actual" == "$expected" ]]; then
      echo "OK    $label: $actual"
    else
      echo "FAIL  $label: expected $expected, got $actual"
      CHECK_FAILED=1
    fi
  }
  check_sha "orchestrator" "$ORCHESTRATOR" "$EXPECTED_ORCH_SHA256"
  check_sha "network_impairment_relay.py" "$RELAY_PY" "$EXPECTED_RELAY_SHA256"
  check_sha "network_impairment.py" "$IMPAIRMENT_PY" "$EXPECTED_IMPAIRMENT_SHA256"
  check_sha "sequence_counter.py" "$SEQCOUNTER_PY" "$EXPECTED_SEQCOUNTER_SHA256"

  echo "=== argument passthrough check ==="
  echo "condition_id=${ARGS[0]:-<none>} trial_index=${ARGS[1]:-<none>} extra_args=${ARGS[*]:2}"

  if [[ $CHECK_FAILED -ne 0 ]]; then
    echo "--check-only FAILED -- one or more behavioral-code files do not match the frozen Condition A SHA-256 values, or the orchestrator is missing. NOT ready to launch a trial." >&2
    exit 1
  fi
  echo "--check-only PASSED. No Webots launched."
  exit 0
fi

echo "=== switching to eamon and invoking the orchestrator (args unchanged: ${ARGS[*]}) ==="
QUOTED_ARGS="$(printf '%q ' "${ARGS[@]}")"
su - eamon -c "bash '$ORCHESTRATOR' $QUOTED_ARGS"
