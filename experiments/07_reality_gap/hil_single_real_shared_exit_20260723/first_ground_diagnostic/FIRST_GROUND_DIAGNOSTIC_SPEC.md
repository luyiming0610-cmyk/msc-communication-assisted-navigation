# First supervised low-speed HIL ground diagnostic -- specification

**Classification: `EXCLUSIONARY_DIAGNOSTIC`, not a formal navigation
trial and not the formal shared-exit experiment.** Prepared offline
2026-07-24, following the accepted
`SUSPENDED_ZERO_MOTION_COMMAND_EVIDENCE_PASS` record (commit
`d49be1083a36c539aa5f5d756d1318eac2f14f5f`). This document is a
specification only -- nothing here has been run. See
`GROUND_DIAGNOSTIC_RUNBOOK.md` for the exact execution procedure and
`../HIL_SAFETY_CHECKLIST.md` for the binding safety gate this
diagnostic must pass through unchanged.

## Purpose

The first time the real e-puck2 (device `epuck5809`) is commanded to
move while its wheels are actually bearing weight on the ground, under
full command-evidence observation (Pi-side audit + WSL-side recorder,
both already verified working:
`../command_evidence_activation_pass_20260724/SUMMARY.md`). Its only
goal is to observe a single bounded, low-speed, straight-line motion
safely and completely, with continuous evidence, before any formal
trial is ever attempted on this hardware.

## Scope (binding)

- **One real e-puck2 only.** No virtual peer.
- **Straight-line movement only.** `angular.z` is fixed at `0.0` for
  the entire diagnostic -- requested, guarded, and applied. Ground
  angular motion is explicitly **prohibited** for this first test (see
  `ground_diagnostic_params.json`'s `diagnostic_command_limits`).
- **One short, bounded, low-speed pulse**, within the previously
  suspended-wheel-tested linear range: requested speed `0.015 m/s`
  (the exact value physically tested and confirmed in
  `../suspended_wheel_diagnostic_20260723/SUMMARY.md`), independently
  capped by the guard at `0.02 m/s` (the existing, already-confirmed
  `hil_frozen_params.json` value, unchanged). The pulse itself reuses
  `hil_wheel_suspension_test.py` exactly as already built and tested
  for the suspended-wheel diagnostic -- no new pulse-generation
  mechanism is introduced.
- **Explicit zero-command hold before and after the pulse** (1.0s
  before, 1.0s after -- `hil_wheel_suspension_test.py`'s own
  `--zero-hold-s`/`--post-hold-s`, reused unchanged).
- **Never runs a second pulse automatically.** The pulse tool exits
  after its one bounded run; nothing in this diagnostic's runbook or
  tooling loops, retries, or re-arms automatically.

## Explicitly excluded from this diagnostic

No virtual peer, no cooperative/CPA avoidance behaviour being
exercised (the frozen `cooperative_avoider.py` is not even started --
the pulse is injected directly on `cmd_vel_unguarded` by
`hil_wheel_suspension_test.py`, exactly as the suspended-wheel
diagnostics already did), no exit search, no obstacle avoidance, no
`HIL_COMM_OFF`/`HIL_COMM_ON` comparison, no Webots. This diagnostic
answers exactly one question: can this real robot, on the ground, be
commanded to a small nonzero speed and back to zero, safely and
completely, under full evidence -- nothing more.

## Human approval gate (binding)

The runbook requires an explicit, verbatim human confirmation
immediately before the nonzero pulse is issued (distinct from and in
addition to the four physical confirmations already required before
any powered session). See `GROUND_DIAGNOSTIC_RUNBOOK.md` step 12.

## What this diagnostic does not decide

Passing this diagnostic does not authorize a ground angular test, a
formal navigation trial, or any change to `hil_frozen_params.json`'s
`field_geometry` or `max_angular_speed_rps`. Those remain governed
entirely by `../HIL_SAFETY_CHECKLIST.md` and
`../HIL_KNOWN_LIMITATIONS_AND_READINESS_20260723.md`, unchanged by this
diagnostic's preparation or outcome.
