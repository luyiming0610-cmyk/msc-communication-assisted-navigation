# EpuckState protocol freeze — PROTOCOL_VERSION=1

Date: 2026-07-17. This is a documentation-only freeze: no field was added,
removed, or reordered in `EpuckState.msg` to produce this record. It exists
because reprocessing five pre-freeze bags (`head_on_cpa_only_trial_02..06_postfix`,
2026-07-16) through the current installed message type failed with
`Fast CDR exception deserializing message of type
epuck2_comm_interfaces::msg::dds_::EpuckState_` — proving that appending
fields to a `.msg` file, even without touching any existing field, changes
the generated wire type enough to break deserialization of bags recorded
under an earlier field set. The earlier assumption ("appending fields does
not affect old wire data") was wrong and is corrected here.

## Frozen definition fingerprint

- File: `src/epuck2_comm_interfaces/msg/EpuckState.msg`
- `PROTOCOL_VERSION` constant declared in the message: `1`
- Git commit this freeze is anchored to: `06dae306deec7e1a697358a9b6539403378115fb`
- Git blob hash of the file at that commit: `a76b7f1961ec140fa18a6c6ee19a1aba54c1a0a9`
  (`git hash-object src/epuck2_comm_interfaces/msg/EpuckState.msg`)
- SHA-256 of the file's raw bytes:
  `a7ec4184dec52b157a87beea20b44fb2dff5c6dee199d0c76b7c347c26abe15b`
- ROS 2 Humble does not provide a built-in RIHS/type-hash command
  (`ros2 interface show` has no `--hash` option in this distro; that
  mechanism was added in later ROS 2 releases). The git blob hash + SHA-256
  above are therefore the authoritative fingerprint for this project: any
  future `.msg` change that leaves this fingerprint unchanged is guaranteed
  byte-identical, and any change at all should be assumed wire-incompatible
  with bags recorded before it, exactly as observed with the five legacy
  bags above.

## Frozen field list (21 fields, in declared order)

```
uint8 version
uint8 source
uint16 robot_id
uint32 sequence
builtin_interfaces/Time stamp
float32 x_m
float32 y_m
float32 yaw_rad
float32 linear_velocity_mps
float32 angular_velocity_rps
float32 front_distance_m
float32 left_distance_m
float32 right_distance_m
uint8 obstacle_status
uint8 validity_flags
float32 left_front_m
float32 left_mid_m
float32 left_rear_m
float32 right_front_m
float32 right_mid_m
float32 right_rear_m
```

(plus the `PROTOCOL_VERSION`, `SOURCE_*`, `OBSTACLE_*`, `FLAG_*` constants
already declared in the file.)

## Rules from this point forward

1. **This exact field list, in this exact order, is PROTOCOL_VERSION=1.**
   Formal Phase 4 Trials 01-05, all communication-performance experiments
   (delivery ratio, latency/age, sequence-loss, impairment matrix), and the
   physical Pi-puck validation must all run against this identical message
   structure, so their bags remain mutually re-analyzable with one set of
   analyzer scripts.
2. **No field may be added, removed, or reordered in the `PROTOCOL_VERSION=1`
   message.** If a future need genuinely requires a structural change,
   create `PROTOCOL_VERSION=2` as a distinct message type (e.g. a new
   `.msg` file or a clearly-versioned successor) with its own analyzer
   support and its own bags — never silently mutate the frozen v1 message.
   A conversion/compatibility tool between v1 and v2 bags would need to be
   written explicitly if cross-version analysis is ever required.
3. **Bags recorded before this freeze that cannot be deserialized by the
   current installed message type are "pre-protocol-freeze legacy
   evidence."** Specifically: `head_on_cpa_only_trial_02_postfix` through
   `head_on_cpa_only_trial_06_postfix`
   (`experiments/cooperative_avoidance_20260716/bags/`). Their existing
   `summary.json` / `.md` files (produced by the analyzer versions current
   at the time they were recorded, before this freeze) remain valid
   historical engineering evidence and may still be cited as such. They
   must NOT be presented as compatible with, or reprocessed through, the
   PROTOCOL_VERSION=1 wire schema or any analyzer that assumes it (including
   `analyze_trigger_reason.py`) -- that reprocessing step is what actually
   failed, not the original recording or its original analysis.
4. Every bag recorded from this point forward under an unmodified
   PROTOCOL_VERSION=1 message can be assumed wire-compatible with every
   other such bag and with the current analyzer scripts, without needing a
   per-bag compatibility check.
