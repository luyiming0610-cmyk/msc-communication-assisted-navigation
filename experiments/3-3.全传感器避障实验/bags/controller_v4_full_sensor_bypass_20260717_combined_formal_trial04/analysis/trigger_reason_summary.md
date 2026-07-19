# Trigger reason classification

Bag: `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/controller_v4_full_sensor_bypass_20260717/bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial04`

- trigger_reason: **PROXIMITY_FALLBACK**
- trigger_time_s: 54.581770597
- trigger_distance_m: 0.3397459909048517
- tcpa_at_trigger_s: 4.0
- dcpa_at_trigger_m: 0.21311593457037126
- closing_speed_at_trigger_mps: 0.03988268205009123
- minimum_center_separation_m: 0.27766531493506424

Classification thresholds: PREDICTED_CPA requires tcpa<=4.0s and dcpa<0.14m; PROXIMITY_FALLBACK requires current_distance<0.34m with the predicted condition false. Reconstructed offline from recorded bag state using the same collision_math formulas the live controller runs; the controller itself was not modified or rerun to produce this classification.
