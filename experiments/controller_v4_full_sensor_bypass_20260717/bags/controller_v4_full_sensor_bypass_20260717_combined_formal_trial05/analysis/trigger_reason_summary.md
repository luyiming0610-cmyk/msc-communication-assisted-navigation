# Trigger reason classification

Bag: `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/controller_v4_full_sensor_bypass_20260717/bags/controller_v4_full_sensor_bypass_20260717_combined_formal_trial05`

- trigger_reason: **PROXIMITY_FALLBACK**
- trigger_time_s: 55.224855344
- trigger_distance_m: 0.3383265794827752
- tcpa_at_trigger_s: 4.0
- dcpa_at_trigger_m: 0.2156552224747512
- closing_speed_at_trigger_mps: 0.04002886032345293
- minimum_center_separation_m: 0.27865521286639255

Classification thresholds: PREDICTED_CPA requires tcpa<=4.0s and dcpa<0.14m; PROXIMITY_FALLBACK requires current_distance<0.34m with the predicted condition false. Reconstructed offline from recorded bag state using the same collision_math formulas the live controller runs; the controller itself was not modified or rerun to produce this classification.
