# Trigger reason classification

Bag: `/mnt/c/Users/路一鸣/Desktop/硬件实验毕设/e-puck2-Comm/experiments/controller_v4_full_sensor_bypass_20260717/bags/controller_v4_full_sensor_bypass_20260717_head_on_cpa_pure_c1`

- trigger_reason: **PROXIMITY_FALLBACK**
- trigger_time_s: 11.22020866
- trigger_distance_m: 0.33864879608154297
- tcpa_at_trigger_s: 4.0
- dcpa_at_trigger_m: 0.1402008011937148
- closing_speed_at_trigger_mps: 0.049611998721957117
- minimum_center_separation_m: 0.14164457863579974

Classification thresholds: PREDICTED_CPA requires tcpa<=4.0s and dcpa<0.14m; PROXIMITY_FALLBACK requires current_distance<0.34m with the predicted condition false. Reconstructed offline from recorded bag state using the same collision_math formulas the live controller runs; the controller itself was not modified or rerun to produce this classification.
