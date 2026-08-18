# rm_vision_2027 MCAP topics

## Diagnostic presets

- `performance`: detector preprocess/inference/postprocess/total time; JPEG time, publish latency,
  source-invalid frames, rate-limited frames, and overwritten debug frames.
- `pnp`: latest cumulative truth/detection error summary, solve/refinement counters, and attempt
  source/status counts.
- `prediction`: `dt_s`, NIS, tracker-state counts, reset reasons, and final tracker state.
- `control`: control period, deadline lateness, data age, tracking error, command/failure/feedback
  sources, and fallback activity.
- `all`: run every available preset; report unavailable topic families instead of failing the whole
  report.

`--check` applies the documented detector `total_ms` P95 limit of 16.7 ms and simulation ground-truth
PnP P95 limits: reprojection 0.1 px, position 0.005 m, and rotation 0.5 degrees.

## Useful JSON topics

- `/vision/armor/stats`: `preprocess_ms`, `inference_ms`, `postprocess_ms`, `total_ms`,
  `threshold_candidates`, `kept_detections`.
- `/vision/debug/stats`: `jpeg_encode_ms`, `publish_latency_ms`, `source_invalid_frames`,
  `rate_limited_frames`, `queue_overwritten_frames`.
- `/vision/pnp/stats`: `summary`, `groups`, `solve`, `refinement`, and per-target `attempts[]`.
- `/vision/prediction/state`: `tracker_state`, `dt_s`, `nis`, `associations`, `reset_reason`.
- `/vision/control/state`: nested selection, ballistics, MPC, fire, Talos, actuator, and runtime state.
- `/vision/control/tracking`: scalar reference/MPC/published/estimated/measured comparisons plus
  solver, fallback, feedback, timing, and actuator diagnostics.
- `/vision/control/trajectory`: reference, planned, published, and history arrays.

Field paths are relative to the message `data` object. Arrays use zero-based brackets, for example
`attempts[0].reprojection_rmse_px`.

## Image layers

- `detector`: `/vision/armor/annotations`.
- `pnp` (default): simulation truth annotations, PnP corners/reprojection/error vectors, and corner
  refiner axes/candidates.
- `prediction`: current and future prediction annotations.
- `control`: selected-armor annotations.
- `all`: all annotation topics above; use only when clutter is acceptable.

All vision annotations and `/vision/camera/image` are published with the same frame timestamp. A
missing annotation topic at a selected timestamp is recorded in `manifest.json` and is not by itself
an extraction failure.
