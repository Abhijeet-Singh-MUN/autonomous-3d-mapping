# Research Roadmap

This file summarizes the current research direction for `swarm-v1` and keeps the implementation roadmap separate from day-to-day development notes.

## Current Position

The project is now a browser-based experimental platform for interpretable autonomous swarm mapping. The strongest near-term research contribution is the grey-box policy-coordinate model:

```text
base policy coordinates psi
  -> state-dependent runtime nudges
  -> effective psi
  -> derived behavior profile
  -> behavior weights / controls
  -> drone actions
```

The active model family is `greybox-policy-v1` / `greybox-policy-v1.0`. It exposes four interpretable policy coordinates:

- `coverage_area`
- `aoi_detail`
- `risk_safety`
- `resource_efficiency`

The lower-level gains, role shares, sensing multipliers, movement limits, network constraints, and formation controls remain active, but they are derived through documented coupling coefficients instead of becoming independent optimizer knobs.

## Fastest Publishable Direction

The fastest serious research path is:

**Grey-box policy coordinates for interpretable multi-drone swarm behavior.**

The required implementation is mostly in place:

- persistent run telemetry
- Pareto vector scoring with fixed equal-weight evaluator for fair comparison
- validity filtering
- temporal metrics
- batch runner
- runtime nudge calibration
- per-run control snapshots
- sparse per-drone trajectories
- JSON and CSV export
- dataset workspace tagging

The main missing work is systematic data collection and external statistical analysis.

## Data Collection Sequence

1. Run a first nudge-calibration workspace:

```text
workspace: greybox-calibration-v1
batch mode: nudge calibration
runs: 4 policy presets x 5 nudge profiles = 20
```

2. Inspect batch summaries and exported CSV/JSON.

3. Freeze one default runtime nudge profile.

4. Run Pareto-only batches:

```text
batch mode: pareto only
runs per batch: 4 policy presets x 1 fixed nudge profile
```

5. Repeat across seeds and AOI scenarios:

```text
4 policy presets x 3 seeds x 3 AOI scenarios = 36 runs
```

This is the first useful dataset size. It is not enough for final claims, but it is enough to check whether policy coordinates produce statistically visible behavior changes.

Use fixed-time budgets for this calibration phase. Start with `120s` nudge-calibration runs, use `45s` only for smoke/debug runs, and test `180s+` Pareto/AOI-heavy runs only if exported validity metrics show weak AOI contact or dwell. Do not implement auto-duration yet; compare within each AOI scenario and keep cross-AOI distance bias visible in the dataset through AOI distance, contact time, dwell time, path length, and validity fields.

## Metrics Readiness

Current metrics are strong enough for first-pass grey-box calibration and pilot research:

- fixed score/loss, psi-weighted diagnostic score/loss, and Pareto vector
- role entropy as a behavioral organization signature
- coverage, AOI detail, risk safety, and resource efficiency scores
- scan totals and point voxels
- unique/redundant LiDAR footprint area
- AOI dwell/contact/hit metrics
- network health and fragmentation
- energy/compute placeholder estimates
- behavior-weight change rate
- runtime nudge/effective psi deltas
- sparse per-drone trajectory vectors
- validity flags

They are not yet enough for strong final research claims without:

- repeated seeds
- controlled scenario variation
- external statistical tests
- reconstruction-quality metrics if using 3DGS/NeRF
- calibrated drone energy/sensor constants
- dynamic stressor scenarios

Validity flags currently include `insufficient_scan_time`, `target_not_reached`, `insufficient_aoi_dwell`, `low_raw_aoi_coverage`, `performance_unstable`, and `batch_cancelled`. These runs should remain stored; external analysis can filter them out or inspect them as failure cases.

The role model is good enough for pilot data, but it is not yet a strong expert-mixture story. Every drone still performs baseline mapping. Future research-quality behavior should make scout, mapper, relay, and verifier controls more distinct while keeping a shared mapping minimum for navigation and evidence quality.

## Bayesian / GP Path

Bayesian or GP optimization should start after nudge calibration, not before. The optimizer should see only a small policy surface first:

```text
coverage_area
aoi_detail
risk_safety
resource_efficiency
```

The optimizer target should be a configurable loss:

```text
fixed loss = equal-weight distance from ideal Pareto vector
psi-weighted loss = policy-aligned diagnostic distance from ideal Pareto vector
```

The browser simulator should continue to collect and export runs. External Python tooling should handle GP fitting, plotting, sensitivity analysis, and next-candidate selection.

## Dataset/File Management

The browser cannot reliably write directly to arbitrary folders without a user file picker, so the project uses a workspace layer:

- `Dataset workspace` tags new telemetry runs.
- Saved workspaces are stored in browser `localStorage`.
- Telemetry can filter by active workspace, all workspaces, legacy runs, or a saved workspace.
- JSON and CSV export filenames include the workspace name.

This gives us clean local campaign separation without deleting old IndexedDB records.

## Research Directions

### 1. Grey-Box Policy Coordinates

Near-term and lowest implementation risk. Run many batches, analyze whether the four coordinates produce distinct, interpretable swarm behaviors.

### 2. Swarm Point Cloud To Reconstruction

Use exported PLY point clouds for 3D Gaussian Splatting, NeRF, or point-cloud reconstruction. Compare whether adaptive AOI-focused scanning improves reconstruction quality.

### 3. Swarm Latent-Space Explorer

Project neural network activations or embeddings into 3D and let the swarm map that representation space as a terrain. This is important and novel, but needs a Python/PyTorch bridge.

### 4. World-Model Surrogate

Use many runs to train a model that predicts future coverage/AOI/network outcomes, then use that model as a cheaper optimizer surrogate.

### 5. Emergent Communication / MARL

Later-stage direction. Current communication is rule-based; a learned GNN or MARL controller could be compared against the grey-box baseline.

### 6. Topological Behavior Analysis

Sparse per-drone trajectories make it possible to classify swarm behavior using formation topology, persistent homology, or trajectory clustering.

## Future Semantic Dataset Direction

After initial nudge and Pareto calibration, the terrain generator should evolve into a semantic object relational map:

- Generate a small set of meaningful scene families, not arbitrary random placements.
- Keep objects plausible: trees should not grow in rivers, buildings should not spawn on unrealistic peaks, and access paths/clearings should make sense.
- Attach semantic labels to scan hits and point-cloud records when the hit object is known.
- Export semantic point clouds for later language-aware 3DGS and natural-language 3D generation experiments.
- Add adversarial non-combat stressors only after the baseline semantic dataset is stable.

## Safety Boundary

The simulator can include adversarial non-combat stressors:

- moving obstacles
- deliberate occlusion
- degraded commercial communications
- GPS/sensor dropouts
- emergency or fast landing behavior
- high-risk AOI proximity
- cluttered or deceptive environments

Excluded scope remains:

- strike behavior
- target selection
- weapons
- military autonomy
- harmful jamming instructions
