# Swarm Algorithm Design Rationale

This document is the canonical design record for the swarm behavior algorithm. It captures the reasoning behind the current direction, the mathematical model we intend to use, the role of parameter dependencies, and how future optimization should tune the system without replacing local adaptive behavior.

`ARCHITECTURE.md` owns runtime system placement and data flow. This file owns controller math, parameter dependencies, objective functions, algorithm comparisons, and reasoning history.

## Core Question

The central question is how to make the swarm behavior continuous, adaptive, measurable, and tunable without turning it into a black box.

The current direction is:

- keep runtime behavior interpretable;
- treat fixed formations as baselines or behavior primitives, not the final algorithm;
- compute smooth behavior mixtures from measured environment and swarm state;
- log enough run data to compare behavior profiles scientifically;
- tune parameter profiles offline while preserving local real-time adaptation.

## Reasoning Timeline

### 1. Measuring Swarm Quality

Initial question: how do we know which formation or behavior is better for a given AOI distance, AOI type, scan duration, terrain condition, or communication condition?

Answer: add persistent simulation logging and comparison metrics. The simulator should record completed and incomplete runs, normalize comparable scenarios, and filter bad runs such as target-not-reached or insufficient AOI dwell time.

Direction change: the work expanded from "make the swarm behave better" to "make the swarm measurable before claiming improvement."

### 2. Persistent Dataset And Validity Filters

Follow-up question: what fields should be recorded so different formations, adaptations, and sensing settings can be compared?

Answer: each run should capture scenario config, AOI type and distance, scan time, time to reliable AOI map, raw rays, raw hits, derived points, area mapped reliably, network health, path length or energy proxy, run validity flags, and short qualitative behavior notes.

Direction change: evaluation became a durable local dataset instead of only live UI diagnostics.

### 3. Sensor Footprint Splatting

Question: can every real ray hit create nearby points so the point cloud becomes denser without casting 9-10x more rays?

Answer: yes, but only as a derived sensor-footprint layer. Synthetic neighbor points should not be counted as raw LiDAR evidence. Raw hits and derived points must remain separate in storage, metrics, exports, and comparison scores.

Direction change: the idea changed from "multiply point count" to "model finite sensor footprint while preserving measurement integrity."

### 4. Continuous Behavior Space

Question: can swarm behavior be represented as a smooth function with tunable parameters, where specific regions or states correspond to behavior styles?

Answer: yes. The cleaner model is a continuous mixture of behavior primitives. A "mode" becomes a region in behavior space rather than a hard if/else switch.

Direction change: fixed formations became baselines and possible primitives, while the intended controller became a continuous adaptive mixture.

### 5. Parameter Dependencies

Question: what if one parameter group affects another, such as communication health changing role balance, movement freedom, and AOI focus?

Answer: nested dependencies are not inherently bad. They become problematic only if hidden, circular, or scattered unpredictably. Dependencies should be explicit, directional, logged, and acyclic within a single timestep.

Direction change: the parameter plan now requires a dependency graph instead of a flat knob list.

### 6. Optimization Versus Runtime Adaptation

Question: if optimization tunes parameters, could it over-force behavior and fight local adaptive needs?

Answer: optimization should tune profiles and ranges offline. The runtime controller should still adapt from local state at each timestep. Objective profiles must be documented and versioned because they may change as the drone model, energy proxy, resilience metrics, and mission priorities improve.

Direction change: optimizers are treated as profile-tuning tools, not direct real-time swarm controllers.

## Mathematical Model

### Grey-Box Policy Coordinates

Current model family:

```text
modelFamily = greybox-policy-v1
modelVersion = greybox-policy-v1.0
```

The optimizer-facing search space is now intentionally small:

```text
psi = {
  coverage_area: 0.35,
  aoi_detail: 0.30,
  risk_safety: 0.20,
  resource_efficiency: 0.15
}

sum(psi_i) = 1
0 <= psi_i <= 1
```

This does not remove the low-level parameters. It changes their role. Behavior gains, role shares, task weights, sensing multipliers, network thresholds, formation spread, and soft-constraint gains remain active in the controller, but most are now derived adaptive values under `src/swarm/behavior-profile.js`.

The hierarchy is:

```text
policy coordinates psi
  -> documented structural coupling coefficients k
  -> derived behavior profile
  -> live behavior weights and derived controls
  -> swarm actions
```

The first coupling model is hand-authored, bounded, and monotonic:

```text
low_level_param =
  base
  + k_coverage * psi.coverage_area
  + k_aoi      * psi.aoi_detail
  + k_risk     * psi.risk_safety
  + k_energy   * psi.resource_efficiency
```

The initial `k` values are engineering priors, not learned truth. Later calibration can fit or adjust selected `k` coefficients from small simulation batches instead of exposing every low-level constant to the optimizer.

Let:

```text
s_t = current normalized environment, swarm, and mission state
theta = tunable parameter profile
w_t = f(s_t, theta)

w_i >= 0
sum(w_i) = 1

intent_t = sum_i w_i * primitive_i(s_t)
```

Examples of normalized state signals:

- AOI distance and AOI pressure;
- AOI dwell time and AOI raw coverage;
- frontier density;
- obstacle density;
- openness and corridor score;
- verticality;
- communication health;
- connected components;
- average graph degree;
- scan progress.

Examples of behavior primitives:

- coverage;
- frontier exploration;
- AOI sensing;
- relay/network preservation;
- verification/rescan;
- terrain/object/drone avoidance.

The behavior weights live on a simplex: each weight is non-negative and all weights sum to `1`. This does not mean the whole swarm algorithm is a simple linear function. It means the high-level intent is a normalized mixture of primitives, and each primitive can still contain nonlinear logic.

## Parameter Dependency Model

Parameters should be organized into three layers:

```text
base parameters + measured state signals -> derived parameters -> runtime controls
```

Base parameters are manually selected or optimizer-tuned values, such as role share limits, movement gains, sensing gains, and network pressure gains.

Measured state signals come from the current simulation, such as AOI pressure, communication health, frontier density, scan progress, or obstacle density.

Derived parameters are computed values, such as current role ratios, adaptive neighbor targets, AOI focus strength, relay pressure, and movement freedom.

Runtime adaptation should follow:

```text
state_t -> normalized signals -> behavior weights -> derived controls -> action_t -> state_t+1
```

Soft constraints are part of `state_t`, not a separate override layer. Current soft constraints include AOI proximity risk, mission-time pressure, battery reserve pressure, and compute pressure. They are allowed to tilt the behavior mixture toward avoidance or efficiency, but they should not erase local AOI/coverage/network needs unless a future hard safety rule explicitly requires that.

Allowed:

- communication health increases relay pressure;
- AOI pressure increases mapper/verifier focus;
- poor link health increases adaptive neighbor targets;
- dense obstacles increase avoidance and reduce scout freedom.

Not allowed:

- hidden same-timestep circular dependencies;
- scattered constants that mutate each other unpredictably;
- optimizer code overriding local safety, terrain clearance, or communication constraints.

Feedback over time is expected. Circular mutation within one timestep is not.

## Optimization Direction

Objective functions should be documented and versioned because they will change.

Potential objective profiles:

- balanced mapping;
- AOI quality;
- network resilience;
- energy/path efficiency;
- dense reconstruction;
- compute efficiency.

Optimization should tune `theta` or the smaller `psi` policy-coordinate profile. It should not directly force real-time actions. The runtime behavior function still responds to local state.

For `greybox-policy-v1`, the scalar display score is framed as distance from an ideal Pareto vector:

```text
pareto = {
  coverage_area,
  aoi_detail,
  risk_safety,
  resource_efficiency
}

loss = sum_i psi_i * abs(1 - pareto_i)
score = 1 - loss
```

The run record stores both the Pareto vector and the scalar score/loss. This lets the UI sort runs while preserving why a run was good or bad. The older named score components still exist as diagnostics and can be reweighted for specific research questions.

Telemetry is namespaced by model family. New runs save `greybox-policy-v1` and `greybox-policy-v1.0`; the telemetry panel defaults to compatible runs only. Older IndexedDB records remain stored but do not pollute new-model analysis unless a future UI explicitly opts into cross-family comparison.

The current optimizer-facing parameter set is intentionally small. `OPTIMIZER_PARAMETER_REGISTRY` now exposes only the four policy coordinates. `LOW_LEVEL_PARAMETER_REGISTRY` documents the derived low-level parameters and their coupling bounds, but those are not direct optimizer knobs in this model family. This avoids an underdetermined search where many parameter combinations can look equally good against too few metrics.

Temporal metrics are now part of the evaluation direction. Each run still has end-of-run aggregates, but sampled rates capture the curve of behavior over time:

```text
theta -> runtime trajectory -> M_t -> temporal summary -> J(theta)
```

Examples include new voxels per second, useful AOI hit rate after contact, energy and compute rate, network fragmentation duration, coverage growth, and behavior-weight change rate. This lets future optimization reward fast adaptation and penalize oscillation instead of only scoring final totals.

LiDAR coverage is now treated as more than point count. A ray hit has an approximate footprint:

```text
footprint_radius ~= tan(max(horizontal_step, vertical_step) / 2) * ray_distance
```

That radius is clamped and focus rays use a smaller scale. The simulator records approximate unique footprint area, redundant overlap, and resolution score. This captures the tradeoff between broad low-resolution coverage and close high-resolution detail without adding synthetic splat points to the raw point cloud.

The area/detail relationship is deliberately inverse. Larger footprints increase covered area but reduce `resolutionScore`; smaller footprints increase detail but cover less surface. Redundant overlap is computed across all drones through shared footprint buckets, so inter-drone ray closeness can influence both scoring and future motion.

The current run reward is not a single mysterious score. It is a weighted objective profile over named components:

```text
J(theta) =
  weighted_sum(
    AOI quality,
    coverage,
    network resilience,
    time efficiency,
    compute efficiency,
    energy/path efficiency,
    adaptation smoothness,
    constraint safety
  )
```

Objective profiles change the weights, not the stored run data. That means the same run can be re-scored later under a different research question.

The intended path is:

1. Manual profile comparison through logged simulations.
2. Random or Latin-hypercube sampling for sensitivity baselines.
3. Bayesian optimization once the tunable parameter set is reduced and simulations are expensive.
4. PSO if broader continuous search is useful and enough simulation budget exists.
5. RL only after metrics, datasets, and interpretable baselines are mature.

Bayesian optimization may become preferable because terrain/AOI/drone-count sweeps are expensive. Random or Latin-hypercube sampling remains useful early because it reveals rough sensitivity and bad parameter regions without assuming the objective is stable.

## Algorithm Comparison

These approaches are not mutually exclusive:

- Boids-like rules are useful for local motion primitives such as separation, cohesion, alignment, and obstacle avoidance.
- PSO is useful as an offline parameter optimizer, not necessarily as the real-time swarm behavior controller.
- Bayesian optimization is useful when simulation runs are expensive and the number of high-impact parameters has been reduced.
- RL may eventually learn policies, but it needs stable rewards, a large dataset, and strong baselines. It is too early to make it the core controller.

The current direction is an interpretable adaptive controller first. That keeps the swarm explainable while we build the dataset needed to compare future algorithms fairly.

## How This Differs From The Earlier Plan

The earlier plan described the future adaptive algorithm direction: a continuous controller, parameter registry, telemetry, dashboard, and optimizer path.

This document records why that direction was chosen. It preserves the reasoning trail, the mathematical interpretation, the parameter-dependency concern, and the optimization concern so future changes can be evaluated against the original intent.

## Open Questions

- Which objective profiles should be implemented first?
- Which metrics define "reliable AOI mapping" for each AOI type?
- What energy proxy should be used before a more realistic drone model exists?
- Which real platform, battery, LiDAR, camera, communication, and onboard compute specs should calibrate the placeholder resource model?
- How should resilience combine link loss, recovery time, connected components, and information traversal?
- How small should the optimizer-tunable parameter set be before Bayesian optimization begins?
- When, if ever, should RL be introduced as more than a comparison baseline?
- How should post-scan derived sensor-footprint splats be generated and visualized without encouraging them to be mistaken for raw evidence?

## Current Baseline

The current Swarm Behavior Kernel V1 remains the baseline-under-test until instrumentation and evaluation are added. Further behavior changes should be measured against this baseline rather than judged only by visual appearance.

The first implementation step toward this design is `src/swarm/behavior-profile.js`. It centralizes behavior parameters, objective profiles, behavior-weight calculation, role target calculation, and adaptive neighbor target calculation. The current runtime still uses the existing controller structure, but the major swarm behavior constants now have a named profile home instead of living as isolated literals across the mission loop.

The second implementation step is `src/swarm/run-telemetry.js`. It persists swarm run records to IndexedDB with scenario metadata, behavior profile version, behavior-weight samples, role counts, network health, adaptive neighbor target, raw scan totals, AOI proximity, path length, energy proxy, and validity flags.

The third implementation step is an explicit controller-state pipeline. The swarm controller now computes normalized signals, behavior weights, derived controls, and a dependency graph each update. Runtime motion, assignment blending, AOI focus, network compliance, adaptive neighbor count, and formation spread consume those derived controls instead of reading only fixed constants.

The fourth implementation step adds live algorithm visibility and objective scoring. The UI can inspect the current behavior weights, normalized signals, derived controls, role counts, and active objective. Telemetry records score components for AOI quality, coverage, network resilience, time efficiency, compute efficiency, and energy/path proxy, along with raw measures, normalization targets, and score confidence. Invalid runs remain useful diagnostic records, but their total objective score is suppressed.

The fifth implementation step adds `src/swarm/resource-model.js` as a physics-adjacent accounting layer. It estimates hover, motion, sensor, compute, communication, total energy, and battery remaining from explicit placeholder parameters. This improves the telemetry schema now while keeping the values clearly marked as uncalibrated until a sourced hardware pass replaces the defaults.

The sixth implementation step adds temporal evaluation. `src/swarm/run-telemetry.js` now stores per-sample delta/rate metrics and a per-run temporal summary. `src/swarm/behavior-profile.js` now centralizes evaluation normalizers, validity thresholds, and the first optimizer parameter registry, which later evolved into the four-coordinate grey-box policy registry. Scoring now blends final totals with curve-derived measures such as AOI hit rate after contact, new voxel rate, network fragmentation duration, energy/compute rate, and behavior-weight smoothness.

The seventh implementation step adds soft constraint signals into the behavior mixture. AOI proximity risk, mission-time pressure, battery reserve pressure, and compute pressure now appear in normalized signals. They can raise avoidance and efficiency pressure, expand formation spacing slightly under risk, reduce movement freedom under risk, and contribute to a `constraintSafety` score through temporal risk exposure.

The eighth implementation step adds lightweight LiDAR footprint coverage metrics. Each raw hit estimates a distance-scaled footprint radius from scan angular spacing. Telemetry records total footprint area, approximate unique footprint area, redundant footprint area, footprint redundancy ratio, and a resolution score. Coverage scoring now blends raw voxel growth with footprint coverage and penalizes excessive overlap.

The ninth implementation step feeds footprint redundancy back into topology. High inter-drone footprint overlap raises area-spread pressure, expanding formation radius and slightly increasing vertical separation. Low footprint resolution creates detail-tightening pressure that can partially counteract spreading. This makes area coverage and scan resolution competing influences on swarm spacing rather than passive after-run metrics.

Runtime sensor-footprint splatting was tested and removed from the live swarm loop. The current point cloud is raw LiDAR voxel evidence only in `sim.voxelMap`; 9-neighbor/footprint splats are deferred to a future post-scan reconstruction step after Stop/completion/export so synthetic density does not compete with raycasting, control updates, telemetry, and rendering.

The tenth implementation step is the grey-box policy-coordinate refactor. The model family is `greybox-policy-v1`; the model version is `greybox-policy-v1.0`. The optimizer-facing surface is now `coverage_area`, `aoi_detail`, `risk_safety`, and `resource_efficiency`, while low-level controller parameters are derived through documented `k` coupling coefficients. Telemetry, scoring, exports, and the Algorithm panel now carry the model namespace, policy coordinates, derived profile summary, Pareto vector, scalar score, and scalar loss.

The eleventh implementation step exposes those four policy coordinates in the Mission Controls panel. The slider values are normalized before deriving the behavior profile, so users can quickly test policy emphasis changes while telemetry records the normalized policy vector and the full raw control snapshot for fair later filtering.

The twelfth implementation step adds a small policy batch runner. It is not Bayesian optimization yet; it is a controlled comparison harness that runs four interpretable presets against the current scenario for a fixed duration and saves each run through the same telemetry/scoring path. This gives us a low-friction sensitivity baseline before spending time on GP surrogate or Bayesian search.

For fast iteration, the default runtime configuration is Economy scan density with the Capture performance budget. Richer semantic object graphs, obstacle-heavy scenes, and adversarial terrain conditions are deferred to the dataset phase; those scenarios should expand recorded fields and feed risk/safety pressure rather than becoming undocumented behavior overrides.
