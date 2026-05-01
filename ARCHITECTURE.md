# Swarm V1 Architecture

## Purpose

This branch is a browser-based, non-weaponized spatial AI simulator for autonomous multi-drone terrain mapping. The system generates synthetic 3D terrain, flies a distributed swarm through it, raycasts LiDAR observations, and records/export point-cloud data for future dataset and reconstruction work.

## Module Map

```text
src/
  main.js              Three.js scenes, terrain generation, UI, mission loop, sensing, rendering, exports
  styles.css           Application layout and viewport styling
  core/
    constants.js       Occupancy and sensor preset constants
    math.js            Numeric helpers, clamps, interpolation, seeded RNG
    planner.js         3D voxel graph search used by the preserved baseline logic
    voxel-grid.js      Voxel graph storage and world/grid coordinate transforms
  swarm/
    behavior-profile.js
                       Parameter registry, normalized behavior weights, objective profiles
    constants.js       Roles, formation modes, terrain classes, AOI types
    drone-agent.js     Per-drone role, route, mailbox, observations, and metrics state
    communication-graph.js
                       K-nearest, range-limited drone-to-drone link model
    formation-graph.js 3D formation target generation and adaptive topology hints
    terrain-classifier.js
                       Lightweight terrain/problem classification scaffold
    task-allocator.js  Utility scoring for frontier, AOI, relay, and verification assignments
    map-fusion.js      Point-cloud tile and mission replay export scaffolding
    resource-model.js  Placeholder drone platform, sensor, compute, and energy accounting
    run-telemetry.js   IndexedDB-backed simulation run telemetry records
    swarm-controller.js
                       Swarm facade that owns agents, comms, formation, allocation, and fusion
```

`main.js` is still intentionally large because the prototype is moving quickly. Extraction targets are listed below so future work can make the project easier to reason about without changing behavior all at once.

## Runtime Data Flow

1. Terrain generation builds a mountain-valley scene, village/tree/vehicle/object props, AOI markers, raycast target meshes, and a coarse preview voxel volume.
2. Mission start initializes the swarm, assigns ground-lattice launch positions, starts the takeoff phase, and resets the point cloud/map state.
3. The animation loop updates camera controls, swarm motion, communication links, formation targets, scan timing, cloud flush timing, and readouts.
4. The Swarm Behavior Kernel updates communication health, adaptive scout/mapper/relay/verifier roles, task assignments, relay waypoints, and smoothed formation targets.
5. Each scan pass asks every active drone to cast its configured LiDAR fan against the scene meshes. AOI focus rays are role-weighted and added only when a drone is close enough to an AOI.
6. Ray hits are voxel-deduplicated into `sim.voxelMap`; each raw voxel stores averaged position, hit count, distance sum, and AOI/focus metadata.
7. Each LiDAR hit also updates a lightweight footprint-coverage estimate. The footprint radius grows with hit distance and scan angular spacing, records approximate unique area, redundancy, and resolution score, and does not create synthetic point-cloud points.
8. Runtime behavior computes normalized signals, behavior weights, derived controls, and an explicit dependency graph from the central behavior profile.
9. Point-cloud rendering samples raw voxel maps up to the visible point cap and uploads positions/colors into reusable typed-array buffers.
10. Swarm telemetry samples behavior weights, derived controls, role counts, network state, AOI proximity, scan totals, path length, placeholder resource use, temporal rate metrics, and validity flags into local IndexedDB run records.
11. PLY export currently uses raw stored points only, not post-processed splats or only visible sampled points.

## Current Performance Bottlenecks

- Scene raycasting scales with `drone count * rays per drone * scan passes`.
- Dense point-cloud capture can create many voxel entries quickly.
- Visible point-cloud uploads are expensive if the whole buffer is rebuilt too often.
- `sim.voxelMap` uses string keys and object entries, which is convenient but not ideal for very dense survey-scale capture.
- AOI focus scanning is useful but can become a local ray multiplier if the swarm crowds an object field.
- `main.js` combines terrain generation, UI, sensing, rendering, and controls, which makes isolated profiling harder.

## Current Optimizations

- `three-mesh-bvh` accelerates mesh raycasting without giving drones privileged terrain knowledge.
- Scan density presets keep per-drone fan sizes bounded while preserving the rule that every drone scans on each scan pass.
- Performance budgets control render scale, visible point cap, scan cadence, point-cloud flush cadence, comms redraw cadence, graph redraw cadence, and visible beam count.
- Point-cloud rendering now reuses typed arrays and `BufferGeometry` attributes instead of allocating new arrays and attributes every flush.
- Swarm scan direction sets are cached by ray count and FOV.
- Formation targets and drone steering are smoothed so topology changes do not snap the swarm between graph states.
- Relay tasks and network-envelope corrections bias motion back toward reliable K-nearest communication without treating comm links as formation lines.
- Live diagnostics report FPS/frame time, scan pass time, rays/hits per pass, cloud flush time, visible points, and active budget.
- The telemetry panel reads persistent IndexedDB run records and can export saved runs as JSON for later analysis.
- Telemetry run lifecycle is experiment-safe: pause/resume keeps a run open, Stop/reset/completion saves it, live-tunable controls are logged without reinitializing the swarm, and experiment-defining controls clear the map before a new run.
- The Algorithm panel exposes the live controller state: normalized signals, behavior weights, derived controls, role counts, and objective profile.

## Swarm Behavior Kernel V1

- The controller now rebalances scout, mapper, relay, and verifier roles from communication health, frontier density, terrain class, and AOI presence.
- Task allocation scores frontier, AOI, relay, and verification tasks with role-specific weights and a small continuity bias to reduce assignment churn.
- Movement blends graph formation targets with role assignments, then applies velocity smoothing, terrain clearance, object avoidance, drone separation, and network-envelope corrections.
- AOI sensing is focused most strongly by mappers and verifiers, while relay drones preserve lighter sensing and stay biased toward network continuity.

## Important Algorithm Design Direction

`SWARM_ALGORITHM.md` is the canonical design record for the swarm behavior model. This architecture file describes where runtime modules live and how data moves through the simulator; the algorithm file describes the controller math, parameter dependency model, optimization rationale, and design-history reasoning.

Behavior constants now begin moving through `src/swarm/behavior-profile.js`, which owns the default behavior profile, objective profiles, normalized behavior-weight calculation, role target calculation, and adaptive neighbor target calculation. Runtime swarm control should follow an explicit pipeline:

```text
state_t -> normalized signals -> behavior weights -> derived controls -> action_t -> state_t+1
```

Nested parameter dependencies are allowed only when they are explicit, logged, and acyclic within a single timestep. Optimizers should tune parameter profiles offline, while the runtime controller remains locally adaptive to terrain, AOIs, communication health, and safety constraints.

The controller now exposes a controller-state snapshot with:

- normalized state signals;
- simplex-normalized behavior weights;
- derived runtime controls for formation spread, role speed, assignment pull, sensing focus, and network compliance;
- a dependency graph describing the same-timestep flow from signals to actions.

Soft constraints are now part of the normalized signal layer. AOI proximity risk, mission-time pressure, battery reserve pressure, and compute pressure can increase avoidance or efficiency pressure while still leaving AOI/coverage/relay behavior to compete inside the behavior mixture. These are not hard physics contacts; they are risk-aware control signals for the swarm algorithm.

Saved runs are scored with the active objective profile. The current score components are AOI quality, coverage, network resilience, time efficiency, compute efficiency, energy/path proxy, adaptation smoothness, and constraint safety. Scores retain raw measures, temporal measures, normalization targets, and confidence metadata. Invalid runs remain in the dataset with validity flags, but their total score is suppressed so incomplete simulations do not win comparisons.

`src/swarm/behavior-profile.js` now also owns the first evaluation profile and optimizer-parameter registry. The evaluation profile centralizes scoring normalizers and validity thresholds. The optimizer registry deliberately exposes only a small high-impact parameter set, so future Bayesian optimization searches the shape of the adaptive controller rather than all constants at once.

Coverage scoring blends point-voxel growth with footprint area. Footprint metrics approximate how much terrain/object surface the LiDAR sampled at the requested angular resolution and how much of that footprint was redundant overlap. This keeps area coverage measurable without adding derived splat points to the live cloud.

Footprint redundancy also feeds back into formation topology. When multiple drones repeatedly scan overlapping footprint buckets, the controller raises area-spread pressure and expands X/Z formation radius with a smaller vertical-spacing adjustment. When footprint resolution is low, detail pressure can partially tighten spacing so area and resolution behave as opposing factors instead of both being rewarded blindly.

`src/swarm/resource-model.js` is the first explicit home for physics-adjacent accounting. It currently estimates hover, motion, sensor, compute, communication, total energy, and battery remaining from placeholder parameters. These values are suitable for schema design and relative smoke testing only; a later calibration pass should replace the placeholder defaults with sourced drone, battery, sensor, and onboard-compute data.

Telemetry comparison can re-score saved runs under the currently selected objective profile, filter by AOI scenario or validity, then sort by total score, confidence, or individual subscores. This keeps raw run data persistent while allowing the evaluation target to change as the research question changes.

## Extraction Targets

- `src/render/point-cloud-buffer.js`: visible cloud buffer allocation, sampling, coloring, and upload.
- `src/sensing/lidar-scanner.js`: scan direction cache, AOI focus ray generation, raycast result aggregation.
- `src/environment/terrain-builder.js`: terrain mesh, village/tree/AOI/object placement, raycast target registration.
- `src/controls/viewport-controls.js`: shared free-look navigation for simulation and point-cloud viewports.
- `src/export/point-cloud-export.js`: PLY and future tiled dataset export preparation.
- Web Worker path for non-render aggregation/export prep. Three.js scene raycasting should remain on the main thread until a worker-safe geometry representation exists.

## Do Not Regress

- Keep the posted single-drone baseline preserved on `main`.
- Keep `swarm-v1` focused on non-weaponized spatial intelligence and dataset capture.
- Every active drone must scan on each scan pass; performance tuning should reduce aggregation/upload/render cost rather than silently skipping drones.
- LiDAR must raycast against discovered/rendered scene geometry, not analytic terrain shortcuts that give the swarm hidden knowledge.
- Export should use the full stored cloud, not only visible sampled points.
- Communication links represent usable K-nearest network edges inside reliable range, not formation lines.
