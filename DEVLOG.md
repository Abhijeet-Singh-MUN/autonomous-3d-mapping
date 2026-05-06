# Swarm V1 Development Log

This log captures why changes are made, which issues they address, and what they unlock for the later README and research narrative.

## 2026-05-02

### Grey-Box Policy Coordinate Refactor

- Changed the default scan density to `Economy` and performance budget to `Capture` for faster policy-batch simulations.
- Grouped policy-batch telemetry runs by batch id and displayed preset labels directly in the telemetry panel.
- Added `greybox-policy-v1` / `greybox-policy-v1.0` as the active swarm algorithm model namespace.
- Replaced the optimizer-facing low-level parameter registry with four Pareto policy coordinates: `coverage_area`, `aoi_detail`, `risk_safety`, and `resource_efficiency`.
- Kept low-level behavior gains, role shares, sensing multipliers, task weights, network thresholds, and footprint controls active as derived parameters instead of deleting them.
- Added documented `k` coupling coefficients so `psi -> derivedProfile -> controller` is explicit, bounded, and inspectable.
- Added mission-control sliders for the four policy coordinates so grey-box behavior can be varied without editing code.
- Added a first policy batch runner that cycles through balanced, coverage-heavy, AOI-detail-heavy, and safe-efficient policy presets for a fixed duration and saves each as normal telemetry.
- Updated scoring to store a Pareto vector, scalar loss from ideal scores, scalar display score, and the older diagnostic score components.
- Namespaced telemetry by model family so old IndexedDB runs remain stored but are hidden from the current grey-box analysis/export by default.
- Updated the Algorithm panel to show model identity, active policy coordinates, and the derived profile summary.

### Resource Metrics Scaffold

- Added `src/swarm/resource-model.js` as the first explicit home for drone platform, battery, sensor, compute, and communication accounting.
- Telemetry now stores hover, motion, sensor, compute, communication, total energy, and estimated battery remaining fields alongside the older path-length energy proxy.
- Updated objective scoring so the energy/path component uses total estimated Wh when available, falling back to the older path proxy for existing runs.
- Kept the new model clearly marked as placeholder calibration data; the schema is useful now, but the constants need a later sourced hardware pass before making real-world claims.
- Restored the point-cloud overlay to show `shown / total pts` in the compact window header while shortening the text so small overlays are less likely to clip the total.

### Temporal Evaluation Scaffold

- Added per-sample temporal metrics for new voxels/sec, raw hits/sec, AOI hits/sec, focused hits/sec, path meters/sec, energy Wh/sec, compute Wh/sec, coverage gain/sec, behavior-weight change/sec, network fragmentation, and AOI-active seconds.
- Finished runs now save a `temporalSummary` so comparison can use the shape of the run, not just the final totals.
- Added `DEFAULT_SWARM_EVALUATION_PROFILE` to centralize scoring normalizers and validity thresholds.
- Added `OPTIMIZER_PARAMETER_REGISTRY` with a deliberately small high-impact parameter set for future Bayesian optimization.
- Updated objective scoring to blend final aggregates with temporal measures, including useful AOI hit rate after contact, coverage growth rate, network fragmented duration, energy/compute rate, and adaptation smoothness.

### Soft Constraint Layer

- Added normalized soft-constraint signals for AOI proximity risk, mission-time pressure, battery reserve pressure, and compute pressure.
- Wired risk pressure into the behavior mixture so close AOI work can increase avoidance/dodging pressure while still preserving AOI sensing pressure.
- Added efficiency pressure from mission time, battery reserve, and compute load so the controller can reduce exploratory spread as constraints tighten.
- Added temporal risk exposure and a `constraintSafety` score component so optimization can penalize risky behavior instead of only rewarding point accumulation.
- Expanded the optimizer registry with first-pass constraint parameters for AOI risk gain, risk avoidance boost, and battery efficiency gain.

### LiDAR Footprint Coverage Metrics

- Added lightweight distance-scaled LiDAR footprint estimates for each ray hit without creating synthetic point-cloud samples.
- The footprint radius is derived from ray distance and scan angular spacing, with tighter focused rays and clamped radius bounds.
- Telemetry now records total footprint area, approximate unique footprint area, redundant footprint area, footprint redundancy ratio, and resolution score.
- Coverage scoring now blends raw point-voxel growth with footprint area coverage and applies a redundancy penalty when drones repeatedly scan overlapping footprints.
- Fed inter-drone footprint redundancy back into formation topology: high overlap expands area-coverage spacing, while low resolution creates a competing detail-tightening pressure.

## 2026-05-01

### Swarm Behavior Kernel V1

- Added adaptive role rebalancing for scout, mapper, relay, and verifier drones from communication health, frontier density, terrain class, and AOI presence.
- Expanded task allocation to score frontier, AOI, relay, and verification work with role-specific utilities and continuity bias.
- Added relay waypoint generation between the launch/base side and current mission center so network-aware movement has explicit task anchors.
- Smoothed formation topology with per-agent target memory and smoothed visible motion with velocity steering before terrain/object/drone/network constraints are applied.
- Role-weighted AOI focus sensing now gives mappers and verifiers stronger close-range AOI sampling while relay drones keep lighter focus and preserve communication continuity.
- Preserved the all-drone scanning contract: every active drone still casts its base LiDAR fan on every swarm scan pass.

### Swarm Algorithm Reasoning Record

- Added `SWARM_ALGORITHM.md` as the canonical design/rationale file for the swarm controller math, parameter dependency model, optimization direction, and reasoning history.
- Recorded the discussion progression from fixed formation/AOI comparisons to persistent evaluation datasets with normalized metrics and bad-run filtering.
- Documented the decision that 9x neighbor-point generation should be derived reconstruction, not raw LiDAR evidence.
- Reframed fixed modes as baselines or behavior primitives inside a future continuous mixture-of-behaviors controller.
- Documented the parameter dependency rule: nested dependencies are acceptable only when explicit, directional, logged, and acyclic within a single timestep.
- Clarified that optimization should tune parameter profiles offline while runtime behavior remains locally adaptive to terrain, AOIs, communication health, and safety constraints.
- Captured the optimizer direction: manual comparisons first, random or Latin-hypercube sampling for sensitivity baselines, Bayesian optimization once the parameter set is reduced, PSO when larger simulation budgets allow, and RL only after metrics and datasets mature.

### Behavior Profile Registry

- Added `src/swarm/behavior-profile.js` as the first central parameter registry for swarm behavior constants.
- Moved role-balance shares, behavior-mixture weights, task scoring weights, movement smoothing, role speed scales, network envelope gains, AOI focus multipliers, relay waypoint settings, formation thresholds, and objective profiles into the registry.
- Wired the swarm controller, task allocator, formation graph, and mission-loop helper functions to read from the behavior profile instead of isolated literals.
- Added normalized behavior weights and adaptive neighbor target metrics to the swarm controller snapshot so future telemetry can record the controller state alongside run outcomes.
- Added an explicit controller-state pipeline that records normalized signals, behavior weights, derived controls, and the same-timestep dependency graph.
- Connected derived controls to formation spread, role speed, assignment blending, AOI focus scale, and network compliance so the continuous behavior mixture now affects runtime actions.
- Added a live Algorithm panel for inspecting behavior weights, normalized signals, derived controls, role counts, and the active objective profile.
- Added objective-weighted run scoring for AOI quality, coverage, network resilience, time efficiency, compute efficiency, and energy/path proxy. Invalid runs stay saved but have their competitive total score suppressed.
- Added objective-profile selection plus telemetry sorting/filtering so saved runs can be compared by total score, confidence, AOI scenario, validity, or individual subscores under the current objective.
- Expanded score records with raw measures, normalization targets, performance aggregates, and additional objectives for coverage exploration, compute efficiency, and energy/path efficiency.
- Stabilized the point-cloud overlay readout by showing a compact total point count in a fixed-width header area, with visible/total detail preserved in the tooltip.

### Persistent Swarm Telemetry

- Added `src/swarm/run-telemetry.js` with IndexedDB-backed run records for swarm simulation telemetry.
- Swarm missions now start a telemetry run, sample behavior weights/network/AOI/path metrics over time, accumulate raw ray/hit/focused/AOI scan totals, and save the run on reset or completion.
- Added validity flags for insufficient scan time, target not reached, insufficient AOI dwell, low raw AOI coverage, and unstable performance.
- Added a `Telemetry` sidebar panel that lists saved runs, shows basic aggregate counts, and exports run records as JSON.
- Kept the live point cloud raw-only in `sim.voxelMap` after testing the runtime sensor-footprint splat prototype.
- Deferred 9-neighbor/footprint splatting to a future post-scan reconstruction step so synthetic density does not run inside the real-time sensor loop.
- PLY export remains raw-only for measurement integrity.
- Corrected telemetry lifecycle so pause/resume keeps one open run; reset or completion saves it.
- Split live-tunable controls from experiment-defining controls so formation, communication, scan-density, and performance changes no longer reinitialize the swarm over an existing point cloud. Experiment-defining terrain/AOI/swarm-size changes save the previous run and clear the map first.
- Added a Stop button that finalizes the current run, saves telemetry, and leaves the point cloud available for inspection/export without resetting.
- Removed runtime derived-splat maps, active-derived counters, raw-covered cache bookkeeping, and AOI scan throttles from the splat experiment.
- Restored the previous valley aperture object-field prop placement after the circular layout made the AOI read worse visually.
- Set the default Light swarm scan preset to four horizontal rays and four vertical rays per drone for easier inspection and lower default point-cloud churn.
- Restored the softer AOI local-Z color variation after the sharper red ramp proved too harsh in the raw point cloud.

## 2026-04-27

### Branch and baseline

- Created `swarm-v1` as a separate worktree so the posted single-drone project on `main` remains stable.
- Added `PLAN.md` to define the non-weaponized spatial AI swarm roadmap before larger implementation work.

### Swarm architecture scaffolding

- Added swarm modules for drone agents, communication graph, adaptive formation graph, task allocation, terrain classification, map fusion, and swarm control.
- Rationale: keep swarm logic out of the single-drone mission loop until each layer is testable.

### Visible swarm preview

- Added `Mission mode` with `Single drone baseline` and `Swarm V1 preview`.
- Added controls for drone count, formation mode, reliable link distance, K-nearest links, and dropout.
- Added role-colored drones and visible communication links.
- Issue addressed: swarm architecture needed to become inspectable in the scene before deeper autonomy work.

### Communication and formation corrections

- Changed communication from all-to-all within range to K-nearest links within a maximum reliable distance.
- Decoupled communication graph from formation intent: links show who can talk, not where drones must fly.
- Added square-grid spawn, 3D formation altitude offsets, and simple collision envelopes.
- Issue addressed: early preview made formations look flat and communication links looked like formation constraints.

### Adaptive algorithm direction

- The intended system is not a menu of fixed formations. It is an adaptive graph controller.
- Sensor-derived heuristics should estimate openness, aperture width/height, obstacle density, corridor/valley narrowness, vertical clearance, AOI priority, and communication health.
- Formation behavior should adapt from those signals:
  - open terrain -> 3D scatter or line sweep
  - narrow valley/corridor -> compressed wedge
  - hoop/tunnel aperture -> single-file chain
  - weak communication -> relay-preserving topology
  - AOI/object capture -> perimeter/ring from multiple heights
- Smooth motion should be generated below the planner using curves, velocity limits, separation, and collision constraints.

### Next environment requirement

- The room is too small to evaluate swarm behavior.
- Swarm testing needs a large terrain sandbox with mountains, valleys, village/city structures, trees, apertures, and AOIs.

### Terrain sandbox implementation

- Added a mountain valley sandbox as the first large-scale test environment.
- Included a small village, tree field, complex building, and hoop-like valley aperture as object/AOI targets.
- Rationale: adaptive swarm behavior cannot be judged inside the compact room because scale, altitude layers, terrain openness, valley narrowness, and AOI focus never become meaningfully different.
- Fixed terrain visibility by using a separate outdoor atmosphere profile with a much longer fog range.
- Issue addressed: the terrain was inheriting the close indoor fog, so distant mountains and valley features only became visible after zooming in.
- Rescaled the sandbox from a compact valley preview into a larger mountain-valley environment with taller ridges, wider valley travel, larger village lanes, and smaller terrain-mode drones.
- Added terrain-following clearance checks so swarm drones cannot fly through the terrain mesh.
- Issue addressed: earlier swarm targets and clamps used room-like floor assumptions, which allowed terrain clipping.
- Added a ground-level square launch lattice and takeoff phase before adaptive formation behavior begins.
- Issue addressed: drones previously scattered immediately, making it hard to inspect initial conditions, launch location, and odometry.
- Added lightweight per-drone ray scans that write point-cloud samples and per-agent observations into the swarm map-fusion scaffolding.
- Issue addressed: the preview was mostly formation animation; it now has first-pass sensor-derived observations and heuristic task biasing, while full shared voxel fusion remains a later step.
- Improved terrain-mode performance by separating preview-scale terrain mapping from room-scale voxel density.
- Batched swarm point-cloud geometry updates and throttled communication graph redraws to reduce frame-time spikes.
- Rescaled the point-cloud reference grid/camera for terrain mode so it no longer looks like the old room viewport.
- Reworked terrain height generation toward interior ridges, peaks, and edge falloff to avoid the hard boundary/clipped-cloth appearance.

### Network and sensing contract

- Removed single-drone mode selection from `swarm-v1`; the stable single-drone simulator remains on `main`.
- Clarified that visible links are usable communication edges, not formation geometry.
- Changed communication graph construction to capped undirected K-neighbor links within reliable range.
- Added a network envelope to pull drones back toward their required nearest reliable neighbors before rendering links.
- Increased swarm LiDAR density using a configurable scan fan derived from horizontal rays, vertical rays, and FOV controls.
- Replaced numeric per-axis ray controls with a six-level swarm scan density dropdown.
- Restored all-drone scanning on every scan pass, while keeping per-drone fan size bounded by the selected density level.
- Re-exposed per-drone horizontal/vertical ray controls for manual tuning and lowered preset counts so non-economy modes remain more practical.
- Optimized the dense scan path by bounding fan size per density level and flushing point-cloud geometry on a timer.
- Removed analytic terrain intersections from LiDAR sensing because they gave the swarm privileged knowledge of the generated terrain.
- Kept terrain height checks only for simulator-side ground clearance and crash prevention.
- Added `three-mesh-bvh` so LiDAR rays against terrain and objects use spatial acceleration instead of default brute-force mesh raycasting.

### Terrain control cleanup

- Replaced hidden room-generation controls with terrain-generation controls for width, depth, flight ceiling, mountain height, valley width, village structures, tree count, and preview voxel resolution.
- Removed old room control listeners from the active environment regeneration path.
- Reframed legacy single-drone bootstrap controls: vertical seed levels and bootstrap scans are no longer part of the swarm terrain UI.
- Removed temporary sphere/ring drone markers because they looked like unexplained objects instead of UI aids.

### Network correctness pass

- Added terrain flight clearance control so drones can cruise higher above the terrain surface.
- Changed communication links to amber and swarm LiDAR beams to blue so network edges and sensor rays are visually distinct.
- Added terrain line-of-sight filtering so communication edges are not considered usable when the straight link passes through terrain.

### Performance budget pass

- Added visible performance profiles instead of claiming a hard dedicated-GPU percentage cap, because browser WebGL does not expose reliable GPU utilization control.
- Budget profiles adjust render scale, visible point-cloud cap, scan cadence, point-cloud geometry upload cadence, communication redraw cadence, mission graph redraw cadence, and visible beam count.
- Kept the swarm contract intact: every scan pass still scans every active drone. The budget layer spaces expensive passes/uploads and samples the rendered point cloud, while the stored/exported point cloud remains full fidelity.
- Added adaptive frame timing so the sim can gently lower render scale and slow visual uploads when average frame time rises.

### Object-capture readability pass

- Changed point-cloud coloring from a mostly one-dimensional height/range ramp to a multi-axis gradient using height, range, confidence, lateral/depth position, and small spatial variation.
- Rationale: thin objects such as trees, poles, vehicles, and buildings need more chromatic separation from terrain to be inspectable in the point-cloud overlay.
- Replaced the relay-tower AOI with a larger complex building target.
- Added inert static vehicle/artifact silhouettes near the valley aperture as non-operational LiDAR dataset objects only; they do not add targeting, strike, jamming, or weapon behavior.
- Expanded the valley aperture object field with additional trucks, tracked vehicle silhouettes, and gun-shaped static artifacts for denser object-capture testing.
- Added terrain-aware placement search and embedded terrain foundations so the building and object-field props adapt to generated terrain instead of floating or intersecting steep mountain surfaces.
- Added viewport keyboard navigation: the hovered sim or point-cloud window accepts `W/A/S/D`, `Q/E`, and `Shift` for faster inspection movement while mouse orbit/zoom still works.
- Increased practical scan density slightly and added AOI-focused scan cones that activate near selected AOIs, giving structures and object fields more point samples without globally overloading terrain scans.
- Added AOI return metadata and a high-contrast AOI point-cloud highlight blend so object-capture regions do not visually disappear into the terrain color ramp.
- Changed viewport keyboard motion from ground-plane panning to camera-relative fly movement, so `W/S` follows the mouse-set viewing angle including altitude.
- Changed AOI point-cloud highlighting from a mostly uniform tint to an object-scale local gradient based on each point's position around the AOI.
- Sharpened the AOI color gradient with higher local frequency, stronger saturation, and stronger AOI blending so small object-scale geometry changes produce clearer color changes.
- Rebalanced the AOI gradient after testing: softer than the sharp pass, but still object-scale instead of map-scale.
- Changed selected-AOI swarm behavior from stopping at the AOI center to orbiting around the AOI, improving point capture from multiple positions.
- Reduced the valley object-field prop count while scaling the props up so trucks, tracked silhouettes, and static artifacts are easier to inspect.
- Added click-to-focus orbit behavior for both the simulation and point-cloud views, so the mouse rotates around the selected local point rather than the whole map center.
- Updated Swarm V1 defaults to 24 drones, 6 K-nearest comms links, Light scan density, and a higher Quality performance budget.
- Replaced click-to-focus retargeting with free-look left-drag rotation to remove camera jumps while preserving WASD movement, wheel zoom, and right-drag pan.
- Softened the global and AOI point-cloud palettes with smoother hue, saturation, and lightness changes for a less harsh visual read.
- Added velocity smoothing to keyboard camera translation so WASD/QE accelerates and eases out instead of stepping abruptly.
- Retuned global and AOI point-cloud gradients to make X/Z ground-plane depth and vertical Y height more legible while keeping the palette eye-friendly.
- Increased the visible point cap path from `300,000` to `1,200,000` so custom high point budgets can actually render more of the cloud when the machine can handle it.
- Reduced AOI-specific coloring to a light local Z-depth cue only, avoiding circular color artifacts caused by orbiting AOI capture paths.
- Inverted mouse vertical look direction for the free-look camera controls.
- Replaced selected-AOI circular orbiting with a directional Z-depth inspection sweep so drones keep moving and collecting without painting circular artifacts.
- Increased focused AOI scan density only at close range, preserving normal terrain scan cost while improving near-object capture.
- Retuned AOI coloring to a sharper local Z-only gradient scaled by AOI/object size.
- Changed the AOI Z-depth gradient to a red-only scale for a clearer object-capture visual cue.
- Excluded visual AOI marker spheres from LiDAR raycast targets so focused scanning does not over-sample the marker itself.
- Changed focused AOI rays to aim at distributed local Z-depth sample lanes across the AOI/object field rather than the AOI center point.
- Expanded AOI focus sampling to more Z-depth lanes with wider X/Y offsets so multiple objects inside an AOI field receive focused rays.
- Intensified the red AOI Z-depth gradient while keeping the cue strictly local-Z based.

### Adaptive algorithm design stance

- We are not using every algorithm as a separate standalone mode.
- The intended system is a custom layered adaptive controller:
  - LiDAR/radar-inspired sensing estimates local geometry and constraints.
  - K-nearest communication shares compressed neighbor state through the swarm.
  - Terrain heuristics classify openness, corridor/valley narrowness, aperture size, obstacle density, vertical clearance, and AOI value.
  - Task allocation assigns scout, mapper, relay, and verifier roles to frontier/AOI/network tasks.
  - Formation topology changes in 3D from those signals.
  - Smooth motion converts graph targets into curves and constrained trajectories.
- Voxel planning remains useful for state-space reasoning, but the visible drone motion should be smooth, adaptive, and constraint-aware.

### Performance and continuity milestone

- Added `ARCHITECTURE.md` to document the current module layout, runtime data flow, bottlenecks, future extraction targets, and non-regression rules before the custom swarm behavior kernel begins.
- Added `SESSION_HANDOFF.md` so future sessions can recover branch state, run commands, current defaults, warnings, and the next milestone order without relying only on chat context.
- Added top-bar performance diagnostics for FPS/frame time, scan pass time, rays per pass, hit count, point-cloud flush time, visible point count, and active budget context.
- Reworked visible point-cloud flushing to reuse typed-array buffers and `BufferGeometry` draw ranges instead of rebuilding fresh arrays and attributes on every visual upload.
- Cached swarm scan direction fans by ray count and FOV so Dense/Survey passes spend less time recreating identical direction vectors.
- Preserved the swarm sensing contract: every drone still scans on each scan pass; the optimization targets buffer/upload/allocation cost rather than skipping agents.
