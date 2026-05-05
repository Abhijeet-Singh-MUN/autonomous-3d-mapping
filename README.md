# Autonomous Terrain Swarm LiDAR Sim

Working repo name: `autonomous-3d-mapping`

This is a standalone Three.js + Vite simulator for an autonomous terrain-mapping drone swarm that:

- launches multiple coordinated drones into a synthetic mountain-valley environment
- scans terrain and objects with scene raycast LiDAR
- records colored point clouds as `PLY`
- maintains range-limited drone-to-drone communication links
- adapts swarm formations around terrain, AOIs, and network constraints

## Current Architecture

```text
src/
  main.js              UI wiring, Three.js scene setup, mission loop
  styles.css           App layout and visual styling
  core/
    constants.js       Occupancy states and sensor presets
    math.js            Shared numeric helpers and seeded RNG
    planner.js         3D graph search over voxel state points
    voxel-grid.js      Voxel graph storage and world/grid transforms
  swarm/
    behavior-profile.js
                       Parameter registry, behavior-weight math, objective profiles
    constants.js       Swarm roles, formation modes, terrain classes, AOI types
    drone-agent.js     Per-drone state container for roles, route, messages, observations
    communication-graph.js
                       Range-limited drone-to-drone graph and message delivery model
    formation-graph.js Adaptive graph formation target generator
    terrain-classifier.js
                       Terrain/problem classifier for adaptive behavior
    task-allocator.js  Utility-based drone-to-task assignment
    map-fusion.js      Point-cloud tile and replay-log aggregation scaffolding
    resource-model.js  First-pass drone energy/compute/sensor accounting model
    run-telemetry.js   IndexedDB run records for behavior/scan/network metrics
    swarm-controller.js
                       Facade for agents, communication, formation, allocation, fusion
```

The simulator now treats the map as a voxel-based state graph. Each grid vertex is a possible drone state point with a known, free, or occupied state. LiDAR ray samples update that graph, the planner scores frontier and inspection goals from the same graph, and collision avoidance uses the live map plus local ray probes.

Coordinate convention:

- `col`: terrain X axis
- `row`: terrain Z/depth axis
- `layer`: terrain Y/height axis

This matters because the mapper, coverage view, and collision clearance must all agree on what "up" means.

## Swarm V1 Branch

The `swarm-v1` branch is the active development branch for adaptive multi-drone terrain exploration. The posted single-drone room simulator remains preserved on `main`; this branch is now terrain/swarm focused.

Swarm V1 direction:

- Preserve the posted single-drone workflow on `main`.
- Add configurable swarms of `3..24` drones.
- Use adaptive graph formations rather than fixed-only formations.
- Simulate range-limited drone-to-drone communication with latency/dropout hooks.
- Classify terrain/problem context to choose sparse search, relay, wedge, perimeter, or scatter behavior.
- Fuse per-drone map and point-cloud observations into dataset-oriented outputs.

Current Swarm V1 preview:

- `Mission mode` is now fixed to `Swarm V1 preview` on this branch. The single-drone baseline remains preserved on `main`.
- `ARCHITECTURE.md` documents the current module layout, runtime data flow, bottlenecks, extraction targets, and non-regression rules.
- `SESSION_HANDOFF.md` is the starting point for future chats/sessions, with branch state, run commands, current defaults, and next milestone order.
- Terrain generation controls shape the mountain-valley sandbox directly.
- `AOI preset` selects autonomous AOIs or a specific village, complex building, or aperture focus target before mission start.
- `Swarm drones` configures the visible drone count from `3..24`.
- `Swarm formation` exposes adaptive graph, scatter, line sweep, wedge, relay chain, and perimeter ring modes.
- `Max reliable link distance`, `K-nearest comms links`, and `Link dropout probability` drive the range-limited communication graph.
- Communication links represent currently usable drone-to-drone network edges after reliable range, K-neighbor degree limits, and dropout are applied. They are not formation lines.
- Swarm drones are role-colored and visible in the main scene.
- Communication links are drawn between connected drones.
- Initial swarm spawn uses a ground-level square launch lattice, then a visible takeoff phase before the selected formation takes over.
- Formations now use 3D offsets instead of a single flat plane.
- Drone-drone, drone-object, and terrain-clearance envelopes keep preview drones separated from each other, known objects, and the mountain surface.
- Swarm drones run budgeted local ray scans, add hits to the point cloud, and record per-agent observations for map-fusion scaffolding.
- Swarm motion now applies a communication envelope so drones are pulled back toward their nearest reliable neighbors before links are rendered.
- Swarm Behavior Kernel V1 rebalances scout, mapper, relay, and verifier roles from frontier pressure, AOI presence, terrain class, and communication health.
- Role-aware task allocation now includes frontier, AOI, relay, and verification tasks, with relay waypoints laid between launch/base and the active mission center.
- Formation targets and drone movement are smoothed with per-agent target memory and velocity steering before terrain, object, drone, and network constraints are applied.
- `Telemetry` opens a local run-history panel backed by IndexedDB; swarm runs are saved on Stop, reset, or completion with behavior, scan, AOI, network, and validity metrics. Pause/Continue keeps the same run open.
- `Algorithm` opens a live controller panel showing behavior weights, normalized signals, derived controls, role counts, and the active objective profile.
- `Objective profile` changes how runs are scored for comparison: balanced mapping, AOI quality, network resilience, coverage exploration, compute efficiency, or energy/path efficiency.
- This slice previews swarm topology, launch behavior, heuristic sensing, and mission state; full shared voxel fusion and robust planner integration are next-stage work.
- The terrain sandbox is scaled up with taller mountains, a wider valley corridor, larger village spacing, trees, a complex building AOI, and a hoop-like aperture object field to make 3D swarm behavior testable at a more realistic scale.
- Terrain mode uses a much wider outdoor camera fog range so distant mountains and valley features remain visible before zooming in.
- Terrain mode uses a coarse preview voxel grid, lower-density terrain mesh, BVH-accelerated scene raycasts, selectable swarm scan density, throttled communication redraws, and batched point-cloud updates to keep the swarm preview responsive.
- The point-cloud viewport now scales its reference grid/camera to the active terrain environment instead of staying room-sized.

Adaptive algorithm direction:

- V1 is moving toward a custom layered controller: sensor heuristics, distributed communication, task allocation, formation topology, and smooth constrained motion.
- LiDAR/radar-style heuristics should estimate openness, aperture size, vertical clearance, obstacle density, AOI value, and communication health.
- The swarm should act like one distributed organism: each drone senses locally, shares compact state with nearest neighbors, and adapts the group topology from partial knowledge.
- Motion should remain smooth even when voxel planning is used underneath. Curves, velocity limits, separation, and collision constraints should translate graph decisions into flight paths.
- The first behavior-kernel slice keeps all drones sensing on every scan pass, while mappers/verifiers receive stronger AOI focus rays and relay drones favor network continuity.

Swarm algorithm direction:

- The swarm is moving toward a continuous mixture-of-behaviors controller rather than a set of hard fixed modes.
- The current model family is `greybox-policy-v1` / `greybox-policy-v1.0`.
- Future optimizers now see four interpretable policy coordinates: `coverage_area`, `aoi_detail`, `risk_safety`, and `resource_efficiency`.
- The four policy coordinates are exposed as mission-control sliders and normalized before deriving the active behavior profile.
- Low-level controller parameters still matter, but they are derived from those policy coordinates through documented coupling coefficients instead of being optimized independently.
- Behavior weights will be computed from normalized terrain, AOI, communication, coverage, and mission-progress signals.
- The current controller now logs normalized signals and derives bounded controls for formation spread, movement speed, assignment pull, AOI focus, and network compliance from those behavior weights.
- Soft constraints now feed the controller as normalized signals: AOI proximity risk, mission-time pressure, battery reserve pressure, and compute pressure. They can raise avoidance/efficiency behavior without replacing the AOI, coverage, and relay objectives.
- Saved runs are scored with the active objective profile across AOI quality, coverage, network resilience, time efficiency, compute efficiency, energy/path proxy, adaptation smoothness, and constraint safety. Scoring also stores raw measures, temporal measures, normalization targets, and confidence metadata.
- Telemetry now stores first-pass resource fields for hover, motion, sensor, compute, communication, total energy, and estimated battery remaining. The current values are placeholder simulation estimates until calibrated from sourced platform and sensor data.
- Telemetry samples now include temporal rate metrics such as new voxels per second, AOI hits per second after contact, energy/compute rate, network fragmentation duration, and behavior-weight change rate. Run scores can therefore use curve-derived behavior, not only final totals.
- LiDAR area coverage is estimated separately from raw point storage. Each hit contributes a distance-scaled footprint radius for coverage/resolution/redundancy metrics, but it does not create synthetic point-cloud samples.
- Inter-drone footprint redundancy now feeds back into formation spacing: overlapping LiDAR footprints increase area-spread pressure, while low footprint resolution can tighten spacing for detail.
- Telemetry can sort saved runs by score, confidence, or subscore, filter to valid runs only, and filter by AOI scenario.
- Telemetry defaults to the active model family, so old runs remain stored locally but do not mix into grey-box comparison/export by default.
- `SWARM_ALGORITHM.md` is the detailed design rationale for the math, parameter dependencies, optimization direction, and reasoning history behind this approach.

Formation notes:

- `Line sweep` is a broad scanning front for coverage. It spreads drones across the search width and staggers altitude layers.
- `Relay chain` is a communication-preserving chain from base toward a remote region. It prioritizes network continuity and altitude stepping over area width.
- `Scatter` uses a spread-out graph pattern for sparse terrain. Communication is K-nearest within the max reliable link distance rather than all-to-all.
- `Link dropout probability` defaults to `0` for stable network testing. Raising it intentionally simulates unreliable links even when drones are inside reliable range.

## Run locally in a normal browser

```bash
npm install
npm run dev
```

Then open the local URL shown by Vite, usually:

```text
http://127.0.0.1:5173/
```

On Windows PowerShell, use `npm.cmd run dev` if the shell tries to open `npm.ps1` with another app.

## Production preview

```bash
npm run build
npm run preview
```

## Mission Controls

### Viewports

- The main terrain simulation and point-cloud overlay are both resizable from the lower-right grip.
- The point-cloud overlay can also be dragged by its header.
- Both 3D windows use mouse navigation: left-drag rotates the camera view in place, wheel zooms, and right-drag pans without retargeting the camera.
- Hover either 3D window and use smoothed camera-relative fly controls: mouse sets viewing direction, `W/S` moves forward/back along that direction, `A/D` strafes, `Q/E` moves down/up, and `Shift` moves faster.

### Swarm and terrain

- `AOI preset`: chooses autonomous AOIs or a specific village, complex building, or aperture focus target.
- `Terrain width`, `Terrain depth`: change the horizontal scale of the synthetic environment.
- `Flight ceiling`: controls the maximum drone altitude and mapper volume.
- `Mountain height`: scales ridge and peak height.
- `Valley width`: changes how open or narrow the central valley corridor is.
- `Village structures`: changes the number of generated village buildings.
- `Tree count`: changes generated vegetation density.
- `Preview voxel resolution`: controls the coarse swarm preview occupancy grid.
- `Swarm drones`: controls active drone count from `3..24`; the Swarm V1 default is `24`.
- `Swarm formation`: selects adaptive graph, scatter, line sweep, wedge, relay chain, or perimeter ring behavior.
- `Max reliable link distance`: maximum distance for a usable drone-to-drone communication edge.
- `K-nearest comms links`: target reliable neighbor count for each drone; the default is `6`.
- `Link dropout probability`: defaults to `0`; raising it intentionally stress-tests unreliable comms.
- `Performance budget`: browser-side workload profile. It controls render scale, visible point-cloud cap, scan cadence, point-cloud upload cadence, comm-link redraw cadence, and mission graph redraw cadence.
- Top-bar diagnostics show FPS/frame time, scan pass cost, rays/hits per pass, and point-cloud flush time so Dense/Survey tuning is based on measured bottlenecks.
- `Visible point cap`: maximum point-cloud samples drawn in the viewport at once, up to `1,200,000` in custom mode. Export still uses the full stored point cloud.
- `Render scale cap`: maximum WebGL pixel ratio used by the simulation and point-cloud canvases.
- `Safety radius`: minimum collision/clearance envelope used by swarm separation and terrain clearance.
- `Terrain flight clearance`: target altitude above terrain for cruise and formation targets.
- `Move speed`: visible swarm motion speed.
- `Randomize Terrain`: rebuilds the mountain-valley sandbox seed.

### LiDAR and point-cloud capture

- `Swarm scan density`: six-level preset dropdown for per-drone scan fan density: economy, light, balanced, dense, survey, and max capture. The default is `Light`; `Custom` is selected automatically when ray counts are edited directly.
- `Per-drone H rays`, `Per-drone V rays`: explicit horizontal and vertical ray counts per drone per scan pass.
- `Sensor H FOV`, `Sensor V FOV`: control the horizontal and vertical scan cone in degrees.
- `Sensor range`: caps raycast distance and therefore how far free/occupied voxels can be discovered.
- `Voxel size`: controls point-cloud downsampling size.
- `Dwell / scan interval`: timing knob retained for mission update cadence.
- AOI capture adds a focused scan cone only when drones are close to an AOI, so buildings and valley object fields receive denser point samples without globally increasing every terrain ray.
- AOI focus rays are distributed across wider Z-depth lanes and nearby X/Y offsets inside the AOI/object-field area instead of aiming at the visual AOI marker.
- The live point cloud is raw LiDAR voxel evidence only. The 9-neighbor/footprint-splat idea is deferred to a post-scan reconstruction step so synthetic density does not slow the real-time swarm loop or affect scan telemetry. PLY export currently uses raw points only.

Current limits:

- Horizontal FOV is clamped to `45..360` degrees.
- Vertical FOV is clamped to `12..130` degrees.
- Sensor range is clamped to `2..80` world units in terrain mode.

### Mapping and safety

- The current swarm branch uses coarse preview voxel state for responsiveness while the point cloud records denser samples.
- LiDAR discovery raycasts against the rendered scene meshes. The terrain height model is only used by the simulator for ground clearance/crash prevention, not as drone knowledge.
- Point-cloud geometry is flushed in batches: drone observations and global visualization samples are accumulated first, then periodically uploaded to the Three.js `BufferGeometry` used by the visible point cloud.
- The visible point cloud now reuses typed-array buffers and draw ranges instead of rebuilding fresh JavaScript arrays and geometry attributes every flush.
- Per-density/FOV scan direction sets are cached, reducing repeated allocation during LiDAR passes.
- Point-cloud colors use a multi-axis gradient from world X/Z position, vertical Y height, sensor range, and confidence so depth and altitude are easier to read.
- AOI-focused returns use an intensified red local Z-depth highlight scaled by AOI/object size, avoiding circular color artifacts from inspection paths.
- Visual AOI markers are excluded from LiDAR ray targets, so they do not create artificial hot spots in the point cloud.
- Performance budgets are simulator-side safety limits, not a true operating-system GPU cap. Browser WebGL does not expose reliable dedicated-GPU utilization control, so the sim manages workload through render scale, visible point cap, redraw cadence, and adaptive frame timing.
- Every scan pass still includes every active drone. Under heavier budgets, the sim may space scan passes and visual uploads farther apart instead of rotating drones out of sensing.
- Raycasting uses `three-mesh-bvh` acceleration so all drones can scan without falling back to privileged terrain shortcuts.
- Communication links are terrain line-of-sight filtered; links are not considered usable if the straight connection passes through the terrain surface.
- Full planner-backed voxel traversal and map fusion are upcoming milestones.

## Export

Use `Export PLY` after a mission to save the current colored point cloud for later reconstruction work.

## Development Log

### 2026-04-27

- Added `DEVLOG.md` to track issues, design rationale, and feature decisions as Swarm V1 evolves.
- Added a mountain valley sandbox environment with terrain, village structures, trees, a complex building AOI, and an aperture object field.
- Rescaled the terrain sandbox to a larger mountain-valley environment with wider village lanes and smaller terrain-mode drone meshes.
- Added terrain-following clearance so swarm drones stay above the mountain surface instead of clipping through it.
- Added a square-lattice launch phase with takeoff before the swarm begins formation movement.
- Added lightweight per-drone ray scans that record point-cloud samples and feed early map-fusion/heuristic scaffolding.
- Improved terrain-mode performance by using a coarse preview voxel grid, batching swarm point-cloud flushes, and throttling communication-line redraws.
- Removed single-drone runtime selection from the swarm branch to avoid terrain-mode hangs; `main` remains the preserved single-drone baseline.
- Changed communication graph construction to capped undirected K-neighbor links and added a movement envelope that keeps drones within reliable neighbor range.
- Added a six-level swarm scan density dropdown and restored all-drone scanning on every scan pass.
- Added editable per-drone horizontal/vertical ray controls and lowered preset ray counts for practical browser simulation.
- Kept dense scanning responsive by capping per-drone fan size by density level and batching point-cloud flushes.
- Removed the analytic terrain-hit shortcut from LiDAR sensing so terrain is discovered through scene raycasts rather than privileged generator knowledge.
- Added `three-mesh-bvh` to accelerate scene raycasts for dense swarm LiDAR.
- Added terrain flight clearance control and terrain line-of-sight filtering for network links.
- Changed network-link color to amber and LiDAR beam color to blue so comms and sensing read as separate systems.
- Added simulator performance budgets for render scale, visible point-cloud cap, scan cadence, point-cloud geometry uploads, comm redraws, and mission graph redraws.
- Updated point-cloud coloring from a mostly height/range ramp to a multi-axis gradient so object geometry is easier to read.
- Replaced the relay tower AOI with a larger complex building and added inert static vehicle/artifact props near the valley aperture as LiDAR dataset targets.
- Added terrain-aware AOI prop placement with embedded foundations so generated buildings and valley object-field props sit naturally on changed terrain.
- Added keyboard navigation for both the terrain simulation and point-cloud viewports.
- Added AOI-focused scan cones and AOI-highlight point-cloud coloring so the swarm records selected structures/object fields with denser, more readable samples.
- Removed the temporary sphere/ring markers above swarm drones.
- Changed the terrain height model from boundary-rising slopes to interior ridges/peaks with edge falloff so mountains read less like a clipped cloth plane.
- Scaled the point-cloud viewport grid and camera to terrain mode instead of leaving it at room scale.
- Fixed terrain visibility by giving outdoor mode a much longer fog range instead of reusing the close indoor room fog.
- Added AOI preset control and wired AOI targets into swarm preview assignment/formation behavior.
- Documented the layered adaptive algorithm direction for sensor heuristics, distributed communication, topology, and smooth constrained motion.
- Changed swarm communication from all-to-all-within-range to K-nearest links inside a max reliable distance.
- Added square-grid initial swarm spawning before formation movement.
- Added 3D altitude offsets to swarm formations and collision envelopes for drone-drone/object avoidance.
- Clarified Swarm V1 preview controls and formation behavior in documentation.
- Wired the Swarm V1 preview into the UI with mission mode, drone count, formation, and communication controls.
- Added multi-drone rendering with role-colored agents and visible communication graph links.
- Preserved the single-drone simulator as the default baseline mode.
- Added initial `swarm-v1` scaffolding modules for drone agents, communication graphs, adaptive formations, task allocation, terrain classification, map fusion, and swarm control.
- Documented the active swarm branch direction while keeping the current single-drone simulator as the baseline.

### 2026-04-25

- Made LiDAR numeric controls editable for every sensor preset.
- Added visible resize grips and resize support for the room simulation and point-cloud overlay.
- Expanded mission-control documentation for LiDAR limits, planner scoring, steering, and viewport behavior.
- Standardized the project/package name on `autonomous-3d-mapping`.
- Added repository hygiene with `.gitignore`.
- Split core simulator logic into reusable modules for constants, math helpers, voxel-grid state, and 3D planning.
- Corrected the voxel coordinate model so vertical planning uses `layer`/Y instead of mixing Y and Z axes.
- Documented the voxel state-graph direction for autonomous traversal, exploration, LiDAR object recording, and obstacle avoidance.

## GitHub Publishing

The public repository is:

https://github.com/Abhijeet-Singh-MUN/autonomous-3d-mapping
