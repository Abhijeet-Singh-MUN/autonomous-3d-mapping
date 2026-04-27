# Autonomous Room LiDAR Sim

Working repo name: `autonomous-3d-mapping`

This is a standalone Three.js + Vite simulator for an autonomous drone that:

- bootstraps a map from LiDAR data
- builds a discovered 3D voxel graph online
- chooses frontier and inspection goals from sensed space
- replans with `A*`, weighted `A*`, or greedy best-first search
- avoids collisions with short-range forward safety checks
- exports colored point clouds as `PLY`

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
    constants.js       Swarm roles, formation modes, terrain classes, AOI types
    drone-agent.js     Per-drone state container for roles, route, messages, observations
    communication-graph.js
                       Range-limited drone-to-drone graph and message delivery model
    formation-graph.js Adaptive graph formation target generator
    terrain-classifier.js
                       Terrain/problem classifier for adaptive behavior
    task-allocator.js  Utility-based drone-to-task assignment
    map-fusion.js      Point-cloud tile and replay-log aggregation scaffolding
    swarm-controller.js
                       Facade for agents, communication, formation, allocation, fusion
```

The simulator now treats the map as a voxel-based state graph. Each grid vertex is a possible drone state point with a known, free, or occupied state. LiDAR ray samples update that graph, the planner scores frontier and inspection goals from the same graph, and collision avoidance uses the live map plus local ray probes.

Coordinate convention:

- `col`: room X axis
- `row`: room Z/depth axis
- `layer`: room Y/height axis

This matters because the planner, coverage map, height bands, and collision clearance must all agree on what "up" means.

## Swarm V1 Branch

The `swarm-v1` branch is the active development branch for adaptive multi-drone exploration. The posted single-drone room simulator remains the baseline behavior while the swarm layer is built in modules.

Swarm V1 direction:

- Preserve the current single-drone planner and room workflow.
- Add configurable swarms of `3..24` drones.
- Use adaptive graph formations rather than fixed-only formations.
- Simulate range-limited drone-to-drone communication with latency/dropout hooks.
- Classify terrain/problem context to choose sparse search, relay, wedge, perimeter, or scatter behavior.
- Fuse per-drone map and point-cloud observations into dataset-oriented outputs.

Initial swarm modules are scaffolding only; they are not yet wired into the visible simulation UI.

## Run locally in a normal browser

```bash
npm install
npm run dev
```

Then open the local URL shown by Vite, usually:

```text
http://127.0.0.1:5173/
```

## Production preview

```bash
npm run build
npm run preview
```

## Mission Controls

### Viewports

- The main room simulation and point-cloud overlay are both resizable from the lower-right grip.
- The point-cloud overlay can also be dragged by its header.
- Both 3D windows use `OrbitControls`, so orbit, zoom, and pan are handled by the Three.js control layer.

### Room generation

- `Room preset`: changes the furniture mix, such as mixed room, living room, or bedroom.
- `Room width`, `Room depth`, `Room height`: resize the simulated room.
- `Furniture count`: controls how many generated objects are placed.
- `Ceiling obstacle`: toggles generated ceiling hazards that affect clearance and flight planning.
- `Randomize Room`: changes the room seed and rebuilds the layout.

### LiDAR and point-cloud capture

- `Sensor preset`: loads defaults for spinning LiDAR, forward LiDAR, depth camera, or custom scanning.
- `Horizontal rays`, `Vertical rays`: control the scan ray grid density. Higher values capture more detail but cost more CPU.
- `Sensor H FOV`, `Sensor V FOV`: control the horizontal and vertical scan cone in degrees.
- `Sensor range`: caps raycast distance and therefore how far free/occupied voxels can be discovered.
- `Voxel size`: controls point-cloud downsampling size, not the planner grid.

Current limits:

- Horizontal rays are clamped to `16..220`.
- Vertical rays are clamped to `8..96`.
- Horizontal FOV is clamped to `45..360` degrees.
- Vertical FOV is clamped to `12..130` degrees.
- Sensor range is clamped to `2..20` world units.
- Presets choose the scan mode and pitch bias. Numeric LiDAR values can be edited after choosing any preset.
- `Custom` currently uses forward scan mode with no pitch bias.

### Mission timing and exploration

- `Scan mode`: stepped scans less often; continuous scans more often while moving and dwelling.
- `Vertical seed levels`: number of starting height bands used during bootstrap.
- `Bootstrap scans`: number of yaw samples per seed level before frontier exploration begins.
- `Dwell / scan interval`: base timing for bootstrap, hold scans, and move scans.
- `Move speed`: drone travel speed along the sampled route.
- `Goal reach radius`: distance at which a goal can be counted as reached.

### Mapping and safety

- `Voxel resolution`: planner-grid spacing for the occupancy graph.
- `Safety radius`: clearance buffer used when deriving traversable voxels and during live collision checks.
- The mapper stores each voxel state as unknown, free, or occupied. LiDAR rays mark free space along the ray and occupied space at hits.

### Planner names and scoring

The planner dropdown controls the 3D voxel graph search in `src/core/planner.js`:

- `A*`: `heuristicWeight = 1`, `gFactor = 1`
- `Weighted A*`: `heuristicWeight = 1.8`, `gFactor = 1`
- `Greedy best-first`: `heuristicWeight = 2.6`, `gFactor = 0.2`

`heuristicWeight` increases how strongly the planner favors cells closer to the goal. `gFactor` controls how much already-traveled path cost matters. A lower `gFactor` makes the search greedier because it cares less about the accumulated route cost.

Goal selection is separate from path search. Candidate goals are scored by information gain, clearance, travel cost, revisit penalty, and altitude-band penalty. The selected goal is then routed with the chosen planner mode.

### Steering style

Planner output is a voxel path. `Steering style` changes how that path becomes motion samples:

- `Manhattan`: follows the simplified voxel waypoint path directly.
- `Hybrid`: smooths the simplified path with a moderate Catmull-Rom curve.
- `Curved`: uses the same curve with denser sampling for smoother motion.

Catmull-Rom is an interpolating spline: the curve passes through the route waypoints while rounding sharp corners into smoother flight motion.

## Export

Use `Export PLY` after a mission to save the current colored point cloud for later reconstruction work.

## Development Log

### 2026-04-27

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
