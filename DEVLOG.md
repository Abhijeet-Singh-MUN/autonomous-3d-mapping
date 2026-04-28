# Swarm V1 Development Log

This log captures why changes are made, which issues they address, and what they unlock for the later README and research narrative.

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
- Included a small village, tree field, relay tower, and hoop-like valley aperture as object/AOI targets.
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
- Removed temporary sphere/ring drone markers because they looked like unexplained objects instead of UI aids.

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
