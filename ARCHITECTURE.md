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
    constants.js       Roles, formation modes, terrain classes, AOI types
    drone-agent.js     Per-drone role, route, mailbox, observations, and metrics state
    communication-graph.js
                       K-nearest, range-limited drone-to-drone link model
    formation-graph.js 3D formation target generation and adaptive topology hints
    terrain-classifier.js
                       Lightweight terrain/problem classification scaffold
    task-allocator.js  Utility scoring for frontier, AOI, relay, and verification assignments
    map-fusion.js      Point-cloud tile and mission replay export scaffolding
    swarm-controller.js
                       Swarm facade that owns agents, comms, formation, allocation, and fusion
```

`main.js` is still intentionally large because the prototype is moving quickly. Extraction targets are listed below so future work can make the project easier to reason about without changing behavior all at once.

## Runtime Data Flow

1. Terrain generation builds a mountain-valley scene, village/tree/vehicle/object props, AOI markers, raycast target meshes, and a coarse preview voxel volume.
2. Mission start initializes the swarm, assigns ground-lattice launch positions, starts the takeoff phase, and resets the point cloud/map state.
3. The animation loop updates camera controls, swarm motion, communication links, formation targets, scan timing, cloud flush timing, and readouts.
4. Each scan pass asks every active drone to cast its configured LiDAR fan against the scene meshes. AOI focus rays are added only when a drone is close enough to an AOI.
5. Ray hits are voxel-deduplicated into `sim.voxelMap`; each voxel stores averaged position, hit count, distance sum, and AOI/focus metadata.
6. Point-cloud rendering samples the stored voxel map up to the visible point cap and uploads positions/colors into reusable typed-array buffers.
7. Export uses the stored point cloud, not just the currently visible sampled subset.

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
- Live diagnostics report FPS/frame time, scan pass time, rays/hits per pass, cloud flush time, visible points, and active budget.

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
