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
```

The simulator now treats the map as a voxel-based state graph. Each grid vertex is a possible drone state point with a known, free, or occupied state. LiDAR ray samples update that graph, the planner scores frontier and inspection goals from the same graph, and collision avoidance uses the live map plus local ray probes.

Coordinate convention:

- `col`: room X axis
- `row`: room Z/depth axis
- `layer`: room Y/height axis

This matters because the planner, coverage map, height bands, and collision clearance must all agree on what "up" means.

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

## Main controls

- `Planner`: chooses the grid search strategy
- `Steering style`: controls how the local path is shaped after planning
- `Bootstrap scans`: how much the drone scans before choosing its first goal
- `Safety radius`: short-range collision buffer
- `Goal reach radius`: how close the drone must get before it holds and rescans
- `Grid spacing`: resolution of the discovered occupancy map

## Export

Use `Export PLY` after a mission to save the current colored point cloud for later reconstruction work.

## Development Log

### 2026-04-25

- Standardized the project/package name on `autonomous-3d-mapping`.
- Added repository hygiene with `.gitignore`.
- Split core simulator logic into reusable modules for constants, math helpers, voxel-grid state, and 3D planning.
- Corrected the voxel coordinate model so vertical planning uses `layer`/Y instead of mixing Y and Z axes.
- Documented the voxel state-graph direction for autonomous traversal, exploration, LiDAR object recording, and obstacle avoidance.

## GitHub Publishing

This folder is ready to become a public GitHub repository. The current machine does not have GitHub CLI installed, so create an empty public repo on GitHub or install/authenticate `gh`, then add the remote and push:

```bash
git remote add origin https://github.com/<your-user>/<repo-name>.git
git branch -M main
git push -u origin main
```
