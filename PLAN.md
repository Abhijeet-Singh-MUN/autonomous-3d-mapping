# Swarm Spatial AI V1 Plan

## Goal

Build a non-weaponized adaptive drone swarm simulator for large-scale 3D spatial AI research. The swarm should explore synthetic 3D environments, coordinate through distributed communication, capture LiDAR point clouds, fuse voxel maps, and generate dataset-ready 3D samples for future reconstruction and generation models.

## Baseline

The existing single-drone room simulator remains the stable baseline. It already supports LiDAR scanning, voxel occupancy mapping, A*/Weighted A*/Greedy pathfinding, collision checks, and point-cloud export.

Swarm V1 builds on that system without rewriting the current path planner.

## V1 Scope

- Configurable swarm size: 3-24 drones.
- Default environment: hybrid city-terrain.
- Default formation mode: adaptive graph formation.
- Manual formation modes for comparison:
  - Scatter
  - Line Sweep
  - Wedge
  - Relay Chain
  - Perimeter Ring
- Drone roles:
  - Scout
  - Mapper
  - Relay
  - Verifier
- Distributed communication:
  - Range-limited neighbor graph
  - Simulated latency/dropout
  - Shared position, role, frontier, AOI, and map-delta messages
- AOI handling:
  - Autonomous AOI detection from terrain/geometry signals
  - User-marked AOIs
- Exports:
  - Point-cloud tiles with metadata
  - Mission replay logs with trajectories, roles, formation states, communication graph snapshots, scan events, and AOI events

## Adaptive Formation Concept

The swarm should not behave as a set of fixed presets. Formations are named graph behaviors inside a dynamic topology.

Adaptive mode chooses and blends formation behavior based on:

- Frontier density
- Terrain openness/sparsity
- City density and occlusion
- Tall structures or vertical features
- Communication graph health
- Drone spacing and collision risk
- AOI priority
- Coverage quality

Expected behavior:

- Sparse open areas favor Scatter or Line Sweep.
- Dense city corridors favor Wedge.
- Weak connectivity favors Relay Chain.
- Important structures or AOIs favor Perimeter Ring.
- Many unknown frontiers favor adaptive Scatter.

## Architecture

### SwarmController

Owns drone agents, mission state, swarm metrics, adaptive mode, task allocation, and global update loop.

### DroneAgent

Represents each drone's mesh, role, route, local sensor state, mailbox, local map confidence, point-cloud observations, and current assignment.

### CommunicationGraph

Builds range-limited neighbor links, simulates latency/dropout, tracks connected components, and routes local messages.

### FormationGraph

Produces dynamic topology targets from selected formation mode, adaptive signals, AOIs, and communication constraints.

### TerrainClassifier

Labels regions as sparse/open, dense urban, corridor, vertical/tall-structure, occluded, or AOI-rich.

### TaskAllocator

Assigns drones to frontiers, relay positions, AOIs, and verification tasks using utility scoring.

### MapFusion

Merges LiDAR observations into shared voxel state and point-cloud tiles while preserving metadata.

## Metrics

Track and display:

- Coverage percentage over time
- Known voxel count
- Point-cloud sample count
- AOI count and capture quality
- Communication graph connectivity
- Drone role distribution
- Collision/replan count
- Swarm efficiency compared with single-drone baseline

## Non-Goals For V1

- No military strike, jamming, or targeting behavior.
- No advanced planner rewrite.
- No physically perfect RF/network simulation.
- No training pipeline for 3D AI models yet.
- No server backend; keep it browser-based.

## Validation

- `npm run build` passes.
- Existing single-drone room mode still works.
- Swarm initializes with 3, 6, 12, and 24 drones.
- Drones maintain separation and avoid identical goal assignment.
- Communication graph visibly changes as drones spread.
- Adaptive mode changes topology based on environment and AOIs.
- Point-cloud tile and replay exports produce usable files.
