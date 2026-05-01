export {
  adaptiveNeighborTarget,
  BEHAVIOR_PRIMITIVES,
  computeBehaviorWeights,
  computeControllerState,
  computeDerivedControls,
  DEFAULT_SWARM_EVALUATION_PROFILE,
  DEFAULT_SWARM_BEHAVIOR_PROFILE,
  normalizeSwarmSignals,
  normalizeWeights,
  OPTIMIZER_PARAMETER_REGISTRY,
  PARAMETER_DEPENDENCY_GRAPH,
  roleCountTargets,
  scoreSwarmRun
} from './behavior-profile.js';
export { CommunicationGraph } from './communication-graph.js';
export { DEFAULT_SWARM_CONFIG, DRONE_ROLES, FORMATION_MODES, TERRAIN_CLASSES, AOI_TYPES } from './constants.js';
export { DroneAgent } from './drone-agent.js';
export { FormationGraph } from './formation-graph.js';
export { MapFusion } from './map-fusion.js';
export { DEFAULT_DRONE_RESOURCE_MODEL, estimateSwarmResourceUse } from './resource-model.js';
export { SwarmRunTelemetry } from './run-telemetry.js';
export { SwarmController } from './swarm-controller.js';
export { TaskAllocator } from './task-allocator.js';
export { TerrainClassifier } from './terrain-classifier.js';
