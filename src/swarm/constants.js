export const DRONE_ROLES = {
  SCOUT: 'scout',
  MAPPER: 'mapper',
  RELAY: 'relay',
  VERIFIER: 'verifier'
};

export const FORMATION_MODES = {
  ADAPTIVE: 'adaptive',
  SCATTER: 'scatter',
  LINE_SWEEP: 'line_sweep',
  WEDGE: 'wedge',
  RELAY_CHAIN: 'relay_chain',
  PERIMETER_RING: 'perimeter_ring'
};

export const TERRAIN_CLASSES = {
  SPARSE_OPEN: 'sparse_open',
  DENSE_URBAN: 'dense_urban',
  CORRIDOR: 'corridor',
  VERTICAL_STRUCTURE: 'vertical_structure',
  OCCLUDED: 'occluded',
  AOI_RICH: 'aoi_rich'
};

export const AOI_TYPES = {
  FRONTIER_CLUSTER: 'frontier_cluster',
  OCCLUSION: 'occlusion',
  VERTICAL_FEATURE: 'vertical_feature',
  USER_MARKED: 'user_marked',
  LOW_CONFIDENCE_TILE: 'low_confidence_tile'
};

export const DEFAULT_SWARM_CONFIG = {
  droneCount: 6,
  minDrones: 3,
  maxDrones: 24,
  communicationRange: 8,
  maxNeighbors: 3,
  communicationDropout: 0,
  communicationLatencyMs: 120,
  formationMode: FORMATION_MODES.ADAPTIVE,
  spacing: 1.8
};
