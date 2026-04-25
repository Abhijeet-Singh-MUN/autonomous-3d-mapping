export const CELL_UNKNOWN = 0;
export const CELL_FREE = 1;
export const CELL_OCCUPIED = 2;

export const SENSOR_PRESETS = {
  spinning: {
    label: 'Spinning LiDAR',
    mode: 'spinning',
    horizontalRays: 96,
    verticalRays: 24,
    horizontalFov: 360,
    verticalFov: 55,
    maxRange: 10,
    pitchBiasDeg: 0
  },
  forward: {
    label: 'Forward LiDAR',
    mode: 'forward',
    horizontalRays: 72,
    verticalRays: 28,
    horizontalFov: 120,
    verticalFov: 55,
    maxRange: 9,
    pitchBiasDeg: -6
  },
  depthcam: {
    label: 'Depth Camera',
    mode: 'forward',
    horizontalRays: 80,
    verticalRays: 48,
    horizontalFov: 90,
    verticalFov: 58,
    maxRange: 8,
    pitchBiasDeg: -12
  },
  custom: {
    label: 'Custom',
    mode: 'forward',
    horizontalRays: null,
    verticalRays: null,
    horizontalFov: null,
    verticalFov: null,
    maxRange: null,
    pitchBiasDeg: 0
  }
};

export const AXIAL_NEIGHBORS_3D = [
  [1, 0, 0],
  [-1, 0, 0],
  [0, 1, 0],
  [0, -1, 0],
  [0, 0, 1],
  [0, 0, -1]
];
