import { TERRAIN_CLASSES } from './constants.js';

export class TerrainClassifier {
  classify({
    frontierDensity = 0,
    obstacleDensity = 0,
    verticality = 0,
    occlusion = 0,
    corridorScore = 0,
    aoiCount = 0
  }) {
    if (aoiCount > 0) {
      return TERRAIN_CLASSES.AOI_RICH;
    }
    if (corridorScore > 0.65) {
      return TERRAIN_CLASSES.CORRIDOR;
    }
    if (verticality > 0.6) {
      return TERRAIN_CLASSES.VERTICAL_STRUCTURE;
    }
    if (occlusion > 0.58) {
      return TERRAIN_CLASSES.OCCLUDED;
    }
    if (obstacleDensity > 0.5 || frontierDensity > 0.55) {
      return TERRAIN_CLASSES.DENSE_URBAN;
    }
    return TERRAIN_CLASSES.SPARSE_OPEN;
  }
}
