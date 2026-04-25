import * as THREE from 'three';
import { CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN } from './constants.js';

export class VoxelGrid {
  constructor({ room, resolution, margin }) {
    this.res = resolution;
    this.cols = Math.ceil((room.width + margin * 2) / resolution) + 1;
    this.rows = Math.ceil((room.depth + margin * 2) / resolution) + 1;
    this.layers = Math.ceil((room.height + margin * 2) / resolution) + 1;
    this.originX = -room.width * 0.5 - margin;
    this.originY = -margin;
    this.originZ = -room.depth * 0.5 - margin;

    const count = this.cols * this.rows * this.layers;
    this.states = new Int8Array(count).fill(CELL_UNKNOWN);
    this.visits = new Uint16Array(count);
    this.clearance = new Float32Array(count);
    this.traversable = new Uint8Array(count);
    this.frontierIndices = [];
    this.inspectionIndices = [];
    this.knownCount = 0;
    this.freeCount = 0;
    this.occupiedCount = 0;
  }

  worldToVoxel(x, y, z) {
    const col = Math.round((x - this.originX) / this.res);
    const row = Math.round((z - this.originZ) / this.res);
    const layer = Math.round((y - this.originY) / this.res);
    if (!this.inBounds(col, row, layer)) {
      return null;
    }
    return { col, row, layer };
  }

  voxelToWorld(col, row, layer) {
    return new THREE.Vector3(
      this.originX + col * this.res,
      this.originY + layer * this.res,
      this.originZ + row * this.res
    );
  }

  packIndex(col, row, layer) {
    return col + row * this.cols + layer * this.cols * this.rows;
  }

  unpackIndex(index) {
    const layerStride = this.cols * this.rows;
    const layer = Math.floor(index / layerStride);
    const withinLayer = index - layer * layerStride;
    const row = Math.floor(withinLayer / this.cols);
    const col = withinLayer - row * this.cols;
    return { col, row, layer };
  }

  inBounds(col, row, layer) {
    return (
      col >= 0 &&
      row >= 0 &&
      layer >= 0 &&
      col < this.cols &&
      row < this.rows &&
      layer < this.layers
    );
  }

  getState(col, row, layer) {
    if (!this.inBounds(col, row, layer)) {
      return CELL_OCCUPIED;
    }
    return this.states[this.packIndex(col, row, layer)];
  }

  markWorld(x, y, z, state) {
    const cell = this.worldToVoxel(x, y, z);
    if (!cell) {
      return;
    }
    this.markState(cell.col, cell.row, cell.layer, state);
  }

  markState(col, row, layer, state) {
    if (!this.inBounds(col, row, layer)) {
      return;
    }
    const index = this.packIndex(col, row, layer);
    const current = this.states[index];

    if (state === CELL_FREE) {
      if (current === CELL_UNKNOWN) {
        this.states[index] = CELL_FREE;
        this.knownCount += 1;
        this.freeCount += 1;
      }
      return;
    }

    if (state === CELL_OCCUPIED) {
      if (current === CELL_UNKNOWN) {
        this.knownCount += 1;
        this.occupiedCount += 1;
      } else if (current === CELL_FREE) {
        this.freeCount -= 1;
        this.occupiedCount += 1;
      }
      this.states[index] = CELL_OCCUPIED;
    }
  }
}
