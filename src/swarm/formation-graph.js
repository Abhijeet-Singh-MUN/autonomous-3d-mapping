import * as THREE from 'three';
import { FORMATION_MODES, TERRAIN_CLASSES } from './constants.js';

export class FormationGraph {
  constructor({ spacing }) {
    this.spacing = spacing;
    this.currentMode = FORMATION_MODES.ADAPTIVE;
  }

  resolveMode({ requestedMode, terrainClass, communicationHealth, aoiCount, frontierDensity }) {
    if (requestedMode && requestedMode !== FORMATION_MODES.ADAPTIVE) {
      this.currentMode = requestedMode;
      return requestedMode;
    }

    if (communicationHealth < 0.72) {
      this.currentMode = FORMATION_MODES.RELAY_CHAIN;
    } else if (aoiCount > 0) {
      this.currentMode = FORMATION_MODES.PERIMETER_RING;
    } else if (terrainClass === TERRAIN_CLASSES.CORRIDOR || terrainClass === TERRAIN_CLASSES.DENSE_URBAN) {
      this.currentMode = FORMATION_MODES.WEDGE;
    } else if (terrainClass === TERRAIN_CLASSES.SPARSE_OPEN && frontierDensity < 0.35) {
      this.currentMode = FORMATION_MODES.LINE_SWEEP;
    } else {
      this.currentMode = FORMATION_MODES.SCATTER;
    }

    return this.currentMode;
  }

  buildTargets({ mode, agents, center = new THREE.Vector3(), heading = new THREE.Vector3(1, 0, 0), radius = 4, verticalSpan = 2 }) {
    const normalizedHeading = heading.clone().setY(0);
    if (normalizedHeading.lengthSq() < 1e-6) {
      normalizedHeading.set(1, 0, 0);
    }
    normalizedHeading.normalize();

    const side = new THREE.Vector3(-normalizedHeading.z, 0, normalizedHeading.x);

    switch (mode) {
      case FORMATION_MODES.LINE_SWEEP:
        return this.lineSweepTargets(agents, center, side, verticalSpan);
      case FORMATION_MODES.WEDGE:
        return this.wedgeTargets(agents, center, normalizedHeading, side, verticalSpan);
      case FORMATION_MODES.RELAY_CHAIN:
        return this.relayChainTargets(agents, center, normalizedHeading, verticalSpan);
      case FORMATION_MODES.PERIMETER_RING:
        return this.perimeterRingTargets(agents, center, radius, verticalSpan);
      case FORMATION_MODES.SCATTER:
      default:
        return this.scatterTargets(agents, center, radius, verticalSpan);
    }
  }

  lineSweepTargets(agents, center, side, verticalSpan) {
    const offset = (agents.length - 1) * 0.5;
    return agents.map((agent, index) => ({
      droneId: agent.id,
      position: center
        .clone()
        .addScaledVector(side, (index - offset) * this.spacing)
        .add(new THREE.Vector3(0, this.layerOffset(index, verticalSpan), 0))
    }));
  }

  wedgeTargets(agents, center, heading, side, verticalSpan) {
    return agents.map((agent, index) => {
      if (index === 0) {
        return { droneId: agent.id, position: center.clone().addScaledVector(heading, this.spacing).add(new THREE.Vector3(0, verticalSpan * 0.22, 0)) };
      }
      const rank = Math.ceil(index / 2);
      const direction = index % 2 === 0 ? 1 : -1;
      return {
        droneId: agent.id,
        position: center
          .clone()
          .addScaledVector(heading, -rank * this.spacing)
          .addScaledVector(side, direction * rank * this.spacing)
          .add(new THREE.Vector3(0, this.layerOffset(index, verticalSpan), 0))
      };
    });
  }

  relayChainTargets(agents, center, heading, verticalSpan) {
    return agents.map((agent, index) => ({
      droneId: agent.id,
      position: center
        .clone()
        .addScaledVector(heading, (index - (agents.length - 1) * 0.5) * this.spacing)
        .add(new THREE.Vector3(0, (index / Math.max(agents.length - 1, 1) - 0.5) * verticalSpan, 0))
    }));
  }

  perimeterRingTargets(agents, center, radius, verticalSpan) {
    return agents.map((agent, index) => {
      const angle = (index / Math.max(agents.length, 1)) * Math.PI * 2;
      return {
        droneId: agent.id,
        position: center.clone().add(new THREE.Vector3(
          Math.cos(angle) * radius,
          Math.sin(angle * 2) * verticalSpan * 0.28,
          Math.sin(angle) * radius
        ))
      };
    });
  }

  scatterTargets(agents, center, radius, verticalSpan) {
    const goldenAngle = Math.PI * (3 - Math.sqrt(5));
    return agents.map((agent, index) => {
      const distance = Math.sqrt(index + 1) * (radius / Math.sqrt(Math.max(agents.length, 1)));
      const angle = index * goldenAngle;
      const vertical = ((index * 2) % 5 - 2) * verticalSpan * 0.18;
      return {
        droneId: agent.id,
        position: center.clone().add(new THREE.Vector3(Math.cos(angle) * distance, vertical, Math.sin(angle) * distance))
      };
    });
  }

  layerOffset(index, verticalSpan) {
    return ((index % 3) - 1) * verticalSpan * 0.2;
  }
}
