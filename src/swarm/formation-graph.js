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

  buildTargets({ mode, agents, center = new THREE.Vector3(), heading = new THREE.Vector3(1, 0, 0), radius = 4 }) {
    const normalizedHeading = heading.clone().setY(0);
    if (normalizedHeading.lengthSq() < 1e-6) {
      normalizedHeading.set(1, 0, 0);
    }
    normalizedHeading.normalize();

    const side = new THREE.Vector3(-normalizedHeading.z, 0, normalizedHeading.x);

    switch (mode) {
      case FORMATION_MODES.LINE_SWEEP:
        return this.lineSweepTargets(agents, center, side);
      case FORMATION_MODES.WEDGE:
        return this.wedgeTargets(agents, center, normalizedHeading, side);
      case FORMATION_MODES.RELAY_CHAIN:
        return this.relayChainTargets(agents, center, normalizedHeading);
      case FORMATION_MODES.PERIMETER_RING:
        return this.perimeterRingTargets(agents, center, radius);
      case FORMATION_MODES.SCATTER:
      default:
        return this.scatterTargets(agents, center, radius);
    }
  }

  lineSweepTargets(agents, center, side) {
    const offset = (agents.length - 1) * 0.5;
    return agents.map((agent, index) => ({
      droneId: agent.id,
      position: center.clone().addScaledVector(side, (index - offset) * this.spacing)
    }));
  }

  wedgeTargets(agents, center, heading, side) {
    return agents.map((agent, index) => {
      if (index === 0) {
        return { droneId: agent.id, position: center.clone().addScaledVector(heading, this.spacing) };
      }
      const rank = Math.ceil(index / 2);
      const direction = index % 2 === 0 ? 1 : -1;
      return {
        droneId: agent.id,
        position: center
          .clone()
          .addScaledVector(heading, -rank * this.spacing)
          .addScaledVector(side, direction * rank * this.spacing)
      };
    });
  }

  relayChainTargets(agents, center, heading) {
    return agents.map((agent, index) => ({
      droneId: agent.id,
      position: center.clone().addScaledVector(heading, (index - (agents.length - 1) * 0.5) * this.spacing)
    }));
  }

  perimeterRingTargets(agents, center, radius) {
    return agents.map((agent, index) => {
      const angle = (index / Math.max(agents.length, 1)) * Math.PI * 2;
      return {
        droneId: agent.id,
        position: center.clone().add(new THREE.Vector3(Math.cos(angle) * radius, 0, Math.sin(angle) * radius))
      };
    });
  }

  scatterTargets(agents, center, radius) {
    const goldenAngle = Math.PI * (3 - Math.sqrt(5));
    return agents.map((agent, index) => {
      const distance = Math.sqrt(index + 1) * (radius / Math.sqrt(Math.max(agents.length, 1)));
      const angle = index * goldenAngle;
      return {
        droneId: agent.id,
        position: center.clone().add(new THREE.Vector3(Math.cos(angle) * distance, 0, Math.sin(angle) * distance))
      };
    });
  }
}
