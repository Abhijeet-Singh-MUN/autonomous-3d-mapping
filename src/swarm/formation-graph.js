import * as THREE from 'three';
import { DEFAULT_SWARM_BEHAVIOR_PROFILE } from './behavior-profile.js';
import { FORMATION_MODES, TERRAIN_CLASSES } from './constants.js';

export class FormationGraph {
  constructor({ spacing, behaviorProfile = DEFAULT_SWARM_BEHAVIOR_PROFILE }) {
    this.spacing = spacing;
    this.behaviorProfile = behaviorProfile;
    this.currentMode = FORMATION_MODES.ADAPTIVE;
    this.targetMemory = new Map();
  }

  resolveMode({ requestedMode, terrainClass, communicationHealth, aoiCount, frontierDensity }) {
    if (requestedMode && requestedMode !== FORMATION_MODES.ADAPTIVE) {
      this.currentMode = requestedMode;
      return requestedMode;
    }

    const formation = this.behaviorProfile.formation;
    if (communicationHealth < formation.weakCommunicationModeThreshold) {
      this.currentMode = FORMATION_MODES.RELAY_CHAIN;
    } else if (aoiCount > 0) {
      this.currentMode = FORMATION_MODES.PERIMETER_RING;
    } else if (terrainClass === TERRAIN_CLASSES.CORRIDOR || terrainClass === TERRAIN_CLASSES.DENSE_URBAN) {
      this.currentMode = FORMATION_MODES.WEDGE;
    } else if (terrainClass === TERRAIN_CLASSES.SPARSE_OPEN && frontierDensity < formation.lineSweepMaxFrontierDensity) {
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

    let targets;
    switch (mode) {
      case FORMATION_MODES.LINE_SWEEP:
        targets = this.lineSweepTargets(agents, center, side, verticalSpan);
        break;
      case FORMATION_MODES.WEDGE:
        targets = this.wedgeTargets(agents, center, normalizedHeading, side, verticalSpan);
        break;
      case FORMATION_MODES.RELAY_CHAIN:
        targets = this.relayChainTargets(agents, center, normalizedHeading, verticalSpan);
        break;
      case FORMATION_MODES.PERIMETER_RING:
        targets = this.perimeterRingTargets(agents, center, radius, verticalSpan);
        break;
      case FORMATION_MODES.SCATTER:
      default:
        targets = this.scatterTargets(agents, center, radius, verticalSpan);
        break;
    }

    return this.smoothTargets(targets, mode);
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

  smoothTargets(targets, mode) {
    const retainedIds = new Set(targets.map((target) => target.droneId));
    this.targetMemory.forEach((_, droneId) => {
      if (!retainedIds.has(droneId)) {
        this.targetMemory.delete(droneId);
      }
    });

    const formation = this.behaviorProfile.formation;
    const blend = mode === this.currentMode ? formation.targetBlendSameMode : formation.targetBlendModeChange;
    return targets.map((target) => {
      const previous = this.targetMemory.get(target.droneId);
      const position = previous ? previous.clone().lerp(target.position, blend) : target.position.clone();
      this.targetMemory.set(target.droneId, position.clone());
      return { ...target, position };
    });
  }

  layerOffset(index, verticalSpan) {
    return ((index % 3) - 1) * verticalSpan * 0.2;
  }
}
