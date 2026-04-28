import * as THREE from 'three';
import { clampInt } from '../core/math.js';
import { CommunicationGraph } from './communication-graph.js';
import { DEFAULT_SWARM_CONFIG, DRONE_ROLES, FORMATION_MODES } from './constants.js';
import { DroneAgent } from './drone-agent.js';
import { FormationGraph } from './formation-graph.js';
import { MapFusion } from './map-fusion.js';
import { TaskAllocator } from './task-allocator.js';
import { TerrainClassifier } from './terrain-classifier.js';

export class SwarmController {
  constructor(config = {}) {
    this.config = { ...DEFAULT_SWARM_CONFIG, ...config };
    this.agents = [];
    this.communicationGraph = new CommunicationGraph({
      range: this.config.communicationRange,
      maxNeighbors: this.config.maxNeighbors,
      dropout: this.config.communicationDropout,
      latencyMs: this.config.communicationLatencyMs
    });
    this.formationGraph = new FormationGraph({ spacing: this.config.spacing });
    this.terrainClassifier = new TerrainClassifier();
    this.taskAllocator = new TaskAllocator();
    this.mapFusion = new MapFusion();
    this.metrics = {
      coverage: 0,
      knownVoxels: 0,
      pointSamples: 0,
      aoiCount: 0,
      connectedComponents: 0,
      formationMode: FORMATION_MODES.ADAPTIVE
    };
  }

  initializeAgents({ count = this.config.droneCount, origin = new THREE.Vector3(), createMesh = null } = {}) {
    const safeCount = clampInt(count, this.config.minDrones, this.config.maxDrones);
    const roles = this.rolePlan(safeCount);
    this.agents = Array.from({ length: safeCount }, (_, index) => {
      const columns = Math.ceil(Math.sqrt(safeCount));
      const rows = Math.ceil(safeCount / columns);
      const col = index % columns;
      const row = Math.floor(index / columns);
      const spawnSpacing = Math.max(this.config.spacing * 0.9, 1.2);
      const position = origin.clone().add(new THREE.Vector3(
        (col - (columns - 1) * 0.5) * spawnSpacing,
        0,
        (row - (rows - 1) * 0.5) * spawnSpacing
      ));
      return new DroneAgent({
        id: `drone-${index + 1}`,
        role: roles[index],
        position,
        mesh: createMesh ? createMesh(index, roles[index]) : null
      });
    });
    return this.agents;
  }

  rolePlan(count) {
    const roles = [DRONE_ROLES.SCOUT, DRONE_ROLES.MAPPER, DRONE_ROLES.RELAY, DRONE_ROLES.VERIFIER];
    return Array.from({ length: count }, (_, index) => roles[index % roles.length]);
  }

  update({ requestedFormation = this.config.formationMode, environmentSignals = {}, frontiers = [], aois = [], relayTargets = [], linkValidator = null } = {}) {
    const communication = this.communicationGraph.update(this.agents, { canLink: linkValidator });
    const connectedComponents = communication.components.length;
    const communicationHealth = this.agents.length <= 1 ? 1 : 1 / Math.max(connectedComponents, 1);
    const terrainClass = this.terrainClassifier.classify({
      ...environmentSignals,
      frontierDensity: environmentSignals.frontierDensity ?? frontiers.length / Math.max(this.agents.length * 8, 1),
      aoiCount: aois.length
    });
    const formationMode = this.formationGraph.resolveMode({
      requestedMode: requestedFormation,
      terrainClass,
      communicationHealth,
      aoiCount: aois.length,
      frontierDensity: environmentSignals.frontierDensity ?? 0
    });
    const assignments = this.taskAllocator.assign({ agents: this.agents, frontiers, aois, relayTargets });
    assignments.forEach((assignment) => {
      const agent = this.agents.find((candidate) => candidate.id === assignment.droneId);
      agent?.assign(assignment.task);
    });

    this.agents.forEach((agent) => {
      this.mapFusion.ingestMapDeltas(agent);
      this.mapFusion.ingestPointObservations(agent);
    });

    this.metrics = {
      ...this.metrics,
      aoiCount: aois.length,
      connectedComponents,
      formationMode,
      pointSamples: this.mapFusion.exportPointTiles().reduce((sum, tile) => sum + tile.pointCount, 0)
    };

    return {
      terrainClass,
      communication,
      formationMode,
      assignments,
      metrics: { ...this.metrics }
    };
  }

  buildFormationTargets({ center = new THREE.Vector3(), heading = new THREE.Vector3(1, 0, 0), radius = 4 } = {}) {
    return this.formationGraph.buildTargets({
      mode: this.metrics.formationMode,
      agents: this.agents,
      center,
      heading,
      radius,
      verticalSpan: this.config.verticalSpan ?? 2
    });
  }

  snapshot() {
    return {
      config: { ...this.config },
      agents: this.agents.map((agent) => agent.snapshot()),
      communication: this.communicationGraph.snapshot(),
      metrics: { ...this.metrics },
      pointTiles: this.mapFusion.exportPointTiles()
    };
  }
}
