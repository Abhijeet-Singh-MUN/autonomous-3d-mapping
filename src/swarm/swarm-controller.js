import * as THREE from 'three';
import { clampInt } from '../core/math.js';
import {
  adaptiveNeighborTarget,
  BEHAVIOR_PRIMITIVES,
  computeControllerState,
  computeBehaviorWeights,
  DEFAULT_SWARM_BEHAVIOR_PROFILE,
  deriveBehaviorProfile,
  DEFAULT_POLICY_COORDINATES,
  normalizePolicyCoordinates,
  roleCountTargets
} from './behavior-profile.js';
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
    this.baseBehaviorProfile = config.behaviorProfile ?? DEFAULT_SWARM_BEHAVIOR_PROFILE;
    this.policyCoordinates = normalizePolicyCoordinates(this.baseBehaviorProfile.policyCoordinates ?? DEFAULT_POLICY_COORDINATES);
    this.behaviorProfile = deriveBehaviorProfile(this.baseBehaviorProfile, this.policyCoordinates);
    this.agents = [];
    this.communicationGraph = new CommunicationGraph({
      range: this.config.communicationRange,
      maxNeighbors: this.config.maxNeighbors,
      dropout: this.config.communicationDropout,
      latencyMs: this.config.communicationLatencyMs
    });
    this.formationGraph = new FormationGraph({ spacing: this.config.spacing, behaviorProfile: this.behaviorProfile });
    this.terrainClassifier = new TerrainClassifier();
    this.taskAllocator = new TaskAllocator({ behaviorProfile: this.behaviorProfile });
    this.mapFusion = new MapFusion();
    this.metrics = {
      coverage: 0,
      knownVoxels: 0,
      pointSamples: 0,
      aoiCount: 0,
      connectedComponents: 0,
      formationMode: FORMATION_MODES.ADAPTIVE,
      behaviorWeights: computeBehaviorWeights({}, this.behaviorProfile),
      normalizedSignals: {},
      derivedControls: {},
      basePolicyCoordinates: this.behaviorProfile.basePolicyCoordinates,
      policyCoordinates: this.behaviorProfile.policyCoordinates,
      runtimeNudgeProfile: this.behaviorProfile.runtimeNudgeProfile,
      runtimeNudgeScale: this.behaviorProfile.runtimeNudgeScale,
      runtimeNudge: this.behaviorProfile.runtimeNudge,
      effectivePolicyCoordinates: this.behaviorProfile.effectivePolicyCoordinates,
      deltaPolicyCoordinates: this.behaviorProfile.deltaPolicyCoordinates,
      derivedProfileSummary: this.behaviorProfile.derivedProfileSummary,
      dependencyGraph: [],
      modelFamily: this.behaviorProfile.modelFamily,
      modelVersion: this.behaviorProfile.modelVersion,
      behaviorProfileVersion: this.behaviorProfile.version,
      adaptiveNeighborTarget: this.config.maxNeighbors
    };
  }

  setBehaviorProfile(profile) {
    this.baseBehaviorProfile = profile ?? DEFAULT_SWARM_BEHAVIOR_PROFILE;
    this.policyCoordinates = normalizePolicyCoordinates(this.baseBehaviorProfile.policyCoordinates ?? DEFAULT_POLICY_COORDINATES);
    this.behaviorProfile = deriveBehaviorProfile(this.baseBehaviorProfile, this.policyCoordinates);
    this.formationGraph.behaviorProfile = this.behaviorProfile;
    this.taskAllocator.behaviorProfile = this.behaviorProfile;
    this.metrics.behaviorProfileVersion = this.behaviorProfile.version;
    this.metrics.modelFamily = this.behaviorProfile.modelFamily;
    this.metrics.modelVersion = this.behaviorProfile.modelVersion;
    this.metrics.basePolicyCoordinates = this.behaviorProfile.basePolicyCoordinates;
    this.metrics.policyCoordinates = this.behaviorProfile.policyCoordinates;
    this.metrics.runtimeNudgeProfile = this.behaviorProfile.runtimeNudgeProfile;
    this.metrics.runtimeNudgeScale = this.behaviorProfile.runtimeNudgeScale;
    this.metrics.runtimeNudge = this.behaviorProfile.runtimeNudge;
    this.metrics.effectivePolicyCoordinates = this.behaviorProfile.effectivePolicyCoordinates;
    this.metrics.deltaPolicyCoordinates = this.behaviorProfile.deltaPolicyCoordinates;
    this.metrics.derivedProfileSummary = this.behaviorProfile.derivedProfileSummary;
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
    this.communicationGraph.maxNeighbors = adaptiveNeighborTarget({
      baseNeighbors: this.config.maxNeighbors,
      communicationHealth: this.metrics.communicationHealth ?? 1,
      aoiCount: aois.length,
      frontierDensity: environmentSignals.frontierDensity ?? 0,
      agentCount: this.agents.length
    }, this.behaviorProfile);
    const communication = this.communicationGraph.update(this.agents, { canLink: linkValidator });
    const connectedComponents = communication.components.length;
    const communicationHealth = this.agents.length <= 1 ? 1 : 1 / Math.max(connectedComponents, 1);
    const averageDegree = this.agents.length
      ? (communication.edges.length * 2) / this.agents.length
      : 0;
    const localCommunicationHealth = Math.min(1, averageDegree / Math.max(this.communicationGraph.maxNeighbors, 1));
    const terrainClass = this.terrainClassifier.classify({
      ...environmentSignals,
      frontierDensity: environmentSignals.frontierDensity ?? frontiers.length / Math.max(this.agents.length * 8, 1),
      aoiCount: aois.length
    });
    const measuredCommunicationHealth = Math.min(communicationHealth, localCommunicationHealth);
    const profileSignals = {
      ...environmentSignals,
      communicationHealth: measuredCommunicationHealth,
      networkPressure: 1 - measuredCommunicationHealth,
      aoiCount: aois.length
    };
    this.behaviorProfile = deriveBehaviorProfile(this.baseBehaviorProfile, this.policyCoordinates, profileSignals);
    this.formationGraph.behaviorProfile = this.behaviorProfile;
    this.taskAllocator.behaviorProfile = this.behaviorProfile;
    const controllerState = computeControllerState({
      signals: profileSignals,
      baseNeighbors: this.config.maxNeighbors,
      agentCount: this.agents.length
    }, this.behaviorProfile);
    this.communicationGraph.maxNeighbors = controllerState.derivedControls.adaptiveNeighborTarget;
    const behaviorWeights = controllerState.behaviorWeights;
    const normalizedSignals = controllerState.normalizedSignals;
    const derivedControls = controllerState.derivedControls;
    const dependencyGraph = controllerState.dependencyGraph;
    const roleSignals = {
      ...environmentSignals,
      communicationHealth: measuredCommunicationHealth,
      aoiCount: aois.length
    };
    this.rebalanceRoles({
      terrainClass,
      communicationHealth: measuredCommunicationHealth,
      frontierDensity: environmentSignals.frontierDensity ?? 0,
      aoiCount: aois.length,
      behaviorWeights,
      normalizedSignals: roleSignals
    });
    this.updateAgentBehavior({
      communication,
      communicationHealth: measuredCommunicationHealth,
      aoiCount: aois.length,
      frontierDensity: environmentSignals.frontierDensity ?? 0,
      behaviorWeights,
      normalizedSignals,
      derivedControls
    });
    const formationMode = this.formationGraph.resolveMode({
      requestedMode: requestedFormation,
      terrainClass,
      communicationHealth: measuredCommunicationHealth,
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
      communicationHealth: measuredCommunicationHealth,
      behaviorWeights,
      normalizedSignals,
      derivedControls,
      basePolicyCoordinates: controllerState.basePolicyCoordinates,
      policyCoordinates: controllerState.policyCoordinates,
      runtimeNudgeProfile: controllerState.runtimeNudgeProfile,
      runtimeNudgeScale: controllerState.runtimeNudgeScale,
      runtimeNudge: controllerState.runtimeNudge,
      effectivePolicyCoordinates: controllerState.effectivePolicyCoordinates,
      deltaPolicyCoordinates: controllerState.deltaPolicyCoordinates,
      derivedProfileSummary: controllerState.derivedProfileSummary,
      dependencyGraph,
      modelFamily: controllerState.modelFamily,
      modelVersion: controllerState.modelVersion,
      behaviorProfileVersion: this.behaviorProfile.version,
      adaptiveNeighborTarget: this.communicationGraph.maxNeighbors,
      pointSamples: this.mapFusion.exportPointTiles().reduce((sum, tile) => sum + tile.pointCount, 0)
    };

    return {
      terrainClass,
      communication,
      formationMode,
      behaviorWeights,
      normalizedSignals,
      derivedControls,
      dependencyGraph,
      assignments,
      metrics: { ...this.metrics }
    };
  }

  rebalanceRoles({ terrainClass, communicationHealth, frontierDensity, aoiCount }) {
    const count = this.agents.length;
    if (!count) {
      return;
    }

    const roleCounts = roleCountTargets({
      count,
      terrainClass,
      communicationHealth,
      frontierDensity,
      aoiCount
    }, this.behaviorProfile);
    const desiredRoles = [
      ...Array(roleCounts[DRONE_ROLES.RELAY]).fill(DRONE_ROLES.RELAY),
      ...Array(roleCounts[DRONE_ROLES.VERIFIER]).fill(DRONE_ROLES.VERIFIER),
      ...Array(roleCounts[DRONE_ROLES.SCOUT]).fill(DRONE_ROLES.SCOUT),
      ...Array(roleCounts[DRONE_ROLES.MAPPER]).fill(DRONE_ROLES.MAPPER)
    ];

    this.agents.forEach((agent, index) => {
      agent.setRole(desiredRoles[index] ?? DRONE_ROLES.MAPPER);
    });
  }

  updateAgentBehavior({ communication, communicationHealth, aoiCount, frontierDensity, behaviorWeights, normalizedSignals, derivedControls }) {
    const strongestNeighbors = new Map();
    communication.edges.forEach((edge) => {
      [
        [edge.from, edge.to],
        [edge.to, edge.from]
      ].forEach(([from, to]) => {
        const current = strongestNeighbors.get(from);
        if (!current || edge.distance < current.distance) {
          strongestNeighbors.set(from, { id: to, distance: edge.distance });
        }
      });
    });

    this.agents.forEach((agent) => {
      const neighbor = strongestNeighbors.get(agent.id);
      const aoiWeight = behaviorWeights?.[BEHAVIOR_PRIMITIVES.AOI] ?? 0;
      const frontierWeight = behaviorWeights?.[BEHAVIOR_PRIMITIVES.FRONTIER] ?? 0;
      const focus = agent.role === DRONE_ROLES.RELAY
        ? 'network'
        : agent.role === DRONE_ROLES.VERIFIER || (agent.role === DRONE_ROLES.MAPPER && (aoiCount > 0 || aoiWeight > 0.22))
          ? 'aoi'
          : frontierDensity > 0.02 || frontierWeight > 0.18
            ? 'frontier'
            : 'coverage';
      agent.updateBehavior({
        sensingFocus: focus,
        communicationHealth,
        networkAnchor: neighbor?.id ?? null,
        behaviorWeights: behaviorWeights ? { ...behaviorWeights } : null,
        normalizedSignals: normalizedSignals ? { ...normalizedSignals } : null,
        derivedControls: derivedControls ? { ...derivedControls } : null
      });
    });
  }

  buildFormationTargets({ center = new THREE.Vector3(), heading = new THREE.Vector3(1, 0, 0), radius = 4, verticalSpan = this.config.verticalSpan ?? 2 } = {}) {
    return this.formationGraph.buildTargets({
      mode: this.metrics.formationMode,
      agents: this.agents,
      center,
      heading,
      radius,
      verticalSpan
    });
  }

  snapshot() {
    return {
      config: { ...this.config },
      behaviorProfile: {
        version: this.behaviorProfile.version,
        objective: this.behaviorProfile.objectives.default,
        modelFamily: this.behaviorProfile.modelFamily,
        modelVersion: this.behaviorProfile.modelVersion,
        basePolicyCoordinates: { ...this.behaviorProfile.basePolicyCoordinates },
        policyCoordinates: { ...this.behaviorProfile.policyCoordinates },
        runtimeNudgeProfile: this.behaviorProfile.runtimeNudgeProfile,
        runtimeNudgeScale: this.behaviorProfile.runtimeNudgeScale,
        runtimeNudge: { ...this.behaviorProfile.runtimeNudge },
        effectivePolicyCoordinates: { ...this.behaviorProfile.effectivePolicyCoordinates },
        deltaPolicyCoordinates: { ...this.behaviorProfile.deltaPolicyCoordinates },
        derivedProfileSummary: { ...this.behaviorProfile.derivedProfileSummary }
      },
      agents: this.agents.map((agent) => agent.snapshot()),
      communication: this.communicationGraph.snapshot(),
      metrics: { ...this.metrics },
      pointTiles: this.mapFusion.exportPointTiles()
    };
  }
}
