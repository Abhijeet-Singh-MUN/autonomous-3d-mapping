import { DRONE_ROLES, TERRAIN_CLASSES } from './constants.js';

export const BEHAVIOR_PRIMITIVES = {
  COVERAGE: 'coverage',
  FRONTIER: 'frontier',
  AOI: 'aoi',
  RELAY: 'relay',
  VERIFY: 'verify',
  AVOIDANCE: 'avoidance'
};

export const PARAMETER_DEPENDENCY_GRAPH = [
  { from: 'measuredSignals', to: 'normalizedSignals', reason: 'Clamp live terrain, AOI, scan, and network inputs to comparable 0..1 ranges.' },
  { from: 'normalizedSignals', to: 'constraintSignals', reason: 'Expose soft risk, time, battery, and compute pressure before behavior mixing.' },
  { from: 'normalizedSignals', to: 'behaviorWeights', reason: 'Compute the continuous primitive mixture on the behavior simplex.' },
  { from: 'behaviorWeights', to: 'derivedControls', reason: 'Translate primitive pressure into bounded runtime gains.' },
  { from: 'derivedControls', to: 'runtimeActions', reason: 'Movement, sensing, network, task, and formation code consume derived controls.' }
];

export const DEFAULT_SWARM_BEHAVIOR_PROFILE = {
  version: 'swarm-behavior-profile-v1',
  roleBalance: {
    weakCommunicationThreshold: 0.72,
    frontierPressureThreshold: 0.08,
    relayBaseShare: 0.16,
    relayWeakCommsShare: 0.28,
    verifierBaseShare: 0.1,
    verifierAoiShare: 0.18,
    scoutDefaultShare: 0.25,
    scoutOpenShare: 0.34,
    scoutFrontierShare: 0.38
  },
  behaviorMixer: {
    coverageBase: 0.2,
    coverageOpenGain: 0.12,
    coverageEarlyScanGain: 0.08,
    frontierGain: 0.32,
    frontierScale: 0.12,
    aoiGain: 0.34,
    relayGain: 0.36,
    verifyGain: 0.2,
    avoidanceGain: 0.28
  },
  constraints: {
    aoiRiskGain: 0.34,
    riskAoiSuppression: 0.18,
    riskAvoidanceBoost: 0.38,
    batteryEfficiencyGain: 0.22,
    missionTimeEfficiencyGain: 0.16,
    computePressureAvoidanceGain: 0.18,
    footprintRedundancySpreadGain: 0.26,
    footprintResolutionTightenGain: 0.14,
    minBatteryReservePct: 0.18,
    nominalMissionMs: 180000
  },
  taskScoring: {
    informationGainWeight: 0.35,
    distancePenalty: 0.08,
    continuityBias: 0.35,
    weakCommunicationThreshold: 0.65,
    communicationRelayBonus: 0.6,
    verifyPriorityScale: 0.82,
    verifyInformationGainScale: 0.72,
    roleWeights: {
      [DRONE_ROLES.SCOUT]: { frontier: 1.55, aoi: 0.72, relay: 0.35, verify: 0.55 },
      [DRONE_ROLES.MAPPER]: { frontier: 0.95, aoi: 1.35, relay: 0.35, verify: 0.8 },
      [DRONE_ROLES.RELAY]: { frontier: 0.38, aoi: 0.42, relay: 1.75, verify: 0.35 },
      [DRONE_ROLES.VERIFIER]: { frontier: 0.55, aoi: 1.1, relay: 0.3, verify: 1.7 }
    }
  },
  movement: {
    targetSmoothingPerSecond: 2.8,
    stopDampingPerSecond: 9,
    steeringSmoothingPerSecond: 5.6,
    slowdownDistance: 3.5,
    minApproachScale: 0.28,
    assignmentBlend: {
      default: 0.24,
      relay: 0.16,
      verify: 0.34,
      mapper: 0.28,
      aoi: 0.28
    },
    roleSpeedScale: {
      [DRONE_ROLES.SCOUT]: 1.08,
      [DRONE_ROLES.MAPPER]: 1,
      [DRONE_ROLES.RELAY]: 0.82,
      [DRONE_ROLES.VERIFIER]: 0.92
    }
  },
  network: {
    minNeighbors: 1,
    maxNeighbors: 8,
    softRangeFactor: 0.9,
    correctionPasses: 4,
    correctionStrength: 0.5,
    neighborCounterPush: 0.35,
    relayCompliance: 1.25,
    scoutCompliance: 0.82,
    weakHealthThreshold: 0.72,
    weakHealthExtraNeighbors: 1,
    aoiExtraNeighbors: 1,
    frontierExtraNeighbors: 0
  },
  formation: {
    weakCommunicationModeThreshold: 0.72,
    lineSweepMaxFrontierDensity: 0.35,
    targetBlendSameMode: 0.34,
    targetBlendModeChange: 0.22
  },
  sensing: {
    focusRangeSensorFactor: 0.9,
    focusRangeRadiusFactor: 1.65,
    focusRangeMin: 18,
    focusStrengthMin: 0.08,
    roleAoiFocusScale: {
      [DRONE_ROLES.SCOUT]: 0.82,
      [DRONE_ROLES.MAPPER]: 1.08,
      [DRONE_ROLES.RELAY]: 0.56,
      [DRONE_ROLES.VERIFIER]: 1.18
    }
  },
  relay: {
    baseZFactor: -0.42,
    spacingRangeFactor: 0.62,
    minimumSpacing: 8,
    maxTasks: 6,
    altitudeArc: 1.8,
    clearanceScale: 1.08,
    priorityBase: 1.15,
    priorityDistanceGain: 0.2,
    informationGain: 0.45
  },
  objectives: {
    default: 'balanced_mapping',
    profiles: {
      balanced_mapping: {
        label: 'Balanced mapping',
        weights: {
          aoiQuality: 0.28,
          coverage: 0.22,
          networkResilience: 0.18,
          timeEfficiency: 0.16,
          computeEfficiency: 0.1,
          energyProxy: 0.06,
          adaptationSmoothness: 0.04,
          constraintSafety: 0.06
        }
      },
      aoi_quality: {
        label: 'AOI quality',
        weights: {
          aoiQuality: 0.46,
          coverage: 0.14,
          networkResilience: 0.14,
          timeEfficiency: 0.12,
          computeEfficiency: 0.08,
          energyProxy: 0.06,
          adaptationSmoothness: 0.04,
          constraintSafety: 0.04
        }
      },
      network_resilience: {
        label: 'Network resilience',
        weights: {
          aoiQuality: 0.18,
          coverage: 0.16,
          networkResilience: 0.4,
          timeEfficiency: 0.1,
          computeEfficiency: 0.08,
          energyProxy: 0.08,
          adaptationSmoothness: 0.04,
          constraintSafety: 0.06
        }
      },
      coverage_exploration: {
        label: 'Coverage exploration',
        weights: {
          aoiQuality: 0.12,
          coverage: 0.44,
          networkResilience: 0.14,
          timeEfficiency: 0.14,
          computeEfficiency: 0.1,
          energyProxy: 0.06,
          adaptationSmoothness: 0.04,
          constraintSafety: 0.04
        }
      },
      compute_efficiency: {
        label: 'Compute efficiency',
        weights: {
          aoiQuality: 0.14,
          coverage: 0.16,
          networkResilience: 0.12,
          timeEfficiency: 0.18,
          computeEfficiency: 0.34,
          energyProxy: 0.06,
          adaptationSmoothness: 0.04,
          constraintSafety: 0.06
        }
      },
      energy_path_efficiency: {
        label: 'Energy/path efficiency',
        weights: {
          aoiQuality: 0.16,
          coverage: 0.16,
          networkResilience: 0.16,
          timeEfficiency: 0.12,
          computeEfficiency: 0.1,
          energyProxy: 0.3,
          adaptationSmoothness: 0.04,
          constraintSafety: 0.08
        }
      }
    }
  }
};

export const DEFAULT_SWARM_EVALUATION_PROFILE = {
  version: 'swarm-evaluation-profile-v1',
  normalizers: {
    aoiRawHits: 180,
    aoiFocusedHits: 90,
    aoiFocusedDwellMs: 8000,
    aoiFirstContactMs: 45000,
    pointVoxels: 14000,
    elapsedMs: 180000,
    avgScanPassMs: 120,
    avgFrameMs: 42,
    energyProxy: 9000,
    totalEnergyWh: 220,
    newPointVoxelsPerSecond: 90,
    aoiHitsPerSecond: 12,
    usefulAoiHitRateAfterContact: 18,
    behaviorWeightChangePerSecond: 0.55,
    networkFragmentedSeconds: 12,
    energyWhPerSecond: 0.09,
    computeWhPerSecond: 0.008,
    riskExposureSeconds: 18,
    uniqueFootprintAreaM2: 1200,
    uniqueFootprintAreaPerSecond: 9,
    footprintRedundancyRatio: 0.55
  },
  validity: {
    minRunMs: 12000,
    minAoiDwellMs: 3000,
    minAoiHits: 40,
    maxStableFrameMs: 80
  }
};

export const OPTIMIZER_PARAMETER_REGISTRY = [
  { key: 'behaviorMixer.frontierGain', min: 0.12, max: 0.58, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.behaviorMixer.frontierGain, group: 'behavior', reason: 'Controls exploration pressure toward frontier tasks.' },
  { key: 'behaviorMixer.aoiGain', min: 0.14, max: 0.7, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.behaviorMixer.aoiGain, group: 'behavior', reason: 'Controls how strongly AOI presence pulls the mixture.' },
  { key: 'behaviorMixer.relayGain', min: 0.14, max: 0.72, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.behaviorMixer.relayGain, group: 'behavior', reason: 'Controls network-preservation pressure under weak communication.' },
  { key: 'behaviorMixer.verifyGain', min: 0.06, max: 0.46, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.behaviorMixer.verifyGain, group: 'behavior', reason: 'Controls rescan/verification pressure after scan progress increases.' },
  { key: 'behaviorMixer.avoidanceGain', min: 0.12, max: 0.56, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.behaviorMixer.avoidanceGain, group: 'behavior', reason: 'Controls obstacle/corridor caution in the behavior mixture.' },
  { key: 'constraints.aoiRiskGain', min: 0.08, max: 0.62, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.constraints.aoiRiskGain, group: 'constraints', reason: 'Controls how much close AOI work increases local risk pressure.' },
  { key: 'constraints.riskAvoidanceBoost', min: 0.12, max: 0.72, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.constraints.riskAvoidanceBoost, group: 'constraints', reason: 'Controls how strongly soft risk increases avoidance behavior.' },
  { key: 'constraints.batteryEfficiencyGain', min: 0.06, max: 0.48, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.constraints.batteryEfficiencyGain, group: 'constraints', reason: 'Controls how low battery pressure biases toward efficient movement.' },
  { key: 'constraints.footprintRedundancySpreadGain', min: 0.08, max: 0.55, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.constraints.footprintRedundancySpreadGain, group: 'constraints', reason: 'Controls how strongly overlapping LiDAR footprints spread drones for area coverage.' },
  { key: 'roleBalance.relayWeakCommsShare', min: 0.18, max: 0.42, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.roleBalance.relayWeakCommsShare, group: 'roles', reason: 'Controls how many drones become relays during weak communication.' },
  { key: 'roleBalance.verifierAoiShare', min: 0.1, max: 0.32, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.roleBalance.verifierAoiShare, group: 'roles', reason: 'Controls verifier share when AOIs exist.' },
  { key: 'network.weakHealthThreshold', min: 0.55, max: 0.88, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.network.weakHealthThreshold, group: 'network', reason: 'Controls when adaptive neighbor pressure activates.' },
  { key: 'sensing.focusRangeRadiusFactor', min: 1.1, max: 2.3, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.sensing.focusRangeRadiusFactor, group: 'sensing', reason: 'Controls AOI focus activation radius relative to AOI size.' },
  { key: 'taskScoring.continuityBias', min: 0.08, max: 0.58, default: DEFAULT_SWARM_BEHAVIOR_PROFILE.taskScoring.continuityBias, group: 'tasking', reason: 'Controls assignment stability versus fast retasking.' }
];

export function clamp01(value) {
  return Math.min(Math.max(value, 0), 1);
}

function clampRange(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

export function normalizeWeights(weights) {
  const entries = Object.entries(weights).map(([key, value]) => [key, Math.max(0, Number.isFinite(value) ? value : 0)]);
  const total = entries.reduce((sum, [, value]) => sum + value, 0);
  if (total <= 1e-8) {
    const fallback = 1 / Math.max(entries.length, 1);
    return Object.fromEntries(entries.map(([key]) => [key, fallback]));
  }
  return Object.fromEntries(entries.map(([key, value]) => [key, value / total]));
}

export function normalizeSwarmSignals(signals = {}, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const mixer = profile.behaviorMixer;
  const constraints = profile.constraints;
  const batteryRemainingPct = signals.batteryRemainingPct === null || signals.batteryRemainingPct === undefined
    ? 1
    : clamp01(signals.batteryRemainingPct);
  const batteryPressure = clamp01((constraints.minBatteryReservePct - batteryRemainingPct) / Math.max(constraints.minBatteryReservePct, 1e-6));
  const missionTimePressure = clamp01((signals.elapsedMs ?? 0) / Math.max(constraints.nominalMissionMs, 1));
  const computePressure = clamp01(Math.max(
    signals.avgFrameMs ? signals.avgFrameMs / 80 : 0,
    signals.avgScanPassMs ? signals.avgScanPassMs / 160 : 0
  ));
  const aoiProximityRisk = clamp01(signals.aoiProximityRisk ?? 0);
  const footprintRedundancy = clamp01(signals.footprintRedundancy ?? 0);
  const footprintResolutionScore = clamp01(signals.footprintResolutionScore ?? 0);
  const footprintResolutionDeficit = clamp01(1 - footprintResolutionScore);
  const riskPressure = clamp01(
    aoiProximityRisk * constraints.aoiRiskGain
    + batteryPressure * constraints.batteryEfficiencyGain
    + missionTimePressure * constraints.missionTimeEfficiencyGain
    + computePressure * constraints.computePressureAvoidanceGain
  );
  return {
    aoiCount: Math.max(0, signals.aoiCount ?? 0),
    aoiPressure: signals.aoiCount > 0 ? 1 : 0,
    scanProgress: clamp01(signals.scanProgress ?? 0),
    frontierDensity: clamp01(signals.frontierDensity ?? 0),
    frontierPressure: clamp01((signals.frontierDensity ?? 0) / Math.max(mixer.frontierScale, 1e-6)),
    obstacleDensity: clamp01(signals.obstacleDensity ?? 0),
    corridorScore: clamp01(signals.corridorScore ?? 0),
    obstaclePressure: clamp01(Math.max(signals.obstacleDensity ?? 0, signals.corridorScore ?? 0)),
    openness: clamp01(signals.openness ?? 0.5),
    verticality: clamp01(signals.verticality ?? 0),
    occlusion: clamp01(signals.occlusion ?? 0),
    communicationHealth: clamp01(signals.communicationHealth ?? 1),
    networkPressure: clamp01(1 - (signals.communicationHealth ?? 1)),
    aoiProximityRisk,
    batteryRemainingPct,
    batteryPressure,
    missionTimePressure,
    computePressure,
    riskPressure,
    footprintRedundancy,
    footprintResolutionScore,
    footprintResolutionDeficit,
    footprintUniqueAreaRate: Math.max(0, signals.footprintUniqueAreaRate ?? 0),
    agentCount: Math.max(0, signals.agentCount ?? 0)
  };
}

export function computeBehaviorWeights(signals = {}, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const mixer = profile.behaviorMixer;
  const constraints = profile.constraints;
  const normalized = normalizeSwarmSignals(signals, profile);
  const efficiencyPressure = normalized.batteryPressure + normalized.missionTimePressure + normalized.computePressure;
  const areaCoveragePressure = normalized.footprintRedundancy * 0.18 + normalized.footprintResolutionDeficit * 0.08;

  return normalizeWeights({
    [BEHAVIOR_PRIMITIVES.COVERAGE]: mixer.coverageBase + normalized.openness * mixer.coverageOpenGain + (1 - normalized.scanProgress) * mixer.coverageEarlyScanGain + areaCoveragePressure - efficiencyPressure * 0.04,
    [BEHAVIOR_PRIMITIVES.FRONTIER]: normalized.frontierPressure * mixer.frontierGain - efficiencyPressure * 0.05,
    [BEHAVIOR_PRIMITIVES.AOI]: normalized.aoiPressure * mixer.aoiGain * (1 - normalized.aoiProximityRisk * constraints.riskAoiSuppression),
    [BEHAVIOR_PRIMITIVES.RELAY]: normalized.networkPressure * mixer.relayGain,
    [BEHAVIOR_PRIMITIVES.VERIFY]: normalized.aoiPressure * normalized.scanProgress * mixer.verifyGain,
    [BEHAVIOR_PRIMITIVES.AVOIDANCE]: normalized.obstaclePressure * mixer.avoidanceGain
      + normalized.riskPressure * constraints.riskAvoidanceBoost
      + normalized.aoiProximityRisk * constraints.aoiRiskGain
  });
}

export function computeDerivedControls({
  signals = {},
  behaviorWeights = null,
  baseNeighbors = 3,
  agentCount = 1
} = {}, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const normalized = normalizeSwarmSignals({ ...signals, agentCount }, profile);
  const weights = behaviorWeights ?? computeBehaviorWeights(normalized, profile);
  const coverage = weights[BEHAVIOR_PRIMITIVES.COVERAGE] ?? 0;
  const frontier = weights[BEHAVIOR_PRIMITIVES.FRONTIER] ?? 0;
  const aoi = weights[BEHAVIOR_PRIMITIVES.AOI] ?? 0;
  const relay = weights[BEHAVIOR_PRIMITIVES.RELAY] ?? 0;
  const verify = weights[BEHAVIOR_PRIMITIVES.VERIFY] ?? 0;
  const avoidance = weights[BEHAVIOR_PRIMITIVES.AVOIDANCE] ?? 0;
  const riskPressure = normalized.riskPressure ?? 0;
  const areaSpreadPressure = normalized.footprintRedundancy * profile.constraints.footprintRedundancySpreadGain;
  const detailTightenPressure = normalized.footprintResolutionDeficit * profile.constraints.footprintResolutionTightenGain;
  const movementFreedom = clampRange(1 + coverage * 0.14 + frontier * 0.08 - relay * 0.16 - avoidance * 0.34 - riskPressure * 0.18, 0.56, 1.16);

  return {
    adaptiveNeighborTarget: adaptiveNeighborTarget({
      baseNeighbors,
      communicationHealth: normalized.communicationHealth,
      aoiCount: normalized.aoiCount,
      frontierDensity: normalized.frontierDensity,
      agentCount
    }, profile),
    movementFreedom,
    formationRadiusScale: clampRange(1 + coverage * 0.18 + aoi * 0.08 + riskPressure * 0.08 + areaSpreadPressure - detailTightenPressure - relay * 0.12 - avoidance * 0.18, 0.68, 1.34),
    formationVerticalScale: clampRange(1 + areaSpreadPressure * 0.55 - detailTightenPressure * 0.22, 0.78, 1.22),
    riskPressure,
    areaSpreadPressure,
    detailTightenPressure,
    efficiencyPressure: clamp01(normalized.batteryPressure + normalized.missionTimePressure + normalized.computePressure),
    assignmentBlendScale: {
      default: clampRange(1 + coverage * 0.08, 0.86, 1.12),
      frontier: clampRange(1 + frontier * 0.38, 0.9, 1.34),
      aoi: clampRange(1 + aoi * 0.42 + verify * 0.16, 0.92, 1.42),
      relay: clampRange(1 + relay * 0.5, 0.92, 1.48),
      verify: clampRange(1 + verify * 0.44 + aoi * 0.12, 0.9, 1.44),
      mapper: clampRange(1 + aoi * 0.18 + coverage * 0.06, 0.9, 1.24)
    },
    roleSpeedScale: {
      [DRONE_ROLES.SCOUT]: clampRange(1 + frontier * 0.16 - avoidance * 0.1, 0.84, 1.18),
      [DRONE_ROLES.MAPPER]: clampRange(1 + coverage * 0.06 + aoi * 0.08 - avoidance * 0.08, 0.86, 1.12),
      [DRONE_ROLES.RELAY]: clampRange(1 - relay * 0.12 - avoidance * 0.08, 0.78, 1.04),
      [DRONE_ROLES.VERIFIER]: clampRange(1 + verify * 0.14 + aoi * 0.06 - avoidance * 0.08, 0.84, 1.14)
    },
    sensingFocusScale: {
      [DRONE_ROLES.SCOUT]: clampRange(1 + frontier * 0.08, 0.9, 1.12),
      [DRONE_ROLES.MAPPER]: clampRange(1 + aoi * 0.22 + coverage * 0.06, 0.92, 1.26),
      [DRONE_ROLES.RELAY]: clampRange(1 - relay * 0.18, 0.72, 1.04),
      [DRONE_ROLES.VERIFIER]: clampRange(1 + verify * 0.26 + aoi * 0.14, 0.94, 1.34)
    },
    networkComplianceScale: {
      [DRONE_ROLES.SCOUT]: clampRange(1 + relay * 0.18, 0.9, 1.22),
      [DRONE_ROLES.MAPPER]: clampRange(1 + relay * 0.12, 0.92, 1.18),
      [DRONE_ROLES.RELAY]: clampRange(1 + relay * 0.34 + normalized.networkPressure * 0.18, 1, 1.42),
      [DRONE_ROLES.VERIFIER]: clampRange(1 + relay * 0.12, 0.92, 1.18)
    },
    primitivePressure: {
      coverage,
      frontier,
      aoi,
      relay,
      verify,
      avoidance
    }
  };
}

export function computeControllerState({
  signals = {},
  baseNeighbors = 3,
  agentCount = 1
} = {}, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const normalizedSignals = normalizeSwarmSignals({ ...signals, agentCount }, profile);
  const behaviorWeights = computeBehaviorWeights(normalizedSignals, profile);
  const derivedControls = computeDerivedControls({
    signals: normalizedSignals,
    behaviorWeights,
    baseNeighbors,
    agentCount
  }, profile);

  return {
    profileVersion: profile.version,
    objective: profile.objectives.default,
    normalizedSignals,
    behaviorWeights,
    derivedControls,
    dependencyGraph: PARAMETER_DEPENDENCY_GRAPH
  };
}

export function scoreSwarmRun(run, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const evaluation = profile.evaluation ?? DEFAULT_SWARM_EVALUATION_PROFILE;
  const objectiveKey = run?.behaviorProfile?.objective ?? profile.objectives.default;
  const objective = profile.objectives.profiles[objectiveKey] ?? profile.objectives.profiles[profile.objectives.default];
  const elapsedMs = Math.max(run?.elapsedMs ?? 0, 1);
  const elapsedSeconds = elapsedMs / 1000;
  const totals = run?.totals ?? {};
  const aoi = run?.aoi ?? {};
  const network = run?.network ?? {};
  const movement = run?.movement ?? {};
  const resources = run?.resources ?? {};
  const performance = run?.performance ?? {};
  const temporal = run?.temporalSummary ?? summarizeRunTemporalMetrics(run?.samples ?? []);
  const validityComplete = run?.validity?.complete ? 1 : 0;
  const aoiRequired = (aoi.selectedCount ?? 0) > 0;
  const normalizers = evaluation.normalizers;
  const rawMeasures = {
    elapsedSeconds,
    rawHits: totals.rawHits ?? 0,
    rawRays: totals.rawRays ?? 0,
    hitRate: (totals.rawHits ?? 0) / Math.max(totals.rawRays ?? 1, 1),
    pointVoxels: totals.pointVoxels ?? 0,
    footprintAreaM2: totals.footprintAreaM2 ?? 0,
    uniqueFootprintAreaM2: totals.uniqueFootprintAreaM2 ?? 0,
    redundantFootprintAreaM2: totals.redundantFootprintAreaM2 ?? 0,
    footprintRedundancyRatio: (totals.footprintAreaM2 ?? 0) > 0
      ? (totals.redundantFootprintAreaM2 ?? 0) / Math.max(totals.footprintAreaM2 ?? 1, 1e-6)
      : 0,
    avgFootprintResolutionScore: totals.avgFootprintResolutionScore ?? 0,
    pointsPerSecond: (totals.pointVoxels ?? 0) / Math.max(elapsedSeconds, 1),
    aoiRawHits: aoi.rawHits ?? 0,
    aoiFocusedHits: aoi.focusedHits ?? 0,
    aoiDwellSeconds: (aoi.dwellMs ?? 0) / 1000,
    aoiFocusedDwellSeconds: (aoi.focusedDwellMs ?? 0) / 1000,
    aoiFirstContactSeconds: aoi.firstContactMs === null || aoi.firstContactMs === undefined ? null : aoi.firstContactMs / 1000,
    networkMinHealth: network.minHealth ?? 1,
    networkAvgHealth: network.avgHealth ?? 1,
    networkMaxComponents: network.maxComponents ?? 1,
    pathLength: movement.pathLength ?? 0,
    energyProxy: movement.energyProxy ?? movement.pathLength ?? 0,
    totalEnergyWh: resources.totalEnergyWh ?? null,
    hoverWh: resources.hoverWh ?? null,
    motionWh: resources.motionWh ?? null,
    sensorWh: resources.sensorWh ?? null,
    computeWh: resources.computeWh ?? null,
    communicationWh: resources.communicationWh ?? null,
    batteryRemainingPct: resources.batteryRemainingPct ?? null,
    avgFrameMs: performance.avgFrameMs ?? 0,
    maxFrameMs: performance.maxFrameMs ?? 0,
    avgScanPassMs: performance.avgScanPassMs ?? ((totals.scanMs ?? 0) / Math.max(totals.scanPasses ?? 1, 1)),
    maxScanPassMs: performance.maxScanPassMs ?? 0,
    scanPasses: totals.scanPasses ?? 0,
    telemetrySamples: performance.samples ?? run?.samples?.length ?? 0
  };
  const temporalMeasures = {
    avgNewPointVoxelsPerSecond: temporal.avgNewPointVoxelsPerSecond ?? 0,
    avgRawHitsPerSecond: temporal.avgRawHitsPerSecond ?? 0,
    avgAoiHitsPerSecond: temporal.avgAoiHitsPerSecond ?? 0,
    avgFocusedHitsPerSecond: temporal.avgFocusedHitsPerSecond ?? 0,
    usefulAoiHitRateAfterContact: temporal.usefulAoiHitRateAfterContact ?? 0,
    avgPathMetersPerSecond: temporal.avgPathMetersPerSecond ?? 0,
    avgEnergyWhPerSecond: temporal.avgEnergyWhPerSecond ?? 0,
    avgComputeWhPerSecond: temporal.avgComputeWhPerSecond ?? 0,
    avgBehaviorWeightChangePerSecond: temporal.avgBehaviorWeightChangePerSecond ?? 0,
    avgCoverageGainPerSecond: temporal.avgCoverageGainPerSecond ?? 0,
    avgUniqueFootprintAreaPerSecond: temporal.avgUniqueFootprintAreaPerSecond ?? 0,
    avgRedundantFootprintAreaPerSecond: temporal.avgRedundantFootprintAreaPerSecond ?? 0,
    avgFootprintRedundancyRatio: temporal.avgFootprintRedundancyRatio ?? rawMeasures.footprintRedundancyRatio,
    networkFragmentedSeconds: temporal.networkFragmentedSeconds ?? 0,
    riskExposureSeconds: temporal.riskExposureSeconds ?? 0,
    avgRiskPressure: temporal.avgRiskPressure ?? 0,
    avgAoiProximityRisk: temporal.avgAoiProximityRisk ?? 0,
    aoiActiveSeconds: temporal.aoiActiveSeconds ?? 0
  };

  const aoiQuality = aoiRequired
    ? clamp01(rawMeasures.aoiRawHits / normalizers.aoiRawHits) * 0.34
      + clamp01((aoi.focusedDwellMs ?? 0) / normalizers.aoiFocusedDwellMs) * 0.25
      + clamp01(rawMeasures.aoiFocusedHits / normalizers.aoiFocusedHits) * 0.16
      + clamp01(temporalMeasures.usefulAoiHitRateAfterContact / normalizers.usefulAoiHitRateAfterContact) * 0.13
      + (rawMeasures.aoiFirstContactSeconds === null ? 0 : clamp01(1 - (aoi.firstContactMs ?? 0) / normalizers.aoiFirstContactMs) * 0.12)
    : clamp01(rawMeasures.rawHits / 1200);
  const footprintCoverage = clamp01(rawMeasures.uniqueFootprintAreaM2 / normalizers.uniqueFootprintAreaM2) * 0.58
    + clamp01(temporalMeasures.avgUniqueFootprintAreaPerSecond / normalizers.uniqueFootprintAreaPerSecond) * 0.24
    + rawMeasures.avgFootprintResolutionScore * 0.18;
  const redundancyPenalty = clamp01(temporalMeasures.avgFootprintRedundancyRatio / normalizers.footprintRedundancyRatio);
  const coverage = clamp01(
    (clamp01(rawMeasures.pointVoxels / normalizers.pointVoxels) * 0.48
    + clamp01(temporalMeasures.avgNewPointVoxelsPerSecond / normalizers.newPointVoxelsPerSecond) * 0.18
    + footprintCoverage * 0.34)
    * (1 - redundancyPenalty * 0.18)
  );
  const networkResilience = clamp01((network.minHealth ?? 1) * 0.48
    + (network.avgHealth ?? 1) * 0.28
    + (1 - clamp01((network.maxComponents ?? 1) / 6)) * 0.14
    + (1 - clamp01(temporalMeasures.networkFragmentedSeconds / normalizers.networkFragmentedSeconds)) * 0.1);
  const timeEfficiency = clamp01(1 - elapsedMs / normalizers.elapsedMs);
  const computeEfficiency = clamp01(
    (1 - rawMeasures.avgScanPassMs / normalizers.avgScanPassMs) * 0.45
    + (1 - rawMeasures.avgFrameMs / normalizers.avgFrameMs) * 0.3
    + (1 - temporalMeasures.avgComputeWhPerSecond / normalizers.computeWhPerSecond) * 0.25
  );
  const energyProxy = rawMeasures.totalEnergyWh !== null
    ? clamp01((1 - rawMeasures.totalEnergyWh / normalizers.totalEnergyWh) * 0.76 + (1 - temporalMeasures.avgEnergyWhPerSecond / normalizers.energyWhPerSecond) * 0.24)
    : clamp01(1 - rawMeasures.energyProxy / normalizers.energyProxy);
  const adaptationSmoothness = clamp01(1 - temporalMeasures.avgBehaviorWeightChangePerSecond / normalizers.behaviorWeightChangePerSecond);
  const constraintSafety = clamp01(1 - temporalMeasures.riskExposureSeconds / normalizers.riskExposureSeconds);

  const components = {
    aoiQuality,
    coverage,
    networkResilience,
    timeEfficiency,
    computeEfficiency,
    energyProxy,
    adaptationSmoothness,
    constraintSafety
  };
  const weights = objective.weights ?? {};
  const weightedScore = Object.entries(weights).reduce((sum, [key, weight]) => sum + (components[key] ?? 0) * weight, 0);
  const weightTotal = Object.values(weights).reduce((sum, value) => sum + value, 0) || 1;

  return {
    objectiveKey,
    objectiveLabel: objective.label,
    schemaVersion: 2,
    evaluationProfileVersion: evaluation.version,
    total: clamp01(weightedScore / weightTotal) * validityComplete,
    validMultiplier: validityComplete,
    confidence: clamp01((rawMeasures.scanPasses / 20) * 0.5 + (rawMeasures.telemetrySamples / 10) * 0.5) * validityComplete,
    components,
    weights,
    rawMeasures,
    temporalMeasures,
    normalizers
  };
}

function summarizeRunTemporalMetrics(samples = []) {
  const metrics = samples
    .map((sample) => sample.temporalMetrics)
    .filter(Boolean);
  const duration = metrics.reduce((sum, item) => sum + (item.deltaSeconds ?? 0), 0);
  const aoiActive = metrics.filter((item) => item.aoiActive);
  return {
    avgNewPointVoxelsPerSecond: temporalWeightedAverage(metrics, 'newPointVoxelsPerSecond', duration),
    avgRawHitsPerSecond: temporalWeightedAverage(metrics, 'rawHitsPerSecond', duration),
    avgAoiHitsPerSecond: temporalWeightedAverage(metrics, 'aoiHitsPerSecond', duration),
    avgFocusedHitsPerSecond: temporalWeightedAverage(metrics, 'focusedHitsPerSecond', duration),
    usefulAoiHitRateAfterContact: temporalWeightedAverage(aoiActive, 'aoiHitsPerSecond'),
    avgPathMetersPerSecond: temporalWeightedAverage(metrics, 'pathMetersPerSecond', duration),
    avgEnergyWhPerSecond: temporalWeightedAverage(metrics, 'energyWhPerSecond', duration),
    avgComputeWhPerSecond: temporalWeightedAverage(metrics, 'computeWhPerSecond', duration),
    avgBehaviorWeightChangePerSecond: temporalWeightedAverage(metrics, 'behaviorWeightChangePerSecond', duration),
    avgCoverageGainPerSecond: temporalWeightedAverage(metrics, 'coverageGainPerSecond', duration),
    avgUniqueFootprintAreaPerSecond: temporalWeightedAverage(metrics, 'uniqueFootprintAreaPerSecond', duration),
    avgRedundantFootprintAreaPerSecond: temporalWeightedAverage(metrics, 'redundantFootprintAreaPerSecond', duration),
    avgFootprintRedundancyRatio: temporalWeightedAverage(metrics, 'footprintRedundancyRatio', duration),
    networkFragmentedSeconds: metrics.reduce((sum, item) => sum + (item.networkFragmented ? item.deltaSeconds ?? 0 : 0), 0),
    riskExposureSeconds: metrics.reduce((sum, item) => sum + (item.riskExposure ? item.deltaSeconds ?? 0 : 0), 0),
    avgRiskPressure: temporalWeightedAverage(metrics, 'riskPressure', duration),
    avgAoiProximityRisk: temporalWeightedAverage(metrics, 'aoiProximityRisk', duration),
    aoiActiveSeconds: aoiActive.reduce((sum, item) => sum + (item.deltaSeconds ?? 0), 0)
  };
}

function temporalWeightedAverage(metrics, key, knownDuration = null) {
  const duration = knownDuration ?? metrics.reduce((sum, item) => sum + (item.deltaSeconds ?? 0), 0);
  if (duration <= 0) {
    return 0;
  }
  return metrics.reduce((sum, item) => sum + (item[key] ?? 0) * (item.deltaSeconds ?? 0), 0) / duration;
}

export function roleCountTargets({ count, terrainClass, communicationHealth, frontierDensity, aoiCount }, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const balance = profile.roleBalance;
  const relayShare = communicationHealth < balance.weakCommunicationThreshold
    ? balance.relayWeakCommsShare
    : balance.relayBaseShare;
  const verifierShare = aoiCount > 0 ? balance.verifierAoiShare : balance.verifierBaseShare;
  const scoutShare = frontierDensity > balance.frontierPressureThreshold
    ? balance.scoutFrontierShare
    : terrainClass === TERRAIN_CLASSES.SPARSE_OPEN
      ? balance.scoutOpenShare
      : balance.scoutDefaultShare;

  const counts = {
    [DRONE_ROLES.RELAY]: Math.max(1, Math.round(count * relayShare)),
    [DRONE_ROLES.VERIFIER]: Math.max(1, Math.round(count * verifierShare)),
    [DRONE_ROLES.SCOUT]: Math.max(1, Math.round(count * scoutShare)),
    [DRONE_ROLES.MAPPER]: 0
  };

  while (counts[DRONE_ROLES.RELAY] + counts[DRONE_ROLES.VERIFIER] + counts[DRONE_ROLES.SCOUT] > count) {
    const reducible = [DRONE_ROLES.SCOUT, DRONE_ROLES.VERIFIER, DRONE_ROLES.RELAY].find((role) => counts[role] > 1);
    if (!reducible) {
      break;
    }
    counts[reducible] -= 1;
  }

  counts[DRONE_ROLES.MAPPER] = Math.max(
    0,
    count - counts[DRONE_ROLES.RELAY] - counts[DRONE_ROLES.VERIFIER] - counts[DRONE_ROLES.SCOUT]
  );
  return counts;
}

export function adaptiveNeighborTarget({
  baseNeighbors,
  communicationHealth,
  aoiCount,
  frontierDensity,
  agentCount
}, profile = DEFAULT_SWARM_BEHAVIOR_PROFILE) {
  const network = profile.network;
  let target = baseNeighbors;
  if (communicationHealth < network.weakHealthThreshold) {
    target += network.weakHealthExtraNeighbors;
  }
  if (aoiCount > 0) {
    target += network.aoiExtraNeighbors;
  }
  if (frontierDensity > profile.roleBalance.frontierPressureThreshold) {
    target += network.frontierExtraNeighbors;
  }
  return Math.min(
    Math.max(Math.round(target), network.minNeighbors),
    Math.min(network.maxNeighbors, Math.max(agentCount - 1, 1))
  );
}
