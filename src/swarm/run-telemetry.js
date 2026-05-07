import { DEFAULT_POLICY_COORDINATES, GREYBOX_POLICY_MODEL } from './behavior-profile.js';

const DB_NAME = 'autonomous-terrain-swarm-telemetry';
const STORE_NAME = 'runs';
const DB_VERSION = 1;

export class SwarmRunTelemetry {
  constructor({ sampleIntervalMs = 1000, maxSamples = 1200 } = {}) {
    this.sampleIntervalMs = sampleIntervalMs;
    this.maxSamples = maxSamples;
    this.currentRun = null;
    this.lastSampleElapsedMs = -Infinity;
    this.dbPromise = null;
  }

  startRun({ scenario, behaviorProfile, startedAtMs = performance.now() }) {
    this.currentRun = {
      id: `swarm-run-${new Date().toISOString()}-${Math.random().toString(36).slice(2, 8)}`,
      schemaVersion: 3,
      modelFamily: behaviorProfile?.modelFamily ?? GREYBOX_POLICY_MODEL.modelFamily,
      modelVersion: behaviorProfile?.modelVersion ?? GREYBOX_POLICY_MODEL.modelVersion,
      basePolicyCoordinates: { ...(behaviorProfile?.basePolicyCoordinates ?? behaviorProfile?.policyCoordinates ?? DEFAULT_POLICY_COORDINATES) },
      policyCoordinates: { ...(behaviorProfile?.policyCoordinates ?? DEFAULT_POLICY_COORDINATES) },
      runtimeNudgeProfile: behaviorProfile?.runtimeNudgeProfile ?? 'current',
      runtimeNudgeScale: behaviorProfile?.runtimeNudgeScale ?? 1,
      runtimeNudge: { ...(behaviorProfile?.runtimeNudge ?? {}) },
      effectivePolicyCoordinates: { ...(behaviorProfile?.effectivePolicyCoordinates ?? behaviorProfile?.policyCoordinates ?? DEFAULT_POLICY_COORDINATES) },
      deltaPolicyCoordinates: { ...(behaviorProfile?.deltaPolicyCoordinates ?? {}) },
      derivedProfileSummary: { ...(behaviorProfile?.derivedProfileSummary ?? {}) },
      status: 'running',
      startedAt: new Date().toISOString(),
      startedAtMs,
      endedAt: null,
      elapsedMs: 0,
      scenario,
      behaviorProfile,
      totals: {
        rawRays: 0,
        rawHits: 0,
        focusedHits: 0,
        aoiHits: 0,
        scanPasses: 0,
        scanMs: 0,
        pointVoxels: 0,
        footprintAreaM2: 0,
        uniqueFootprintAreaM2: 0,
        redundantFootprintAreaM2: 0,
        avgFootprintResolutionScore: 0,
        footprintSamples: 0
      },
      aoi: {
        selectedCount: scenario.aoiTargets?.length ?? 0,
        nearestDistance: scenario.aoiNearestDistance ?? null,
        firstContactMs: null,
        dwellMs: 0,
        focusedDwellMs: 0,
        rawHits: 0,
        focusedHits: 0
      },
      network: {
        minHealth: 1,
        avgHealth: 0,
        samples: 0,
        maxComponents: 0,
        avgLinks: 0,
        avgAdaptiveNeighborTarget: 0
      },
      movement: {
        pathLength: 0,
        energyProxy: 0,
        avgDronePathLength: 0,
        maxDronePathLength: 0,
        pathImbalance: 0
      },
      resources: {
        modelVersion: null,
        hoverWh: 0,
        motionWh: 0,
        sensorWh: 0,
        computeWh: 0,
        communicationWh: 0,
        totalEnergyWh: 0,
        batteryRemainingPct: null
      },
      performance: {
        avgFrameMs: 0,
        maxFrameMs: 0,
        avgScanPassMs: 0,
        maxScanPassMs: 0,
        samples: 0
      },
      temporalSummary: {
        samples: 0,
        avgNewPointVoxelsPerSecond: 0,
        avgRawHitsPerSecond: 0,
        avgAoiHitsPerSecond: 0,
        avgFocusedHitsPerSecond: 0,
        avgPathMetersPerSecond: 0,
        avgEnergyWhPerSecond: 0,
        avgComputeWhPerSecond: 0,
        avgBehaviorWeightChangePerSecond: 0,
        avgCoverageGainPerSecond: 0,
        avgUniqueFootprintAreaPerSecond: 0,
        avgRedundantFootprintAreaPerSecond: 0,
        avgFootprintRedundancyRatio: 0,
        networkFragmentedSeconds: 0,
        riskExposureSeconds: 0,
        aoiActiveSeconds: 0,
        usefulAoiHitRateAfterContact: 0,
        avgAbsDeltaPsi: 0,
        maxAbsDeltaPsi: 0,
        avgRoleEntropy: 0,
        minRoleEntropy: 0,
        maxRoleEntropy: 0,
        avgRoleEntropyNorm: 0
      },
      validity: {
        complete: false,
        flags: []
      },
      notes: [],
      samples: []
    };
    this.lastSampleElapsedMs = -Infinity;
    return this.currentRun;
  }

  get active() {
    return Boolean(this.currentRun);
  }

  recordScan({
    rayCount,
    hitCount,
    focusedHitCount,
    aoiHitCount,
    scanPassMs,
    pointVoxels,
    footprintAreaM2 = 0,
    uniqueFootprintAreaM2 = 0,
    redundantFootprintAreaM2 = 0,
    avgFootprintResolutionScore = 0
  }) {
    if (!this.currentRun) {
      return;
    }
    const totals = this.currentRun.totals;
    totals.rawRays += rayCount;
    totals.rawHits += hitCount;
    totals.focusedHits += focusedHitCount;
    totals.aoiHits += aoiHitCount;
    totals.scanPasses += 1;
    totals.scanMs += scanPassMs;
    totals.pointVoxels = pointVoxels;
    totals.footprintAreaM2 += footprintAreaM2;
    totals.uniqueFootprintAreaM2 += uniqueFootprintAreaM2;
    totals.redundantFootprintAreaM2 += redundantFootprintAreaM2;
    totals.avgFootprintResolutionScore = runningAverage(
      totals.avgFootprintResolutionScore,
      avgFootprintResolutionScore,
      totals.footprintSamples
    );
    totals.footprintSamples += 1;

    this.currentRun.aoi.rawHits += aoiHitCount;
    this.currentRun.aoi.focusedHits += focusedHitCount;
  }

  addNote(note) {
    if (!this.currentRun || !note) {
      return;
    }
    this.currentRun.notes.push(note);
  }

  recordSample(sample) {
    if (!this.currentRun) {
      return;
    }
    const elapsedMs = sample.elapsedMs ?? 0;
    this.currentRun.elapsedMs = elapsedMs;
    if (elapsedMs - this.lastSampleElapsedMs < this.sampleIntervalMs) {
      return;
    }
    this.lastSampleElapsedMs = elapsedMs;

    const totalsSnapshot = { ...this.currentRun.totals };
    const previousSample = this.currentRun.samples.at(-1) ?? null;
    const enrichedSample = {
      ...sample,
      scanTotals: totalsSnapshot
    };
    enrichedSample.temporalMetrics = computeTemporalMetrics(previousSample, enrichedSample);

    if (this.currentRun.samples.length >= this.maxSamples) {
      this.currentRun.samples.shift();
    }
    this.currentRun.samples.push(enrichedSample);

    const network = this.currentRun.network;
    const health = sample.communicationHealth ?? 1;
    network.minHealth = Math.min(network.minHealth, health);
    network.avgHealth = runningAverage(network.avgHealth, health, network.samples);
    network.avgLinks = runningAverage(network.avgLinks, sample.linkCount ?? 0, network.samples);
    network.avgAdaptiveNeighborTarget = runningAverage(
      network.avgAdaptiveNeighborTarget,
      sample.adaptiveNeighborTarget ?? 0,
      network.samples
    );
    network.maxComponents = Math.max(network.maxComponents, sample.connectedComponents ?? 0);
    network.samples += 1;

    if (sample.aoiInFocus) {
      if (this.currentRun.aoi.firstContactMs === null) {
        this.currentRun.aoi.firstContactMs = elapsedMs;
      }
      this.currentRun.aoi.dwellMs += this.sampleIntervalMs;
    }
    if (sample.aoiFocusedAgents > 0) {
      this.currentRun.aoi.focusedDwellMs += this.sampleIntervalMs;
    }
    this.currentRun.movement.pathLength = sample.totalPathLength ?? this.currentRun.movement.pathLength;
    this.currentRun.movement.energyProxy = sample.energyProxy ?? this.currentRun.movement.energyProxy;
    if (sample.droneTrajectories?.length) {
      const trajectorySummary = summarizeTrajectoryMetrics([sample]);
      this.currentRun.movement.avgDronePathLength = trajectorySummary.avgDronePathLength;
      this.currentRun.movement.maxDronePathLength = trajectorySummary.maxDronePathLength;
      this.currentRun.movement.pathImbalance = trajectorySummary.pathImbalance;
      this.currentRun.trajectorySummary = trajectorySummary;
    }
    if (sample.resources) {
      this.currentRun.resources = {
        ...this.currentRun.resources,
        ...sample.resources
      };
    }

    const performance = this.currentRun.performance;
    const frameMs = sample.frameAvgMs ?? 0;
    const scanMs = sample.scanPassMs ?? 0;
    performance.avgFrameMs = runningAverage(performance.avgFrameMs, frameMs, performance.samples);
    performance.avgScanPassMs = runningAverage(performance.avgScanPassMs, scanMs, performance.samples);
    performance.maxFrameMs = Math.max(performance.maxFrameMs, frameMs);
    performance.maxScanPassMs = Math.max(performance.maxScanPassMs, scanMs);
    performance.samples += 1;
    this.currentRun.temporalSummary = summarizeTemporalMetrics(this.currentRun.samples);
    this.currentRun.nudgeSummary = summarizeNudgeMetrics(this.currentRun.samples);
  }

  async finishRun({ status = 'complete', endedAtMs = performance.now(), validity = null, notes = [], scoring = null } = {}) {
    if (!this.currentRun) {
      return null;
    }
    const run = this.currentRun;
    run.status = status;
    run.endedAt = new Date().toISOString();
    run.elapsedMs = Math.max(run.elapsedMs, endedAtMs - run.startedAtMs);
    run.validity = validity ?? run.validity;
    run.temporalSummary = summarizeTemporalMetrics(run.samples);
    run.nudgeSummary = summarizeNudgeMetrics(run.samples);
    run.trajectorySummary = summarizeTrajectoryMetrics(run.samples);
    run.scoring = scoring ?? run.scoring ?? null;
    run.notes.push(...notes);
    this.currentRun = null;
    await this.saveRun(run);
    return run;
  }

  async saveRun(run) {
    try {
      const db = await this.openDb();
      await new Promise((resolve, reject) => {
        const tx = db.transaction(STORE_NAME, 'readwrite');
        tx.objectStore(STORE_NAME).put(run);
        tx.oncomplete = () => resolve();
        tx.onerror = () => reject(tx.error);
      });
    } catch (error) {
      console.warn('Unable to persist swarm telemetry run.', error);
    }
  }

  async listRuns({ limit = 50, modelFamily = null } = {}) {
    try {
      const db = await this.openDb();
      const runs = await new Promise((resolve, reject) => {
        const tx = db.transaction(STORE_NAME, 'readonly');
        const request = tx.objectStore(STORE_NAME).getAll();
        request.onsuccess = () => resolve(request.result ?? []);
        request.onerror = () => reject(request.error);
      });
      return runs
        .filter((run) => !modelFamily || telemetryRunModelFamily(run) === modelFamily)
        .sort((a, b) => String(b.startedAt).localeCompare(String(a.startedAt)))
        .slice(0, limit);
    } catch (error) {
      console.warn('Unable to read swarm telemetry runs.', error);
      return [];
    }
  }

  async exportRuns({ modelFamily = null } = {}) {
    return this.listRuns({ limit: Number.MAX_SAFE_INTEGER, modelFamily });
  }

  async deleteRunsByWorkspace({ workspace, modelFamily = null } = {}) {
    if (!workspace) {
      return 0;
    }
    try {
      const db = await this.openDb();
      const runs = await this.listRuns({ limit: Number.MAX_SAFE_INTEGER, modelFamily });
      const ids = runs
        .filter((run) => workspace === 'legacy'
          ? !run.scenario?.datasetWorkspace
          : run.scenario?.datasetWorkspace === workspace)
        .map((run) => run.id);
      if (!ids.length) {
        return 0;
      }
      await new Promise((resolve, reject) => {
        const tx = db.transaction(STORE_NAME, 'readwrite');
        const store = tx.objectStore(STORE_NAME);
        ids.forEach((id) => store.delete(id));
        tx.oncomplete = () => resolve();
        tx.onerror = () => reject(tx.error);
      });
      return ids.length;
    } catch (error) {
      console.warn('Unable to delete workspace telemetry runs.', error);
      return 0;
    }
  }

  openDb() {
    if (!this.dbPromise) {
      this.dbPromise = new Promise((resolve, reject) => {
        const request = indexedDB.open(DB_NAME, DB_VERSION);
        request.onupgradeneeded = () => {
          const db = request.result;
          if (!db.objectStoreNames.contains(STORE_NAME)) {
            const store = db.createObjectStore(STORE_NAME, { keyPath: 'id' });
            store.createIndex('startedAt', 'startedAt');
            store.createIndex('status', 'status');
          }
        };
        request.onsuccess = () => resolve(request.result);
        request.onerror = () => reject(request.error);
      });
    }
    return this.dbPromise;
  }
}

function telemetryRunModelFamily(run) {
  return run?.modelFamily ?? run?.behaviorProfile?.modelFamily ?? null;
}

function runningAverage(previousAverage, nextValue, previousCount) {
  return (previousAverage * previousCount + nextValue) / Math.max(previousCount + 1, 1);
}

function computeTemporalMetrics(previousSample, sample) {
  const deltaSeconds = Math.max(((sample.elapsedMs ?? 0) - (previousSample?.elapsedMs ?? 0)) / 1000, 1e-3);
  const previousTotals = previousSample?.scanTotals ?? {};
  const totals = sample.scanTotals ?? {};
  const previousResources = previousSample?.resources ?? {};
  const resources = sample.resources ?? {};
  const previousWeights = previousSample?.behaviorWeights ?? {};
  const weights = sample.behaviorWeights ?? {};
  const behaviorWeightChange = unionKeys(previousWeights, weights).reduce((sum, key) => (
    sum + Math.abs((weights[key] ?? 0) - (previousWeights[key] ?? 0))
  ), 0);
  const roleEntropy = roleDistributionEntropy(sample.roleCounts);

  return {
    deltaSeconds,
    newPointVoxelsPerSecond: rate((sample.pointVoxels ?? 0) - (previousSample?.pointVoxels ?? 0), deltaSeconds),
    rawHitsPerSecond: rate((totals.rawHits ?? 0) - (previousTotals.rawHits ?? 0), deltaSeconds),
    rawRaysPerSecond: rate((totals.rawRays ?? 0) - (previousTotals.rawRays ?? 0), deltaSeconds),
    aoiHitsPerSecond: rate((totals.aoiHits ?? 0) - (previousTotals.aoiHits ?? 0), deltaSeconds),
    focusedHitsPerSecond: rate((totals.focusedHits ?? 0) - (previousTotals.focusedHits ?? 0), deltaSeconds),
    uniqueFootprintAreaPerSecond: rate((totals.uniqueFootprintAreaM2 ?? 0) - (previousTotals.uniqueFootprintAreaM2 ?? 0), deltaSeconds),
    redundantFootprintAreaPerSecond: rate((totals.redundantFootprintAreaM2 ?? 0) - (previousTotals.redundantFootprintAreaM2 ?? 0), deltaSeconds),
    footprintRedundancyRatio: (totals.footprintAreaM2 ?? 0) > 0
      ? (totals.redundantFootprintAreaM2 ?? 0) / Math.max(totals.footprintAreaM2 ?? 1, 1e-6)
      : 0,
    pathMetersPerSecond: rate((sample.totalPathLength ?? 0) - (previousSample?.totalPathLength ?? 0), deltaSeconds),
    energyWhPerSecond: rate((resources.totalEnergyWh ?? 0) - (previousResources.totalEnergyWh ?? 0), deltaSeconds),
    computeWhPerSecond: rate((resources.computeWh ?? 0) - (previousResources.computeWh ?? 0), deltaSeconds),
    coverageGainPerSecond: rate((sample.coverage ?? 0) - (previousSample?.coverage ?? 0), deltaSeconds),
    behaviorWeightChangePerSecond: behaviorWeightChange / deltaSeconds,
    networkFragmented: (sample.connectedComponents ?? 0) > 1 ? 1 : 0,
    riskExposure: (sample.normalizedSignals?.riskPressure ?? 0) > 0.24 ? 1 : 0,
    riskPressure: sample.normalizedSignals?.riskPressure ?? 0,
    aoiProximityRisk: sample.normalizedSignals?.aoiProximityRisk ?? 0,
    aoiActive: sample.aoiInFocus || (sample.aoiFocusedAgents ?? 0) > 0 ? 1 : 0,
    roleEntropy: roleEntropy.entropy,
    roleEntropyNorm: roleEntropy.normalized
  };
}

function summarizeTrajectoryMetrics(samples = []) {
  const latest = [...samples].reverse().find((sample) => sample.droneTrajectories?.length) ?? null;
  const trajectories = latest?.droneTrajectories ?? [];
  if (!trajectories.length) {
    return {
      droneCount: 0,
      avgDronePathLength: 0,
      maxDronePathLength: 0,
      pathImbalance: 0
    };
  }
  const paths = trajectories.map((item) => item.pathLengthM ?? 0);
  const avg = paths.reduce((sum, value) => sum + value, 0) / paths.length;
  const max = paths.reduce((largest, value) => Math.max(largest, value), 0);
  return {
    droneCount: trajectories.length,
    avgDronePathLength: avg,
    maxDronePathLength: max,
    pathImbalance: max / Math.max(avg, 1e-6)
  };
}

function summarizeTemporalMetrics(samples = []) {
  const metrics = samples
    .map((sample) => sample.temporalMetrics)
    .filter(Boolean);
  const seconds = metrics.reduce((sum, item) => sum + (item.deltaSeconds ?? 0), 0);
  const samplesWithAoi = metrics.filter((item) => item.aoiActive);

  return {
    samples: metrics.length,
    avgNewPointVoxelsPerSecond: weightedAverage(metrics, 'newPointVoxelsPerSecond'),
    avgRawHitsPerSecond: weightedAverage(metrics, 'rawHitsPerSecond'),
    avgAoiHitsPerSecond: weightedAverage(metrics, 'aoiHitsPerSecond'),
    avgFocusedHitsPerSecond: weightedAverage(metrics, 'focusedHitsPerSecond'),
    avgPathMetersPerSecond: weightedAverage(metrics, 'pathMetersPerSecond'),
    avgEnergyWhPerSecond: weightedAverage(metrics, 'energyWhPerSecond'),
    avgComputeWhPerSecond: weightedAverage(metrics, 'computeWhPerSecond'),
    avgBehaviorWeightChangePerSecond: weightedAverage(metrics, 'behaviorWeightChangePerSecond'),
    avgCoverageGainPerSecond: weightedAverage(metrics, 'coverageGainPerSecond'),
    avgUniqueFootprintAreaPerSecond: weightedAverage(metrics, 'uniqueFootprintAreaPerSecond'),
    avgRedundantFootprintAreaPerSecond: weightedAverage(metrics, 'redundantFootprintAreaPerSecond'),
    avgFootprintRedundancyRatio: weightedAverage(metrics, 'footprintRedundancyRatio'),
    networkFragmentedSeconds: metrics.reduce((sum, item) => sum + (item.networkFragmented ? item.deltaSeconds ?? 0 : 0), 0),
    riskExposureSeconds: metrics.reduce((sum, item) => sum + (item.riskExposure ? item.deltaSeconds ?? 0 : 0), 0),
    avgRiskPressure: weightedAverage(metrics, 'riskPressure'),
    avgAoiProximityRisk: weightedAverage(metrics, 'aoiProximityRisk'),
    aoiActiveSeconds: samplesWithAoi.reduce((sum, item) => sum + (item.deltaSeconds ?? 0), 0),
    usefulAoiHitRateAfterContact: weightedAverage(samplesWithAoi, 'aoiHitsPerSecond'),
    avgAbsDeltaPsi: summarizeNudgeMetrics(samples).avgAbsDeltaPsi,
    maxAbsDeltaPsi: summarizeNudgeMetrics(samples).maxAbsDeltaPsi,
    avgRoleEntropy: weightedAverage(metrics, 'roleEntropy'),
    minRoleEntropy: minMetric(metrics, 'roleEntropy'),
    maxRoleEntropy: maxMetric(metrics, 'roleEntropy'),
    avgRoleEntropyNorm: weightedAverage(metrics, 'roleEntropyNorm'),
    durationSeconds: seconds
  };
}

function roleDistributionEntropy(roleCounts = {}) {
  const counts = Object.values(roleCounts ?? {})
    .map((value) => Number(value))
    .filter((value) => Number.isFinite(value) && value > 0);
  const total = counts.reduce((sum, value) => sum + value, 0);
  if (total <= 0) {
    return { entropy: 0, normalized: 0 };
  }
  const entropy = counts.reduce((sum, count) => {
    const p = count / total;
    return sum - p * Math.log(p);
  }, 0);
  return {
    entropy,
    normalized: Math.min(Math.max(entropy / Math.log(4), 0), 1)
  };
}

function minMetric(metrics, key) {
  const values = metrics.map((item) => item[key]).filter((value) => Number.isFinite(value));
  return values.length ? Math.min(...values) : 0;
}

function maxMetric(metrics, key) {
  const values = metrics.map((item) => item[key]).filter((value) => Number.isFinite(value));
  return values.length ? Math.max(...values) : 0;
}

function summarizeNudgeMetrics(samples = []) {
  const deltas = samples
    .map((sample) => sample.deltaPolicyCoordinates)
    .filter(Boolean);
  if (!deltas.length) {
    return {
      avgAbsDeltaPsi: 0,
      maxAbsDeltaPsi: 0
    };
  }
  const absValues = deltas.flatMap((delta) => Object.values(delta).map((value) => Math.abs(Number.isFinite(value) ? value : 0)));
  return {
    avgAbsDeltaPsi: absValues.reduce((sum, value) => sum + value, 0) / Math.max(absValues.length, 1),
    maxAbsDeltaPsi: absValues.reduce((max, value) => Math.max(max, value), 0)
  };
}

function weightedAverage(items, key) {
  const duration = items.reduce((sum, item) => sum + (item.deltaSeconds ?? 0), 0);
  if (duration <= 0) {
    return 0;
  }
  return items.reduce((sum, item) => sum + (item[key] ?? 0) * (item.deltaSeconds ?? 0), 0) / duration;
}

function rate(delta, seconds) {
  return delta / Math.max(seconds, 1e-3);
}

function unionKeys(left, right) {
  return Array.from(new Set([...Object.keys(left ?? {}), ...Object.keys(right ?? {})]));
}
