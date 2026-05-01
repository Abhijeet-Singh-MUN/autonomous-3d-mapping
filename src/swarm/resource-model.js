import { DRONE_ROLES } from './constants.js';

export const DEFAULT_DRONE_RESOURCE_MODEL = {
  version: 'drone-resource-model-v0-placeholder',
  notes: 'First-pass simulation model. Values are approximate placeholders until calibrated from sourced platform, battery, sensor, and compute data.',
  platform: {
    batteryWh: 70,
    hoverPowerW: 125,
    motionJoulesPerMeter: 18,
    avionicsPowerW: 8,
    communicationBasePowerW: 1.8,
    communicationPerLinkPowerW: 0.18
  },
  sensors: {
    lidarPowerW: 6.5,
    focusRayPowerScale: 1.35,
    joulesPerRawRay: 0.0009,
    joulesPerFocusedRay: 0.0012
  },
  compute: {
    onboardIdlePowerW: 7,
    joulesPerRay: 0.0016,
    joulesPerPointVoxel: 0.00055,
    scanMsBudget: 120,
    frameMsBudget: 42
  },
  rolePowerScale: {
    [DRONE_ROLES.SCOUT]: 1.05,
    [DRONE_ROLES.MAPPER]: 1.08,
    [DRONE_ROLES.RELAY]: 0.94,
    [DRONE_ROLES.VERIFIER]: 1.12
  }
};

export function estimateSwarmResourceUse({
  agents = [],
  elapsedMs = 0,
  scanTotals = {},
  network = {},
  performance = {},
  model = DEFAULT_DRONE_RESOURCE_MODEL
} = {}) {
  const elapsedHours = Math.max(elapsedMs, 0) / 3600000;
  const agentCount = Math.max(agents.length, 1);
  const averageLinks = Math.max(network.avgLinks ?? network.linkCount ?? 0, 0);
  const rawRays = scanTotals.rawRays ?? 0;
  const focusedHits = scanTotals.focusedHits ?? 0;
  const pointVoxels = scanTotals.pointVoxels ?? 0;
  const platform = model.platform;
  const sensors = model.sensors;
  const compute = model.compute;

  const roleScaledAgentPowerW = agents.reduce((sum, agent) => {
    const roleScale = model.rolePowerScale[agent.role] ?? 1;
    return sum + (platform.hoverPowerW + platform.avionicsPowerW + compute.onboardIdlePowerW) * roleScale;
  }, 0);
  const fallbackAgentPowerW = agentCount * (platform.hoverPowerW + platform.avionicsPowerW + compute.onboardIdlePowerW);
  const hoverWh = (agents.length ? roleScaledAgentPowerW : fallbackAgentPowerW) * elapsedHours;

  const motionWh = agents.reduce((sum, agent) => {
    const roleScale = model.rolePowerScale[agent.role] ?? 1;
    return sum + ((agent.metrics?.distanceTraveled ?? 0) * platform.motionJoulesPerMeter * roleScale) / 3600;
  }, 0);

  const sensorBaseWh = agentCount * sensors.lidarPowerW * elapsedHours;
  const sensorRayWh = ((rawRays * sensors.joulesPerRawRay) + (focusedHits * sensors.joulesPerFocusedRay * sensors.focusRayPowerScale)) / 3600;
  const computeRayWh = ((rawRays * compute.joulesPerRay) + (pointVoxels * compute.joulesPerPointVoxel)) / 3600;
  const communicationWh = agentCount * (platform.communicationBasePowerW + averageLinks * platform.communicationPerLinkPowerW) * elapsedHours;
  const totalEnergyWh = hoverWh + motionWh + sensorBaseWh + sensorRayWh + computeRayWh + communicationWh;
  const totalBatteryWh = agentCount * platform.batteryWh;
  const batteryRemainingPct = totalBatteryWh > 0
    ? Math.max(0, Math.min(1, 1 - totalEnergyWh / totalBatteryWh))
    : null;

  return {
    modelVersion: model.version,
    agentCount,
    elapsedSeconds: elapsedMs / 1000,
    hoverWh,
    motionWh,
    sensorWh: sensorBaseWh + sensorRayWh,
    computeWh: computeRayWh,
    communicationWh,
    totalEnergyWh,
    batteryRemainingPct,
    totalBatteryWh,
    rawRays,
    focusedHits,
    pointVoxels,
    avgScanPassMs: performance.avgScanPassMs ?? 0,
    avgFrameMs: performance.avgFrameMs ?? 0
  };
}
