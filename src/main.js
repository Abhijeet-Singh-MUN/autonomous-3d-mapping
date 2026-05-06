import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree } from 'three-mesh-bvh';
import './styles.css';
import { CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN, SENSOR_PRESETS } from './core/constants.js';
import { VoxelGrid } from './core/voxel-grid.js';
import { searchPath3D as runVoxelPlanner } from './core/planner.js';
import { almostEqual, clamp, clampInt, createRng, generateSeed, lerp, readNumber } from './core/math.js';
import {
  DEFAULT_DRONE_RESOURCE_MODEL,
  DEFAULT_POLICY_COORDINATES,
  DEFAULT_SWARM_BEHAVIOR_PROFILE,
  DEFAULT_SWARM_EVALUATION_PROFILE,
  DRONE_ROLES,
  FORMATION_MODES,
  GREYBOX_POLICY_MODEL,
  estimateSwarmResourceUse,
  normalizePolicyCoordinates,
  runtimeNudgeProfileConfig,
  scoreSwarmRun,
  SwarmController,
  SwarmRunTelemetry
} from './swarm/index.js';

THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;

const els = {
  viewport: document.querySelector('#sceneViewport'),
  cloudViewport: document.querySelector('#cloudViewport'),
  cloudOverlay: document.querySelector('#cloudOverlay'),
  cloudHeader: document.querySelector('#cloudHeader'),
  orbitCanvas: document.querySelector('#orbitCanvas'),
  logWindow: document.querySelector('#logWindow'),
  startBtn: document.querySelector('#startBtn'),
  pauseBtn: document.querySelector('#pauseBtn'),
  continueBtn: document.querySelector('#continueBtn'),
  stopBtn: document.querySelector('#stopBtn'),
  resetBtn: document.querySelector('#resetBtn'),
  randomizeBtn: document.querySelector('#randomizeBtn'),
  exportBtn: document.querySelector('#exportBtn'),
  telemetryBtn: document.querySelector('#telemetryBtn'),
  algorithmBtn: document.querySelector('#algorithmBtn'),
  policyExperimentBtn: document.querySelector('#policyExperimentBtn'),
  telemetryPanel: document.querySelector('#telemetryPanel'),
  telemetryRefreshBtn: document.querySelector('#telemetryRefreshBtn'),
  telemetryExportBtn: document.querySelector('#telemetryExportBtn'),
  telemetryExportCsvBtn: document.querySelector('#telemetryExportCsvBtn'),
  telemetrySort: document.querySelector('#telemetrySort'),
  telemetryScenarioFilter: document.querySelector('#telemetryScenarioFilter'),
  telemetryWorkspaceFilter: document.querySelector('#telemetryWorkspaceFilter'),
  telemetryValidOnly: document.querySelector('#telemetryValidOnly'),
  telemetryRunCount: document.querySelector('#telemetryRunCount'),
  telemetryValidCount: document.querySelector('#telemetryValidCount'),
  telemetryLatestHits: document.querySelector('#telemetryLatestHits'),
  telemetryRuns: document.querySelector('#telemetryRuns'),
  workspaceClearTarget: document.querySelector('#workspaceClearTarget'),
  workspaceClearConfirm: document.querySelector('#workspaceClearConfirm'),
  workspaceRemoveName: document.querySelector('#workspaceRemoveName'),
  workspaceClearBtn: document.querySelector('#workspaceClearBtn'),
  algorithmPanel: document.querySelector('#algorithmPanel'),
  algorithmWeights: document.querySelector('#algorithmWeights'),
  algorithmSignals: document.querySelector('#algorithmSignals'),
  algorithmControls: document.querySelector('#algorithmControls'),
  algorithmRoles: document.querySelector('#algorithmRoles'),
  algorithmObjective: document.querySelector('#algorithmObjective'),
  missionMode: document.querySelector('#missionMode'),
  environmentMode: document.querySelector('#environmentMode'),
  aoiPreset: document.querySelector('#aoiPreset'),
  swarmSize: document.querySelector('#swarmSize'),
  swarmFormation: document.querySelector('#swarmFormation'),
  communicationRange: document.querySelector('#communicationRange'),
  communicationNeighbors: document.querySelector('#communicationNeighbors'),
  communicationDropout: document.querySelector('#communicationDropout'),
  swarmScanDensity: document.querySelector('#swarmScanDensity'),
  performanceBudget: document.querySelector('#performanceBudget'),
  datasetWorkspace: document.querySelector('#datasetWorkspace'),
  datasetWorkspaceSelect: document.querySelector('#datasetWorkspaceSelect'),
  datasetWorkspaceSaveBtn: document.querySelector('#datasetWorkspaceSaveBtn'),
  objectiveProfile: document.querySelector('#objectiveProfile'),
  policyCoverageArea: document.querySelector('#policyCoverageArea'),
  policyAoiDetail: document.querySelector('#policyAoiDetail'),
  policyRiskSafety: document.querySelector('#policyRiskSafety'),
  policyResourceEfficiency: document.querySelector('#policyResourceEfficiency'),
  runtimeNudgeProfile: document.querySelector('#runtimeNudgeProfile'),
  policyExperimentMode: document.querySelector('#policyExperimentMode'),
  policyExperimentSeconds: document.querySelector('#policyExperimentSeconds'),
  policyCoverageAreaValue: document.querySelector('#policyCoverageAreaValue'),
  policyAoiDetailValue: document.querySelector('#policyAoiDetailValue'),
  policyRiskSafetyValue: document.querySelector('#policyRiskSafetyValue'),
  policyResourceEfficiencyValue: document.querySelector('#policyResourceEfficiencyValue'),
  visiblePointBudget: document.querySelector('#visiblePointBudget'),
  renderScaleBudget: document.querySelector('#renderScaleBudget'),
  terrainWidth: document.querySelector('#terrainWidth'),
  terrainDepth: document.querySelector('#terrainDepth'),
  terrainHeight: document.querySelector('#terrainHeight'),
  terrainRidgeHeight: document.querySelector('#terrainRidgeHeight'),
  terrainValleyWidth: document.querySelector('#terrainValleyWidth'),
  villageCount: document.querySelector('#villageCount'),
  treeCount: document.querySelector('#treeCount'),
  terrainVoxelResolution: document.querySelector('#terrainVoxelResolution'),
  scanMode: document.querySelector('#scanMode'),
  sensorPreset: document.querySelector('#sensorPreset'),
  plannerMode: document.querySelector('#plannerMode'),
  pathStyle: document.querySelector('#pathStyle'),
  safetyRadius: document.querySelector('#safetyRadius'),
  terrainFlightClearance: document.querySelector('#terrainFlightClearance'),
  moveSpeed: document.querySelector('#moveSpeed'),
  horizontalRays: document.querySelector('#horizontalRays'),
  verticalRays: document.querySelector('#verticalRays'),
  horizontalFov: document.querySelector('#horizontalFov'),
  verticalFov: document.querySelector('#verticalFov'),
  maxRange: document.querySelector('#maxRange'),
  voxelSize: document.querySelector('#voxelSize'),
  dwellMs: document.querySelector('#dwellMs'),
  modeReadout: document.querySelector('#modeReadout'),
  stateReadout: document.querySelector('#stateReadout'),
  pointsReadout: document.querySelector('#pointsReadout'),
  scansReadout: document.querySelector('#scansReadout'),
  coverageReadout: document.querySelector('#coverageReadout'),
  phaseReadout: document.querySelector('#phaseReadout'),
  plannerReadout: document.querySelector('#plannerReadout'),
  goalReadout: document.querySelector('#goalReadout'),
  seedReadout: document.querySelector('#seedReadout'),
  swarmReadout: document.querySelector('#swarmReadout'),
  formationReadout: document.querySelector('#formationReadout'),
  commsReadout: document.querySelector('#commsReadout'),
  budgetReadout: document.querySelector('#budgetReadout'),
  perfReadout: document.querySelector('#perfReadout'),
  scanPerfReadout: document.querySelector('#scanPerfReadout'),
  cloudMeta: document.querySelector('#cloudMeta')
};

const orbitCtx = els.orbitCanvas.getContext('2d');
const raycaster = new THREE.Raycaster();
raycaster.firstHitOnly = true;

const sim = {
  status: 'idle',
  droneActive: false,
  complete: false,
  missionStarted: false,
  environmentSeed: 0,
  room: null,
  terrainConfig: null,
  scanTargets: [],
  floorFootprints: [],
  ceilingHazard: null,
  aoiTargets: [],
  terrainProfile: null,
  mapper: null,
  scanCounter: 0,
  pointCount: 0,
  lastTime: 0,
  currentPhase: 'idle',
  bootstrapPlan: [],
  bootstrapIndex: 0,
  routeSamples: [],
  routeSampleIndex: 0,
  currentGoal: null,
  currentGoalType: 'frontier',
  currentGoalWorld: null,
  currentBandIndex: 0,
  bandVisitCounts: [],
  holdElapsedMs: 0,
  holdScanElapsedMs: 0,
  moveScanElapsedMs: 0,
  blockedCooldownMs: 0,
  plannerStats: {
    goalsReached: 0,
    replans: 0,
    lastPathLength: 0
  },
  cloudGeometry: new THREE.BufferGeometry(),
  cloudMaterial: new THREE.PointsMaterial({
    size: 0.045,
    sizeAttenuation: true,
    vertexColors: true
  }),
  overlayCloud: null,
  swarmController: null,
  swarmTargets: [],
  swarmSnapshot: null,
  swarmTelemetry: new SwarmRunTelemetry(),
  droneResourceModel: DEFAULT_DRONE_RESOURCE_MODEL,
  swarmTelemetryElapsedMs: 0,
  swarmLaunchElapsedMs: 0,
  swarmScanElapsedMs: 0,
  commsRenderElapsedMs: 0,
  cloudFlushElapsedMs: 0,
  graphRenderElapsedMs: 0,
  cloudDirty: false,
  voxelMap: new Map(),
  footprintCoverageMap: new Map(),
  footprintMetrics: {
    redundancyRatio: 0,
    resolutionScore: 0,
    uniqueAreaRate: 0
  },
  performance: {
    frameAvgMs: 16,
    loadScale: 1,
    renderScaleElapsedMs: 0,
    currentRenderScale: 1,
    displayedPointCount: 0,
    scanPassMs: 0,
    lastScanRayCount: 0,
    lastScanHits: 0,
    lastCloudFlushMs: 0,
    cloudBufferCapacity: 0
  },
  scanDirectionCache: {
    key: '',
    directions: []
  },
  pointCloudBuffer: {
    positions: new Float32Array(0),
    colors: new Float32Array(0),
    capacity: 0
  },
  navigation: {
    activeViewport: 'main',
    keys: new Set(),
    velocity: {
      main: new THREE.Vector3(),
      cloud: new THREE.Vector3()
    },
    freeLook: {
      active: false,
      viewport: 'main',
      lastX: 0,
      lastY: 0
    }
  },
  policyExperiment: {
    active: false,
    runMs: 120000,
    currentIndex: 0,
    runPlan: [],
    waitingForNext: false,
    currentPreset: null,
    batchId: null
  },
  logs: []
};

const temp = {
  source: new THREE.Vector3(),
  dir: new THREE.Vector3(),
  right: new THREE.Vector3(),
  up: new THREE.Vector3(),
  travel: new THREE.Vector3(),
  heading: new THREE.Vector3(),
  worldPoint: new THREE.Vector3(),
  probeOrigin: new THREE.Vector3(),
  cameraForward: new THREE.Vector3(),
  cameraRight: new THREE.Vector3()
};

const POLICY_EXPERIMENT_PRESETS = [
  {
    id: 'balanced-default',
    label: 'Balanced default',
    policyCoordinates: { coverage_area: 0.35, aoi_detail: 0.3, risk_safety: 0.2, resource_efficiency: 0.15 }
  },
  {
    id: 'coverage-heavy',
    label: 'Coverage heavy',
    policyCoordinates: { coverage_area: 0.55, aoi_detail: 0.2, risk_safety: 0.15, resource_efficiency: 0.1 }
  },
  {
    id: 'aoi-detail-heavy',
    label: 'AOI detail heavy',
    policyCoordinates: { coverage_area: 0.18, aoi_detail: 0.56, risk_safety: 0.16, resource_efficiency: 0.1 }
  },
  {
    id: 'safe-efficient',
    label: 'Safe efficient',
    policyCoordinates: { coverage_area: 0.18, aoi_detail: 0.18, risk_safety: 0.34, resource_efficiency: 0.3 }
  }
];

const POLICY_EXPERIMENT_NUDGE_PROFILES = ['very_low', 'low', 'current', 'strong', 'very_strong'];
const DATASET_WORKSPACES_KEY = 'autonomous-terrain-swarm-workspaces';
const DEFAULT_DATASET_WORKSPACE = 'greybox-calibration-v1';

const main = createMainScene();
const cloud = createCloudScene();

setupUI();
createPointCloud();
applySensorPreset(false);
applySwarmScanDensityPreset();
syncPolicyCoordinateOutputs();
regenerateEnvironment(true);
resetMission(false, true);
logMessage('Terrain swarm sandbox ready. Drones map the scene from LiDAR observations and maintain reliable neighbor links.');
animate(0);

function createMainScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#08111d');
  scene.fog = new THREE.Fog('#08111d', 14, 38);

  const camera = new THREE.PerspectiveCamera(52, 1, 0.1, 900);
  camera.position.set(8.5, 6.4, 9.4);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  els.viewport.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.enableRotate = false;
  controls.enablePan = true;
  controls.enableZoom = true;
  controls.screenSpacePanning = true;
  controls.keyPanSpeed = 18;
  controls.mouseButtons = {
    LEFT: THREE.MOUSE.ROTATE,
    MIDDLE: THREE.MOUSE.DOLLY,
    RIGHT: THREE.MOUSE.PAN
  };
  controls.target.set(0, 1.4, 0);

  scene.add(new THREE.HemisphereLight('#dbe7ff', '#202d45', 1.55));

  const key = new THREE.DirectionalLight('#ffffff', 1.05);
  key.position.set(6, 9, 7);
  scene.add(key);

  const fill = new THREE.DirectionalLight('#8dc0ff', 0.5);
  fill.position.set(-5, 4, -6);
  scene.add(fill);

  const roomGroup = new THREE.Group();
  const pathGroup = new THREE.Group();
  const beamGroup = new THREE.Group();
  const swarmGroup = new THREE.Group();
  const commGroup = new THREE.Group();
  scene.add(roomGroup, pathGroup, beamGroup, commGroup, swarmGroup);

  const drone = createDroneMesh();
  scene.add(drone);

  const resize = () => {
    const width = els.viewport.clientWidth;
    const height = els.viewport.clientHeight;
    camera.aspect = width / Math.max(height, 1);
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };

  new ResizeObserver(resize).observe(els.viewport);
  els.viewport.addEventListener('pointerenter', () => {
    sim.navigation.activeViewport = 'main';
  });
  setupFreeLookControls(renderer.domElement, 'main');
  renderer.domElement.tabIndex = 0;
  window.addEventListener('resize', resize);
  resize();

  return { scene, camera, renderer, controls, roomGroup, pathGroup, beamGroup, swarmGroup, commGroup, drone };
}

function createCloudScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#060a12');

  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 900);
  camera.position.set(6, 5, 6);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  els.cloudViewport.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.enableRotate = false;
  controls.enablePan = true;
  controls.enableZoom = true;
  controls.screenSpacePanning = true;
  controls.keyPanSpeed = 18;
  controls.mouseButtons = {
    LEFT: THREE.MOUSE.ROTATE,
    MIDDLE: THREE.MOUSE.DOLLY,
    RIGHT: THREE.MOUSE.PAN
  };
  controls.target.set(0, 1.2, 0);

  scene.add(new THREE.HemisphereLight('#f5fbff', '#223049', 1.15));

  const fill = new THREE.DirectionalLight('#91bfff', 0.78);
  fill.position.set(6, 8, 5);
  scene.add(fill);

  const grid = new THREE.GridHelper(16, 32, '#5176af', '#1e2f49');
  scene.add(grid);
  scene.add(new THREE.AxesHelper(1.8));

  const resize = () => {
    const width = els.cloudViewport.clientWidth;
    const height = els.cloudViewport.clientHeight;
    camera.aspect = width / Math.max(height, 1);
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };

  new ResizeObserver(resize).observe(els.cloudOverlay);
  els.cloudViewport.addEventListener('pointerenter', () => {
    sim.navigation.activeViewport = 'cloud';
  });
  setupFreeLookControls(renderer.domElement, 'cloud');
  renderer.domElement.tabIndex = 0;
  window.addEventListener('resize', resize);
  resize();

  return { scene, camera, renderer, controls, grid };
}

function createDroneMesh() {
  const drone = new THREE.Group();

  const frameMaterial = new THREE.MeshStandardMaterial({
    color: '#79b8ff',
    roughness: 0.35,
    metalness: 0.28
  });
  const accentMaterial = new THREE.MeshStandardMaterial({
    color: '#f1f7ff',
    roughness: 0.28,
    metalness: 0.18
  });

  const body = new THREE.Mesh(new THREE.BoxGeometry(0.46, 0.14, 0.28), frameMaterial);
  drone.add(body);

  const armA = new THREE.Mesh(new THREE.BoxGeometry(0.92, 0.04, 0.05), frameMaterial);
  const armB = armA.clone();
  armB.rotation.y = Math.PI / 2;
  drone.add(armA, armB);

  const sensor = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.09, 0.13, 24), accentMaterial);
  sensor.rotation.x = Math.PI / 2;
  sensor.position.set(0, -0.04, 0.18);
  drone.add(sensor);

  const propGeometry = new THREE.TorusGeometry(0.13, 0.016, 8, 24);
  const propMaterial = new THREE.MeshBasicMaterial({ color: '#9ad0ff' });
  [
    [0.35, 0.02, 0.19],
    [-0.35, 0.02, 0.19],
    [0.35, 0.02, -0.19],
    [-0.35, 0.02, -0.19]
  ].forEach(([x, y, z]) => {
    const prop = new THREE.Mesh(propGeometry, propMaterial);
    prop.rotation.x = Math.PI / 2;
    prop.position.set(x, y, z);
    drone.add(prop);
  });

  return drone;
}

function createSwarmDroneMesh(role, index) {
  const drone = createDroneMesh();
  const color = roleColor(role);
  drone.traverse((child) => {
    if (!child.isMesh || !child.material) {
      return;
    }
    child.material = child.material.clone();
    if (child.material.color) {
      child.material.color.set(index === 0 ? '#f4f8ff' : color);
    }
  });
  drone.scale.setScalar(0.84);
  drone.userData.role = role;
  drone.userData.swarmIndex = index;
  return drone;
}

function refreshSwarmDroneRoleVisual(agent) {
  if (!agent.mesh || agent.mesh.userData.role === agent.role) {
    return;
  }
  const color = roleColor(agent.role);
  const index = agent.mesh.userData.swarmIndex ?? 0;
  agent.mesh.traverse((child) => {
    if (!child.isMesh || !child.material?.color) {
      return;
    }
    child.material.color.set(index === 0 ? '#f4f8ff' : color);
  });
  agent.mesh.userData.role = agent.role;
}

function createPointCloud() {
  const cloudPoints = new THREE.Points(sim.cloudGeometry, sim.cloudMaterial);
  cloud.scene.add(cloudPoints);
  sim.overlayCloud = cloudPoints;
}

function setupUI() {
  els.startBtn.addEventListener('click', () => startMission());
  els.pauseBtn.addEventListener('click', () => pauseMission());
  els.continueBtn.addEventListener('click', () => continueMission());
  els.stopBtn.addEventListener('click', () => stopMission());
  els.resetBtn.addEventListener('click', () => resetMission(true, false));
  els.randomizeBtn.addEventListener('click', () => {
    resetMission(false, false);
    regenerateEnvironment(true);
    syncSwarmMode();
    logMessage(`Terrain sandbox randomized. Seed ${sim.environmentSeed}.`);
  });
  els.exportBtn.addEventListener('click', () => exportPointCloudAsPly());
  els.telemetryBtn.addEventListener('click', () => toggleTelemetryPanel());
  els.algorithmBtn.addEventListener('click', () => toggleAlgorithmPanel());
  els.policyExperimentBtn.addEventListener('click', () => togglePolicyExperimentBatch());
  els.telemetryRefreshBtn.addEventListener('click', () => refreshTelemetryPanel());
  els.telemetryExportBtn.addEventListener('click', () => exportTelemetryRuns());
  els.telemetryExportCsvBtn.addEventListener('click', () => exportTelemetryRunsCsv());
  els.telemetrySort.addEventListener('change', () => refreshTelemetryPanel());
  els.telemetryScenarioFilter.addEventListener('change', () => refreshTelemetryPanel());
  els.telemetryWorkspaceFilter.addEventListener('change', () => {
    refreshWorkspaceClearControls();
    refreshTelemetryPanel();
  });
  els.telemetryValidOnly.addEventListener('change', () => refreshTelemetryPanel());
  els.workspaceClearConfirm.addEventListener('input', () => refreshWorkspaceClearControls());
  els.workspaceRemoveName.addEventListener('change', () => refreshWorkspaceClearControls());
  els.workspaceClearBtn.addEventListener('click', () => clearSelectedWorkspaceTelemetry());
  els.datasetWorkspaceSaveBtn.addEventListener('click', () => saveCurrentDatasetWorkspace());
  els.datasetWorkspaceSelect.addEventListener('change', () => {
    if (els.datasetWorkspaceSelect.value) {
      els.datasetWorkspace.value = els.datasetWorkspaceSelect.value;
      saveCurrentDatasetWorkspace(false);
      refreshWorkspaceClearControls();
      refreshTelemetryPanel();
    }
  });
  els.datasetWorkspace.addEventListener('change', () => {
    saveCurrentDatasetWorkspace(false);
    refreshWorkspaceClearControls();
    refreshTelemetryPanel();
  });
  document.querySelectorAll('.controls-panel input, .controls-panel select, .controls-panel textarea').forEach((control) => {
    control.addEventListener('change', () => restoreViewportKeyboardFocus());
  });

  els.sensorPreset.addEventListener('change', () => {
    resetMissionForControlChange('Sensor preset changed; previous run saved and map cleared.');
    applySensorPreset(true);
    logMessage(`Sensor preset set to ${SENSOR_PRESETS[els.sensorPreset.value].label}.`);
  });

  [
    els.terrainWidth,
    els.terrainDepth,
    els.terrainHeight,
    els.terrainRidgeHeight,
    els.terrainValleyWidth,
    els.villageCount,
    els.treeCount
  ].forEach((input) => {
    input.addEventListener('change', () => {
      resetMissionForControlChange('Terrain controls changed; previous run saved and map cleared.');
      regenerateEnvironment(false, { fitCamera: false });
      syncSwarmMode();
      logMessage('Terrain generation updated from the controls.');
    });
  });

  [
    els.scanMode,
    els.plannerMode,
    els.pathStyle,
    els.safetyRadius,
    els.terrainFlightClearance,
    els.terrainVoxelResolution
  ].forEach((input) => {
    input.addEventListener('change', () => {
      resetMissionForControlChange('Mapping or clearance controls changed; previous run saved and map cleared.');
      initializeMapper();
      logMessage('Swarm mapping and clearance settings updated.');
    });
  });

  [els.moveSpeed, els.horizontalRays, els.verticalRays, els.horizontalFov, els.verticalFov, els.maxRange, els.voxelSize, els.dwellMs].forEach(
    (input) => {
      input.addEventListener('change', () => {
        if (input === els.horizontalRays || input === els.verticalRays) {
          els.swarmScanDensity.value = 'custom';
        }
        noteTelemetryControlChange(input.id);
        refreshStatus();
        drawMissionGraph();
      });
    }
  );

  [els.visiblePointBudget, els.renderScaleBudget].forEach((input) => {
    input.addEventListener('change', () => {
      els.performanceBudget.value = 'custom';
      applyPerformanceBudget(false);
      refreshStatus();
    });
  });

  [
    els.communicationRange,
    els.communicationNeighbors,
    els.communicationDropout,
    els.swarmFormation
  ].forEach((input) => {
    input.addEventListener('change', () => {
      applyLiveSwarmConfigChange(input);
      refreshStatus();
      drawMissionGraph();
    });
  });

  [els.swarmScanDensity, els.performanceBudget].forEach((input) => {
    input.addEventListener('change', () => {
      if (input === els.swarmScanDensity) {
        applySwarmScanDensityPreset();
        noteTelemetryControlChange('scan-density');
      }
      if (input === els.performanceBudget) {
        applyPerformanceBudget(true);
        noteTelemetryControlChange('performance-budget');
      }
      refreshStatus();
      drawMissionGraph();
    });
  });

  els.objectiveProfile.addEventListener('change', () => {
    applyObjectiveProfileChange();
    refreshStatus();
    refreshTelemetryPanel();
  });

  [
    els.policyCoverageArea,
    els.policyAoiDetail,
    els.policyRiskSafety,
    els.policyResourceEfficiency
  ].forEach((input) => {
    input.addEventListener('input', () => {
      syncPolicyCoordinateOutputs();
      renderAlgorithmPanel();
    });
    input.addEventListener('change', () => {
      applyPolicyCoordinateChange(input.id);
      refreshStatus();
      refreshTelemetryPanel();
    });
  });

  els.runtimeNudgeProfile.addEventListener('change', () => {
    applyRuntimeNudgeProfileChange();
    refreshStatus();
    refreshTelemetryPanel();
  });

  [els.missionMode, els.swarmSize, els.environmentMode, els.aoiPreset].forEach((input) => {
    input.addEventListener('change', () => {
      resetMissionForControlChange('Experiment-defining swarm controls changed; previous run saved and map cleared.');
      if (input === els.environmentMode || input === els.aoiPreset) {
        regenerateEnvironment(false, { fitCamera: false });
      }
      syncSwarmMode();
      refreshStatus();
      drawMissionGraph();
    });
  });

  makeOverlayDraggable();
  setupViewportNavigation();
  initializeDatasetWorkspaceManager();
  applyPerformanceBudget(true);
  refreshStatus();
}

function setupViewportNavigation() {
  window.addEventListener('keydown', (event) => {
    if (isEditableTarget(event.target)) {
      return;
    }
    const key = event.key.toLowerCase();
    if (!['w', 'a', 's', 'd', 'q', 'e', 'shift'].includes(key)) {
      return;
    }
    sim.navigation.keys.add(key);
    event.preventDefault();
  });

  window.addEventListener('keyup', (event) => {
    sim.navigation.keys.delete(event.key.toLowerCase());
  });

  window.addEventListener('blur', () => {
    sim.navigation.keys.clear();
  });
}

function isEditableTarget(target) {
  const tag = target?.tagName?.toLowerCase();
  return tag === 'input' || tag === 'select' || tag === 'textarea' || target?.isContentEditable;
}

function restoreViewportKeyboardFocus() {
  if (document.activeElement?.closest?.('.controls-panel')) {
    document.activeElement.blur();
  }
  const targetScene = sim.navigation.activeViewport === 'cloud' ? cloud : main;
  targetScene?.renderer?.domElement?.focus?.({ preventScroll: true });
}

function initializeDatasetWorkspaceManager() {
  const workspaces = datasetWorkspaceList();
  const current = localStorage.getItem(`${DATASET_WORKSPACES_KEY}:active`) || workspaces[0] || DEFAULT_DATASET_WORKSPACE;
  els.datasetWorkspace.value = current;
  renderDatasetWorkspaceOptions(workspaces.includes(current) ? workspaces : [current, ...workspaces]);
}

function datasetWorkspaceList() {
  try {
    const parsed = JSON.parse(localStorage.getItem(DATASET_WORKSPACES_KEY) || '[]');
    const values = Array.isArray(parsed) ? parsed : [];
    return uniqueWorkspaceNames([DEFAULT_DATASET_WORKSPACE, ...values]);
  } catch {
    return [DEFAULT_DATASET_WORKSPACE];
  }
}

function saveCurrentDatasetWorkspace(writeLog = true) {
  const workspace = datasetWorkspaceName();
  const workspaces = uniqueWorkspaceNames([workspace, ...datasetWorkspaceList()]);
  localStorage.setItem(DATASET_WORKSPACES_KEY, JSON.stringify(workspaces));
  localStorage.setItem(`${DATASET_WORKSPACES_KEY}:active`, workspace);
  renderDatasetWorkspaceOptions(workspaces);
  if (writeLog) {
    logMessage(`Dataset workspace set to ${workspace}. New runs and exports will be tagged with this workspace.`);
  }
}

function renderDatasetWorkspaceOptions(workspaces = datasetWorkspaceList()) {
  const current = datasetWorkspaceName();
  const previousFilter = els.telemetryWorkspaceFilter.value || 'active';
  els.datasetWorkspaceSelect.innerHTML = workspaces
    .map((workspace) => `<option value="${escapeHtml(workspace)}"${workspace === current ? ' selected' : ''}>${escapeHtml(workspace)}</option>`)
    .join('');
  els.telemetryWorkspaceFilter.innerHTML = [
    '<option value="active" selected>Active workspace</option>',
    '<option value="all">All workspaces</option>',
    '<option value="legacy">Legacy/untagged</option>',
    ...workspaces.map((workspace) => `<option value="${escapeHtml(workspace)}">${escapeHtml(workspace)}</option>`)
  ].join('');
  els.telemetryWorkspaceFilter.value = [...els.telemetryWorkspaceFilter.options].some((option) => option.value === previousFilter)
    ? previousFilter
    : 'active';
  refreshWorkspaceClearControls();
}

function uniqueWorkspaceNames(values) {
  return [...new Set(values
    .map((value) => String(value || '').trim())
    .filter(Boolean))]
    .slice(0, 30);
}

function workspaceClearTarget() {
  const selected = els.telemetryWorkspaceFilter?.value ?? 'active';
  if (selected === 'all') {
    return null;
  }
  if (selected === 'legacy') {
    return {
      key: 'legacy',
      label: 'legacy',
      removeNameAllowed: false
    };
  }
  const workspace = selected === 'active' ? datasetWorkspaceName() : selected;
  return {
    key: workspace,
    label: workspace,
    removeNameAllowed: workspace !== DEFAULT_DATASET_WORKSPACE
  };
}

function refreshWorkspaceClearControls() {
  if (!els.workspaceClearTarget) {
    return;
  }
  const target = workspaceClearTarget();
  const typed = els.workspaceClearConfirm.value.trim();
  const exactMatch = Boolean(target && typed === target.label);
  els.workspaceClearTarget.textContent = target
    ? `Target: ${target.label}`
    : 'Select one workspace; all workspaces cannot be cleared here.';
  els.workspaceClearBtn.disabled = !exactMatch;
  els.workspaceRemoveName.disabled = !target?.removeNameAllowed;
  if (!target?.removeNameAllowed) {
    els.workspaceRemoveName.checked = false;
  }
}

async function clearSelectedWorkspaceTelemetry() {
  const target = workspaceClearTarget();
  if (!target || els.workspaceClearConfirm.value.trim() !== target.label) {
    refreshWorkspaceClearControls();
    return;
  }
  const deletedCount = await sim.swarmTelemetry.deleteRunsByWorkspace({
    workspace: target.key,
    modelFamily: GREYBOX_POLICY_MODEL.modelFamily
  });
  if (els.workspaceRemoveName.checked && target.removeNameAllowed) {
    removeDatasetWorkspaceName(target.key);
  }
  els.workspaceClearConfirm.value = '';
  refreshWorkspaceClearControls();
  refreshTelemetryPanel();
  logMessage(`Cleared ${deletedCount.toLocaleString()} telemetry run(s) from workspace ${target.label}.`);
}

function removeDatasetWorkspaceName(workspace) {
  const workspaces = datasetWorkspaceList().filter((name) => name !== workspace || name === DEFAULT_DATASET_WORKSPACE);
  localStorage.setItem(DATASET_WORKSPACES_KEY, JSON.stringify(workspaces));
  if (datasetWorkspaceName() === workspace && workspace !== DEFAULT_DATASET_WORKSPACE) {
    els.datasetWorkspace.value = DEFAULT_DATASET_WORKSPACE;
    localStorage.setItem(`${DATASET_WORKSPACES_KEY}:active`, DEFAULT_DATASET_WORKSPACE);
  }
  renderDatasetWorkspaceOptions(workspaces);
}

function applySensorPreset(writeLog) {
  const preset = SENSOR_PRESETS[els.sensorPreset.value] || SENSOR_PRESETS.spinning;
  if (els.sensorPreset.value !== 'custom') {
    els.horizontalRays.value = String(preset.horizontalRays);
    els.verticalRays.value = String(preset.verticalRays);
    els.horizontalFov.value = String(preset.horizontalFov);
    els.verticalFov.value = String(preset.verticalFov);
    els.maxRange.value = String(preset.maxRange);
  }
  if (writeLog) {
    logMessage(`Sensor tuned for ${preset.label}.`);
  }
}

function currentSensorConfig() {
  const preset = SENSOR_PRESETS[els.sensorPreset.value] || SENSOR_PRESETS.spinning;
  const rangeLimit = 80;
  return {
    label: preset.label,
    mode: preset.mode,
    horizontalRays: clampInt(readNumber(els.horizontalRays), 16, 220),
    verticalRays: clampInt(readNumber(els.verticalRays), 8, 96),
    horizontalFov: clamp(readNumber(els.horizontalFov), 45, 360),
    verticalFov: clamp(readNumber(els.verticalFov), 12, 130),
    maxRange: clamp(readNumber(els.maxRange), 2, rangeLimit),
    pitchBiasRad: THREE.MathUtils.degToRad(preset.pitchBiasDeg)
  };
}

function regenerateEnvironment(randomizeSeed, { fitCamera = true } = {}) {
  if (randomizeSeed || !sim.environmentSeed) {
    sim.environmentSeed = generateSeed();
  }

  const rng = createRng(sim.environmentSeed);
  sim.terrainConfig = currentTerrainConfig();
  sim.room = {
    preset: 'terrain',
    width: sim.terrainConfig.width,
    depth: sim.terrainConfig.depth,
    height: sim.terrainConfig.height
  };

  sim.floorFootprints = [];
  sim.ceilingHazard = null;
  sim.aoiTargets = [];
  sim.terrainProfile = null;
  sim.scanTargets = [];
  main.roomGroup.clear();
  main.pathGroup.clear();
  main.beamGroup.clear();

  const roomScene = new THREE.Group();
  buildTerrainSandbox(rng).forEach((object) => roomScene.add(object));

  main.roomGroup.add(roomScene);
  sim.scanTargets = collectMeshes(roomScene);
  prepareRaycastAcceleration(sim.scanTargets);
  initializeMapper();
  applyEnvironmentAtmosphere();
  if (fitCamera) {
    fitCamerasToRoom();
  } else {
    updateCloudReferenceGrid();
    main.controls.update();
    cloud.controls.update();
  }
}

function currentTerrainConfig() {
  return {
    width: clamp(readNumber(els.terrainWidth), 80, 320),
    depth: clamp(readNumber(els.terrainDepth), 80, 320),
    height: clamp(readNumber(els.terrainHeight), 30, 120),
    ridgeHeight: clamp(readNumber(els.terrainRidgeHeight), 12, 80),
    valleyWidth: clamp(readNumber(els.terrainValleyWidth), 10, 60),
    villageCount: clampInt(readNumber(els.villageCount), 0, 24),
    treeCount: clampInt(readNumber(els.treeCount), 0, 140),
    voxelResolution: clamp(readNumber(els.terrainVoxelResolution), 2, 14)
  };
}

function applySwarmScanDensityPreset() {
  const preset = swarmScanDensityConfig(els.swarmScanDensity.value);
  if (!preset.writeInputs) {
    return;
  }
  els.horizontalRays.value = String(preset.horizontal);
  els.verticalRays.value = String(preset.vertical);
}

function performanceBudgetPresets() {
  return {
    safe: {
      label: 'GPU safe',
      renderScale: 1.15,
      visiblePoints: 65000,
      targetFrameMs: 24,
      scanIntervalMs: 235,
      cloudFlushMs: 1150,
      commRenderMs: 270,
      graphRenderMs: 280,
      beamDroneCap: 3,
      beamSamples: 5
    },
    balanced: {
      label: 'Balanced',
      renderScale: 1.5,
      visiblePoints: 125000,
      targetFrameMs: 20,
      scanIntervalMs: 175,
      cloudFlushMs: 760,
      commRenderMs: 165,
      graphRenderMs: 190,
      beamDroneCap: 5,
      beamSamples: 6
    },
    quality: {
      label: 'Quality',
      renderScale: 1.75,
      visiblePoints: 210000,
      targetFrameMs: 18,
      scanIntervalMs: 150,
      cloudFlushMs: 580,
      commRenderMs: 125,
      graphRenderMs: 150,
      beamDroneCap: 7,
      beamSamples: 7
    },
    capture: {
      label: 'Capture',
      renderScale: 1.35,
      visiblePoints: 650000,
      targetFrameMs: 24,
      scanIntervalMs: 165,
      cloudFlushMs: 880,
      commRenderMs: 220,
      graphRenderMs: 230,
      beamDroneCap: 5,
      beamSamples: 6
    }
  };
}

function readPerformanceBudgetConfig() {
  const presets = performanceBudgetPresets();
  const selected = presets[els.performanceBudget.value] ?? null;
  if (selected) {
    return selected;
  }

  return {
    label: 'Custom',
    renderScale: clamp(readNumber(els.renderScaleBudget), 0.65, 2),
    visiblePoints: clampInt(readNumber(els.visiblePointBudget), 10000, 1200000),
    targetFrameMs: 24,
    scanIntervalMs: 210,
    cloudFlushMs: 950,
    commRenderMs: 220,
    graphRenderMs: 240,
    beamDroneCap: 4,
    beamSamples: 5
  };
}

function applyPerformanceBudget(writeInputs) {
  const preset = readPerformanceBudgetConfig();
  if (writeInputs && els.performanceBudget.value !== 'custom') {
    els.visiblePointBudget.value = String(preset.visiblePoints);
    els.renderScaleBudget.value = String(preset.renderScale);
  }
  setRendererScale(preset.renderScale);
  updatePointReadouts();
}

function resetMissionForControlChange(message) {
  if (sim.missionStarted || sim.status === 'paused') {
    resetMission(false, false);
    logMessage(message);
    return true;
  }
  return false;
}

function applyLiveSwarmConfigChange(input) {
  if (input === els.swarmFormation) {
    noteTelemetryControlChange('formation');
  }
  if (!sim.swarmController) {
    syncSwarmMode();
    return;
  }

  const communicationRange = clamp(readNumber(els.communicationRange), 2, 80);
  const maxNeighbors = clampInt(readNumber(els.communicationNeighbors), 1, 8);
  const dropout = clamp(readNumber(els.communicationDropout), 0, 0.45);
  sim.swarmController.config.communicationRange = communicationRange;
  sim.swarmController.config.maxNeighbors = maxNeighbors;
  sim.swarmController.config.communicationDropout = dropout;
  sim.swarmController.config.formationMode = els.swarmFormation.value;
  sim.swarmController.communicationGraph.range = communicationRange;
  sim.swarmController.communicationGraph.maxNeighbors = maxNeighbors;
  sim.swarmController.communicationGraph.dropout = dropout;
  noteTelemetryControlChange(input.id);
}

function noteTelemetryControlChange(controlName) {
  if (!sim.swarmTelemetry?.active) {
    return;
  }
  sim.swarmTelemetry.addNote?.(`Control changed during run: ${controlName}.`);
}

function setRendererScale(renderScale) {
  const scale = Math.min(window.devicePixelRatio || 1, clamp(renderScale, 0.65, 2));
  if (Math.abs(scale - sim.performance.currentRenderScale) < 0.03) {
    return;
  }

  sim.performance.currentRenderScale = scale;
  main.renderer.setPixelRatio(scale);
  cloud.renderer.setPixelRatio(scale);
  resizeRendererToElement(main.renderer, main.camera, els.viewport);
  resizeRendererToElement(cloud.renderer, cloud.camera, els.cloudViewport);
}

function resizeRendererToElement(renderer, camera, element) {
  const width = Math.max(element.clientWidth, 1);
  const height = Math.max(element.clientHeight, 1);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
}

function updatePerformanceBudget(deltaMs) {
  const config = readPerformanceBudgetConfig();
  sim.performance.frameAvgMs = lerp(sim.performance.frameAvgMs, Math.max(deltaMs, 1), 0.06);
  sim.performance.loadScale = clamp(sim.performance.frameAvgMs / Math.max(config.targetFrameMs, 1), 0.85, 2.3);
  sim.performance.renderScaleElapsedMs += deltaMs;

  if (sim.performance.renderScaleElapsedMs < 1500) {
    return;
  }

  sim.performance.renderScaleElapsedMs = 0;
  const overloadScale = sim.performance.loadScale > 1.12 ? Math.min(1.32, sim.performance.loadScale / 1.08) : 1;
  setRendererScale(config.renderScale / overloadScale);
}

function budgetedInterval(baseMs) {
  return Math.round(baseMs * clamp(sim.performance.loadScale, 0.9, 1.85));
}

function applyEnvironmentAtmosphere() {
  main.scene.background = new THREE.Color('#0b1625');
  main.scene.fog = new THREE.Fog('#0b1625', 320, 760);
}

function buildRoomShell(room) {
  const halfW = room.width * 0.5;
  const halfD = room.depth * 0.5;
  const wallThickness = 0.08;
  const floorThickness = 0.12;

  const floorMaterial = new THREE.MeshStandardMaterial({
    color: '#111a2b',
    roughness: 0.92,
    metalness: 0.03
  });
  const wallMaterial = new THREE.MeshStandardMaterial({
    color: '#26324a',
    roughness: 0.88,
    metalness: 0.02,
    transparent: true,
    opacity: 0.18
  });
  const ceilingMaterial = new THREE.MeshStandardMaterial({
    color: '#24324a',
    roughness: 0.95,
    metalness: 0.02,
    transparent: true,
    opacity: 0.08
  });

  const floor = new THREE.Mesh(new THREE.BoxGeometry(room.width, floorThickness, room.depth), floorMaterial);
  floor.position.y = -floorThickness * 0.5;

  const ceiling = new THREE.Mesh(new THREE.BoxGeometry(room.width, wallThickness, room.depth), ceilingMaterial);
  ceiling.position.y = room.height + wallThickness * 0.5;

  const backWall = new THREE.Mesh(new THREE.BoxGeometry(room.width, room.height, wallThickness), wallMaterial);
  backWall.position.set(0, room.height * 0.5, -halfD - wallThickness * 0.5);

  const frontWall = backWall.clone();
  frontWall.position.z = halfD + wallThickness * 0.5;

  const leftWall = new THREE.Mesh(new THREE.BoxGeometry(wallThickness, room.height, room.depth), wallMaterial);
  leftWall.position.set(-halfW - wallThickness * 0.5, room.height * 0.5, 0);

  const rightWall = leftWall.clone();
  rightWall.position.x = halfW + wallThickness * 0.5;

  const grid = new THREE.GridHelper(Math.max(room.width, room.depth) + 2, 28, '#33517f', '#1a2940');
  grid.position.y = 0.002;

  return [floor, ceiling, backWall, frontWall, leftWall, rightWall, grid];
}

function buildTerrainSandbox(rng) {
  const config = sim.terrainConfig ?? currentTerrainConfig();
  const objects = [];
  const terrain = createTerrainMesh();
  objects.push(terrain);

  const grid = new THREE.GridHelper(Math.max(sim.room.width, sim.room.depth), 48, '#456c9f', '#1a314c');
  grid.position.y = 0.015;
  objects.push(grid);
  objects.push(buildTerrainReferenceMarkers());

  const village = buildMountainVillage(rng, config.villageCount);
  objects.push(village);

  const forest = buildTreeField(rng, config.treeCount);
  objects.push(forest);

  const complex = buildComplexBuildingAoi();
  objects.push(complex);

  const aperture = buildValleyAperture();
  objects.push(aperture);

  const markers = buildAoiMarkers();
  objects.push(markers);

  sim.terrainProfile = {
    valleyWidth: config.valleyWidth,
    ridgeHeight: config.ridgeHeight,
    corridorScore: 0.68,
    verticality: 0.72,
    openness: 0.64
  };

  return objects;
}

function buildTerrainReferenceMarkers() {
  const group = new THREE.Group();
  const valleyMaterial = new THREE.LineBasicMaterial({ color: '#80d8ff', transparent: true, opacity: 0.58 });
  const boundaryMaterial = new THREE.LineBasicMaterial({ color: '#ffdf7d', transparent: true, opacity: 0.35 });
  const valleyPoints = [];
  for (let z = -sim.room.depth * 0.45; z <= sim.room.depth * 0.45; z += 2) {
    valleyPoints.push(new THREE.Vector3(0, terrainHeightAt(0, z) + 0.04, z));
  }
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(valleyPoints), valleyMaterial));

  const halfW = sim.room.width * 0.5;
  const halfD = sim.room.depth * 0.5;
  const corners = [
    new THREE.Vector3(-halfW, 0.08, -halfD),
    new THREE.Vector3(halfW, 0.08, -halfD),
    new THREE.Vector3(halfW, 0.08, halfD),
    new THREE.Vector3(-halfW, 0.08, halfD),
    new THREE.Vector3(-halfW, 0.08, -halfD)
  ];
  group.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(corners), boundaryMaterial));
  return group;
}

function terrainHeightAt(x, z) {
  const config = sim.terrainConfig ?? currentTerrainConfig();
  const halfW = sim.room.width * 0.5;
  const halfD = sim.room.depth * 0.5;
  const scaleX = sim.room.width / 180;
  const scaleZ = sim.room.depth / 160;
  const avgScale = (scaleX + scaleZ) * 0.5;
  const heightScale = config.ridgeHeight / 52;
  const valleyCenter = Math.sin(z * 0.026 / avgScale) * 9 * scaleX + Math.sin(z * 0.061 / avgScale) * 3 * scaleX;
  const distFromValley = x - valleyCenter;
  const edgeDistance = Math.min(halfW - Math.abs(x), halfD - Math.abs(z));
  const edgeFade = THREE.MathUtils.smoothstep(edgeDistance, 0, 28 * avgScale);
  const valleyFloor = 2.4 + Math.sin(z * 0.045 / avgScale) * 1.5 + Math.sin((x + z) * 0.028 / avgScale) * 0.6;
  const valleyOffset = config.valleyWidth * 1.8 * scaleX;
  const leftRidge = Math.exp(-(((distFromValley + valleyOffset) ** 2) / (650 * scaleX * scaleX))) * 28 * heightScale;
  const rightRidge = Math.exp(-(((distFromValley - valleyOffset * 1.08) ** 2) / (700 * scaleX * scaleX))) * 31 * heightScale;
  const foldedLeft = Math.exp(-(((distFromValley + valleyOffset * 1.45) ** 2) / (380 * scaleX * scaleX) + ((z + 28 * scaleZ) ** 2) / (2200 * scaleZ * scaleZ))) * 18 * heightScale;
  const foldedRight = Math.exp(-(((distFromValley - valleyOffset * 1.55) ** 2) / (420 * scaleX * scaleX) + ((z - 34 * scaleZ) ** 2) / (1800 * scaleZ * scaleZ))) * 21 * heightScale;
  const northPeak = Math.exp(-(((x + 54 * scaleX) ** 2) / (620 * scaleX * scaleX) + ((z + 34 * scaleZ) ** 2) / (920 * scaleZ * scaleZ))) * 19 * heightScale;
  const southPeak = Math.exp(-(((x - 58 * scaleX) ** 2) / (720 * scaleX * scaleX) + ((z - 38 * scaleZ) ** 2) / (820 * scaleZ * scaleZ))) * 22 * heightScale;
  const ridgeNoise = (Math.sin(z * 0.085 / avgScale) * 1.8 + Math.sin((x - z) * 0.05 / avgScale) * 1.2) * heightScale;
  const height = valleyFloor + leftRidge + rightRidge + foldedLeft + foldedRight + northPeak + southPeak + ridgeNoise;
  return Math.max(0, height * (0.22 + edgeFade * 0.78));
}

function findTerrainPlacement(anchorX, anchorZ, footprintRadius, searchRadius = 18) {
  const halfW = sim.room.width * 0.48;
  const halfD = sim.room.depth * 0.48;
  let best = null;
  for (let dz = -searchRadius; dz <= searchRadius; dz += 4) {
    for (let dx = -searchRadius; dx <= searchRadius; dx += 4) {
      const x = clamp(anchorX + dx, -halfW, halfW);
      const z = clamp(anchorZ + dz, -halfD, halfD);
      const patch = sampleTerrainPatch(x, z, footprintRadius);
      const distancePenalty = Math.hypot(dx, dz) * 0.08;
      const score = patch.range * 3.2 + patch.slope * 28 + distancePenalty;
      if (!best || score < best.score) {
        best = { ...patch, x, z, score };
      }
    }
  }
  return best ?? sampleTerrainPatch(anchorX, anchorZ, footprintRadius);
}

function sampleTerrainPatch(x, z, radius) {
  const offsets = [
    [0, 0],
    [-radius, -radius],
    [radius, -radius],
    [-radius, radius],
    [radius, radius],
    [-radius, 0],
    [radius, 0],
    [0, -radius],
    [0, radius]
  ];
  const heights = offsets.map(([dx, dz]) => terrainHeightAt(x + dx, z + dz));
  const min = Math.min(...heights);
  const max = Math.max(...heights);
  const avg = heights.reduce((sum, value) => sum + value, 0) / heights.length;
  const step = Math.max(1.5, radius * 0.55);
  const normal = new THREE.Vector3(
    terrainHeightAt(x - step, z) - terrainHeightAt(x + step, z),
    step * 2,
    terrainHeightAt(x, z - step) - terrainHeightAt(x, z + step)
  ).normalize();
  return {
    x,
    z,
    min,
    max,
    avg,
    range: max - min,
    slope: Math.acos(clamp(normal.y, -1, 1)),
    normal
  };
}

function createTerrainFoundation(width, depth, placement, material, topOffset = 0.06) {
  const height = Math.max(0.24, placement.max - placement.min + 0.34);
  const pad = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), material);
  pad.position.y = topOffset - height * 0.5;
  return pad;
}

function createTerrainMesh() {
  const width = sim.room.width;
  const depth = sim.room.depth;
  const geometry = new THREE.PlaneGeometry(width, depth, 96, 84);
  geometry.rotateX(-Math.PI / 2);
  const positions = geometry.attributes.position;
  for (let index = 0; index < positions.count; index += 1) {
    const x = positions.getX(index);
    const z = positions.getZ(index);
    positions.setY(index, terrainHeightAt(x, z));
  }
  geometry.computeVertexNormals();

  const material = new THREE.MeshStandardMaterial({
    color: '#3d815f',
    roughness: 0.88,
    metalness: 0.02,
    side: THREE.DoubleSide
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = 'Mountain valley terrain';
  return mesh;
}

function buildMountainVillage(rng, count) {
  const group = new THREE.Group();
  if (count <= 0) {
    return group;
  }
  const houseMaterial = new THREE.MeshStandardMaterial({ color: '#b8a175', roughness: 0.78, metalness: 0.03 });
  const roofMaterial = new THREE.MeshStandardMaterial({ color: '#7b3f4e', roughness: 0.72, metalness: 0.02 });
  const baseX = -sim.room.width * 0.32;
  const baseZ = -sim.room.depth * 0.29;
  const columns = Math.ceil(Math.sqrt(count));
  const laneX = Math.max(8.5, sim.room.width * 0.058);
  const laneZ = Math.max(8, sim.room.depth * 0.062);

  for (let index = 0; index < count; index += 1) {
    const x = baseX + (index % columns) * laneX + rng() * 1.4;
    const z = baseZ + Math.floor(index / columns) * laneZ + rng() * 1.4;
    const y = terrainHeightAt(x, z);
    const width = 3.0 + rng() * 0.8;
    const depth = 2.8 + rng() * 0.8;
    const height = 2.2 + rng() * 1.2;

    const body = new THREE.Mesh(new THREE.BoxGeometry(width, height, depth), houseMaterial);
    body.position.set(x, y + height * 0.5, z);
    group.add(body);

    const roof = new THREE.Mesh(new THREE.ConeGeometry(Math.max(width, depth) * 0.72, 1.0, 4), roofMaterial);
    roof.position.set(x, y + height + 0.5, z);
    roof.rotation.y = Math.PI * 0.25;
    group.add(roof);

    sim.floorFootprints.push({
      minX: x - width * 0.58,
      maxX: x + width * 0.58,
      minZ: z - depth * 0.58,
      maxZ: z + depth * 0.58,
      height: y + height + 1.2,
      type: 'village'
    });
  }

  sim.aoiTargets.push({
    id: 'aoi-village',
    type: 'village',
    label: 'Village cluster',
    position: new THREE.Vector3(baseX + laneX, terrainHeightAt(baseX + laneX, baseZ + laneZ) + 5.2, baseZ + laneZ),
    priority: els.aoiPreset.value === 'village' ? 2.2 : 1.2,
    radius: Math.max(14, laneX * 1.7)
  });

  return group;
}

function buildTreeField(rng, count) {
  const group = new THREE.Group();
  const trunkMaterial = new THREE.MeshStandardMaterial({ color: '#6e4d35', roughness: 0.84, metalness: 0.02 });
  const leafMaterial = new THREE.MeshStandardMaterial({ color: '#2f8b5f', roughness: 0.88, metalness: 0.02 });

  for (let index = 0; index < count; index += 1) {
    const x = lerp(-sim.room.width * 0.42, sim.room.width * 0.42, rng());
    const z = lerp(-sim.room.depth * 0.42, sim.room.depth * 0.42, rng());
    if (Math.abs(x) < 4.8 && Math.abs(z) < 12) {
      continue;
    }
    const y = terrainHeightAt(x, z);
    const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.26, 2.1, 8), trunkMaterial);
    trunk.position.set(x, y + 1.05, z);
    const leaves = new THREE.Mesh(new THREE.ConeGeometry(1.2, 3.2, 9), leafMaterial);
    leaves.position.set(x, y + 3.05, z);
    group.add(trunk, leaves);

    sim.floorFootprints.push({
      minX: x - 0.9,
      maxX: x + 0.9,
      minZ: z - 0.9,
      maxZ: z + 0.9,
      height: y + 4.1,
      type: 'tree'
    });
  }

  return group;
}

function buildComplexBuildingAoi() {
  const group = new THREE.Group();
  const placement = findTerrainPlacement(sim.room.width * 0.26, -sim.room.depth * 0.28, 8, 30);
  const x = placement.x;
  const z = placement.z;
  const y = placement.max;
  group.position.set(x, y, z);
  group.rotation.y = -0.12;
  const concrete = new THREE.MeshStandardMaterial({ color: '#aeb6bf', roughness: 0.82, metalness: 0.04 });
  const dark = new THREE.MeshStandardMaterial({ color: '#2f3845', roughness: 0.72, metalness: 0.08 });
  const glass = new THREE.MeshStandardMaterial({ color: '#5d8fb0', roughness: 0.44, metalness: 0.16 });
  const foundationMaterial = new THREE.MeshStandardMaterial({ color: '#6f735f', roughness: 0.9, metalness: 0.02 });

  group.add(createTerrainFoundation(14.2, 10.8, placement, foundationMaterial, 0.04));

  const podium = new THREE.Mesh(new THREE.BoxGeometry(12, 3.2, 9), concrete);
  podium.position.set(0, 1.68, 0);
  const tower = new THREE.Mesh(new THREE.BoxGeometry(5.6, 10, 5.2), concrete);
  tower.position.set(-2.5, 8.28, -0.6);
  const annex = new THREE.Mesh(new THREE.BoxGeometry(4.6, 5.6, 6.6), dark);
  annex.position.set(4.1, 5.08, 1.0);
  group.add(podium, tower, annex);

  for (let row = 0; row < 3; row += 1) {
    for (let col = 0; col < 3; col += 1) {
      const windowMesh = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.72, 0.72), glass);
      windowMesh.position.set(-5.34, 5.68 + row * 1.55, -2.1 + col * 1.55);
      group.add(windowMesh);
    }
  }

  const roofBeacon = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.28, 1.7, 10), dark);
  roofBeacon.position.set(-2.5, 14.18, -0.6);
  group.add(roofBeacon);

  sim.floorFootprints.push({
    minX: x - 6.7,
    maxX: x + 6.7,
    minZ: z - 5.3,
    maxZ: z + 5.3,
    height: y + 15.2,
    type: 'structure'
  });
  sim.aoiTargets.push({
    id: 'aoi-structure',
    type: 'structure',
    label: 'Complex building',
    position: new THREE.Vector3(x, y + 12.8, z),
    priority: els.aoiPreset.value === 'structure' ? 2.4 : 1.35,
    radius: 18
  });

  return group;
}

function buildValleyAperture() {
  const x = 0;
  const z = sim.room.depth * 0.22;
  const y = terrainHeightAt(x, z) + 4.2;
  const group = new THREE.Group();

  [
    [-10.5, -5.8, 'tracked', -0.25],
    [4.8, -5.6, 'truck', 0.18],
    [-3.8, 2.7, 'field-artifact', Math.PI * 0.45],
    [11.2, 3.4, 'tracked', 0.28],
    [-11.8, 10.6, 'truck', 0.62],
    [4.6, 12.2, 'field-artifact', -0.32],
    [14.2, 10.5, 'truck', -0.35]
  ].forEach(([offsetX, offsetZ, kind, rotation]) => {
    buildStaticSurveyVehicle(group, x + offsetX, z + offsetZ, kind, rotation);
  });

  sim.aoiTargets.push({
    id: 'aoi-aperture',
    type: 'aperture',
    label: 'Valley object field',
    position: new THREE.Vector3(x, y, z),
    priority: els.aoiPreset.value === 'aperture' ? 2.6 : 1.6,
    radius: 24
  });

  return group;
}

function buildStaticSurveyVehicle(group, x, z, kind, rotation) {
  const placement = findTerrainPlacement(x, z, kind === 'field-artifact' ? 2.1 : 2.8, 3);
  const y = placement.max;
  const scale = kind === 'field-artifact' ? 1.32 : kind === 'truck' ? 1.38 : 1.46;
  const bodyMaterial = new THREE.MeshStandardMaterial({ color: kind === 'truck' ? '#68717a' : '#596151', roughness: 0.78, metalness: 0.08 });
  const darkMaterial = new THREE.MeshStandardMaterial({ color: '#22272e', roughness: 0.7, metalness: 0.14 });
  const accentMaterial = new THREE.MeshStandardMaterial({ color: '#8b8371', roughness: 0.74, metalness: 0.08 });
  const padMaterial = new THREE.MeshStandardMaterial({ color: '#595f4a', roughness: 0.92, metalness: 0.02 });
  const vehicle = new THREE.Group();
  const padWidth = kind === 'field-artifact' ? 3.6 : 5.7;
  const padDepth = kind === 'field-artifact' ? 2.4 : 3.4;
  vehicle.add(createTerrainFoundation(padWidth, padDepth, placement, padMaterial, 0.03));

  if (kind === 'truck') {
    const bed = new THREE.Mesh(new THREE.BoxGeometry(4.2, 1.1, 2.0), bodyMaterial);
    bed.position.y = 1.02;
    const cab = new THREE.Mesh(new THREE.BoxGeometry(1.45, 1.5, 1.85), accentMaterial);
    cab.position.set(2.2, 1.22, 0);
    vehicle.add(bed, cab);
    [-1.6, 0.2, 1.8].forEach((wheelX) => {
      [-1.05, 1.05].forEach((wheelZ) => {
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.32, 0.32, 0.24, 12), darkMaterial);
        wheel.rotation.z = Math.PI * 0.5;
        wheel.position.set(wheelX, 0.42, wheelZ);
        vehicle.add(wheel);
      });
    });
  } else if (kind === 'field-artifact') {
    const base = new THREE.Mesh(new THREE.BoxGeometry(2.8, 0.5, 1.5), accentMaterial);
    base.position.y = 0.5;
    const support = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.28, 1.1, 10), darkMaterial);
    support.position.y = 1.13;
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.14, 3.9, 12), darkMaterial);
    barrel.rotation.z = Math.PI * 0.5;
    barrel.rotation.y = -0.16;
    barrel.position.set(1.85, 1.36, 0);
    vehicle.add(base, support, barrel);
  } else {
    const hull = new THREE.Mesh(new THREE.BoxGeometry(4.4, 1.05, 2.45), bodyMaterial);
    hull.position.y = 0.88;
    const turret = new THREE.Mesh(new THREE.BoxGeometry(1.55, 0.72, 1.4), accentMaterial);
    turret.position.set(0.2, 1.63, 0);
    const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.12, 2.7, 12), darkMaterial);
    barrel.rotation.z = Math.PI * 0.5;
    barrel.position.set(1.65, 1.64, 0);
    const leftTrack = new THREE.Mesh(new THREE.BoxGeometry(4.8, 0.38, 0.42), darkMaterial);
    leftTrack.position.set(0, 0.42, -1.28);
    const rightTrack = leftTrack.clone();
    rightTrack.position.z = 1.28;
    vehicle.add(hull, turret, barrel, leftTrack, rightTrack);
  }

  vehicle.position.set(placement.x, y, placement.z);
  vehicle.rotation.y = rotation;
  vehicle.scale.setScalar(scale);
  group.add(vehicle);

  sim.floorFootprints.push({
    minX: placement.x - 3.2 * scale,
    maxX: placement.x + 3.2 * scale,
    minZ: placement.z - 2.4 * scale,
    maxZ: placement.z + 2.4 * scale,
    height: y + 2.6 * scale,
    type: 'static-object'
  });
}

function buildAoiMarkers() {
  const group = new THREE.Group();
  const markerMaterial = new THREE.MeshBasicMaterial({ color: '#ffdf7d', transparent: true, opacity: 0.72 });
  const selected = els.aoiPreset.value;
  sim.aoiTargets.forEach((aoi) => {
    if (selected !== 'auto' && aoi.type !== selected) {
      return;
    }
    const marker = new THREE.Mesh(new THREE.SphereGeometry(0.28, 16, 12), markerMaterial);
    marker.position.copy(aoi.position);
    marker.userData.ignoreRaycast = true;
    marker.userData.aoiMarker = true;
    group.add(marker);
  });
  return group;
}

function buildFurnitureLayout(rng, count) {
  const items = [];
  const footprints = [];
  const templates = getFurnitureTemplates();
  const halfW = sim.room.width * 0.5;
  const halfD = sim.room.depth * 0.5;

  for (let index = 0; index < count; index += 1) {
    const template = pickFurnitureTemplate(rng, templates);
    let placed = null;

    for (let attempt = 0; attempt < 70; attempt += 1) {
      const rotation = rng() < 0.5 ? 0 : Math.PI / 2;
      const dims = rotatedSize(template.width, template.depth, rotation);
      const pos = sampleFurniturePosition(rng, dims, template.wallBias, halfW, halfD);
      const footprint = {
        minX: pos.x - dims.width * 0.5,
        maxX: pos.x + dims.width * 0.5,
        minZ: pos.z - dims.depth * 0.5,
        maxZ: pos.z + dims.depth * 0.5,
        height: template.height,
        type: template.id
      };

      if (overlapsFootprints(footprint, footprints, 0.34) || overlapsSpawnZone(footprint)) {
        continue;
      }

      const group = template.build();
      group.position.set(pos.x, 0, pos.z);
      group.rotation.y = rotation;
      placed = { group, footprint };
      break;
    }

    if (!placed) {
      continue;
    }

    items.push(placed);
    footprints.push(placed.footprint);
  }

  return items;
}

function overlapsSpawnZone(footprint) {
  const padding = 1.1;
  return !(
    footprint.maxX < -padding ||
    footprint.minX > padding ||
    footprint.maxZ < -padding ||
    footprint.minZ > padding
  );
}

function getFurnitureTemplates() {
  const base = [
    {
      id: 'sofa',
      width: 2.1,
      depth: 0.95,
      height: 1.0,
      wallBias: 0.9,
      weight: sim.room.preset === 'living' ? 4 : 2,
      build: buildSofa
    },
    {
      id: 'chair',
      width: 0.7,
      depth: 0.7,
      height: 0.95,
      wallBias: 0.25,
      weight: 3,
      build: buildChair
    },
    {
      id: 'bed',
      width: 2.1,
      depth: 1.7,
      height: 0.95,
      wallBias: 0.95,
      weight: sim.room.preset === 'bedroom' ? 4 : 1,
      build: buildBed
    },
    {
      id: 'table',
      width: 1.4,
      depth: 0.9,
      height: 1.05,
      wallBias: 0.35,
      weight: 2,
      build: buildTable
    },
    {
      id: 'cabinet',
      width: 1.2,
      depth: 0.48,
      height: 1.9,
      wallBias: 0.92,
      weight: sim.room.preset === 'bedroom' ? 2 : 1,
      build: buildCabinet
    }
  ];

  if (sim.room.preset === 'living') {
    return base.filter((item) => item.id !== 'bed');
  }

  if (sim.room.preset === 'bedroom') {
    return base.filter((item) => item.id !== 'sofa');
  }

  return base;
}

function buildSofa() {
  const group = new THREE.Group();
  const frame = new THREE.MeshStandardMaterial({ color: '#6e88d7', roughness: 0.55, metalness: 0.08 });
  const cushion = new THREE.MeshStandardMaterial({ color: '#98b2ff', roughness: 0.5, metalness: 0.05 });

  const base = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.42, 0.82), frame);
  base.position.y = 0.22;
  const back = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.72, 0.16), frame);
  back.position.set(0, 0.58, -0.33);
  const armA = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.55, 0.82), frame);
  armA.position.set(-0.92, 0.3, 0);
  const armB = armA.clone();
  armB.position.x = 0.92;
  const seatA = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.12, 0.58), cushion);
  seatA.position.set(-0.55, 0.47, 0.08);
  const seatB = seatA.clone();
  seatB.position.x = 0;
  const seatC = seatA.clone();
  seatC.position.x = 0.55;
  group.add(base, back, armA, armB, seatA, seatB, seatC);
  return group;
}

function buildChair() {
  const group = new THREE.Group();
  const seatMat = new THREE.MeshStandardMaterial({ color: '#8fd9c8', roughness: 0.6, metalness: 0.06 });
  const legMat = new THREE.MeshStandardMaterial({ color: '#cad6ea', roughness: 0.35, metalness: 0.15 });

  const seat = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.08, 0.58), seatMat);
  seat.position.y = 0.48;
  const back = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.55, 0.08), seatMat);
  back.position.set(0, 0.76, -0.25);
  group.add(seat, back);

  const legGeometry = new THREE.BoxGeometry(0.06, 0.46, 0.06);
  [
    [0.23, 0.23, 0.23],
    [-0.23, 0.23, 0.23],
    [0.23, 0.23, -0.23],
    [-0.23, 0.23, -0.23]
  ].forEach(([x, y, z]) => {
    const leg = new THREE.Mesh(legGeometry, legMat);
    leg.position.set(x, y, z);
    group.add(leg);
  });

  return group;
}

function buildBed() {
  const group = new THREE.Group();
  const frame = new THREE.MeshStandardMaterial({ color: '#7b89a9', roughness: 0.62, metalness: 0.08 });
  const mattress = new THREE.MeshStandardMaterial({ color: '#f0f5ff', roughness: 0.9, metalness: 0.02 });
  const pillow = new THREE.MeshStandardMaterial({ color: '#c4d7ff', roughness: 0.7, metalness: 0.02 });

  const base = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.28, 1.58), frame);
  base.position.y = 0.18;
  const mattressMesh = new THREE.Mesh(new THREE.BoxGeometry(1.94, 0.2, 1.52), mattress);
  mattressMesh.position.y = 0.42;
  const headboard = new THREE.Mesh(new THREE.BoxGeometry(2.0, 0.96, 0.12), frame);
  headboard.position.set(0, 0.55, -0.73);
  const pillowA = new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.1, 0.36), pillow);
  pillowA.position.set(-0.42, 0.57, -0.35);
  const pillowB = pillowA.clone();
  pillowB.position.x = 0.42;
  group.add(base, mattressMesh, headboard, pillowA, pillowB);
  return group;
}

function buildTable() {
  const group = new THREE.Group();
  const top = new THREE.MeshStandardMaterial({ color: '#d7ae74', roughness: 0.58, metalness: 0.04 });
  const leg = new THREE.MeshStandardMaterial({ color: '#6a7c99', roughness: 0.44, metalness: 0.22 });

  const tableTop = new THREE.Mesh(new THREE.BoxGeometry(1.35, 0.12, 0.86), top);
  tableTop.position.y = 0.82;
  group.add(tableTop);

  const legGeometry = new THREE.BoxGeometry(0.09, 0.8, 0.09);
  [
    [0.55, 0.4, 0.32],
    [-0.55, 0.4, 0.32],
    [0.55, 0.4, -0.32],
    [-0.55, 0.4, -0.32]
  ].forEach(([x, y, z]) => {
    const tableLeg = new THREE.Mesh(legGeometry, leg);
    tableLeg.position.set(x, y, z);
    group.add(tableLeg);
  });

  return group;
}

function buildCabinet() {
  const group = new THREE.Group();
  const body = new THREE.MeshStandardMaterial({ color: '#b88b62', roughness: 0.68, metalness: 0.04 });
  const trim = new THREE.MeshStandardMaterial({ color: '#e1edf8', roughness: 0.28, metalness: 0.18 });

  const shell = new THREE.Mesh(new THREE.BoxGeometry(1.08, 1.82, 0.42), body);
  shell.position.y = 0.91;
  group.add(shell);

  const handleA = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.18, 0.02), trim);
  handleA.position.set(-0.12, 1.0, 0.22);
  const handleB = handleA.clone();
  handleB.position.x = 0.12;
  group.add(handleA, handleB);
  return group;
}

function buildCeilingHazard(rng) {
  const style = rng() > 0.5 ? 'fan' : 'chandelier';
  const radius = style === 'fan' ? 0.72 : 0.46;
  const halfW = sim.room.width * 0.5 - 1.2;
  const halfD = sim.room.depth * 0.5 - 1.2;
  const x = lerp(-halfW, halfW, rng());
  const z = lerp(-halfD, halfD, rng());
  const y = sim.room.height - 0.12;

  const group = new THREE.Group();

  if (style === 'fan') {
    const metal = new THREE.MeshStandardMaterial({ color: '#d8e5ff', roughness: 0.28, metalness: 0.35 });
    const blades = new THREE.MeshStandardMaterial({ color: '#617799', roughness: 0.52, metalness: 0.08 });
    const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.32, 14), metal);
    rod.position.y = y - 0.16;
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.08, 20), metal);
    hub.position.y = y - 0.34;
    group.add(rod, hub);
    for (let i = 0; i < 4; i += 1) {
      const blade = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.025, 0.12), blades);
      blade.position.y = y - 0.34;
      blade.rotation.y = (Math.PI / 2) * i;
      group.add(blade);
    }
  } else {
    const brass = new THREE.MeshStandardMaterial({ color: '#e6cc8f', roughness: 0.35, metalness: 0.32 });
    const glass = new THREE.MeshStandardMaterial({ color: '#d6ecff', roughness: 0.12, metalness: 0.08 });
    const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.42, 14), brass);
    rod.position.y = y - 0.21;
    const core = new THREE.Mesh(new THREE.SphereGeometry(0.18, 18, 16), glass);
    core.position.y = y - 0.5;
    group.add(rod, core);
    for (let i = 0; i < 4; i += 1) {
      const arm = new THREE.Mesh(new THREE.BoxGeometry(0.48, 0.03, 0.03), brass);
      arm.position.y = y - 0.46;
      arm.rotation.y = (Math.PI / 2) * i;
      group.add(arm);
      const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.06, 12, 10), glass);
      bulb.position.set(Math.cos((Math.PI / 2) * i) * 0.26, y - 0.46, Math.sin((Math.PI / 2) * i) * 0.26);
      group.add(bulb);
    }
  }

  group.position.set(x, 0, z);
  return {
    group,
    bottomY: style === 'fan' ? y - 0.38 : y - 0.62,
    radius,
    x,
    z,
    type: style
  };
}

function initializeMapper() {
  const res = clamp(readNumber(els.terrainVoxelResolution), 2, 14);
  const margin = res * 1.5;
  sim.mapper = new VoxelGrid({
    room: sim.room,
    resolution: res,
    margin
  });

  sim.bandVisitCounts = Array(buildSeedHeights().length).fill(0);
  derivePlannerField();
}

function fitCamerasToRoom() {
  const radius = Math.max(sim.room.width, sim.room.depth) * 0.9;
  main.controls.target.set(0, 12, -sim.room.depth * 0.3);
  main.camera.position.set(radius * 0.42, 42, -radius * 0.72);
  main.camera.updateProjectionMatrix();
  main.controls.update();

  updateCloudReferenceGrid();
  cloud.controls.target.set(0, sim.room.height * 0.28, 0);
  cloud.camera.position.set(radius * 0.58, Math.max(sim.room.height * 1.05, 62), radius * 0.7);
  cloud.camera.updateProjectionMatrix();
  cloud.controls.update();
}

function updateCloudReferenceGrid() {
  const size = Math.max(sim.room.width, sim.room.depth);
  const divisions = 48;
  if (cloud.grid) {
    cloud.scene.remove(cloud.grid);
    cloud.grid.geometry.dispose();
    if (Array.isArray(cloud.grid.material)) {
      cloud.grid.material.forEach((material) => material.dispose());
    } else {
      cloud.grid.material.dispose();
    }
  }
  cloud.grid = new THREE.GridHelper(size, divisions, '#5176af', '#1e2f49');
  cloud.scene.add(cloud.grid);
}

function buildSeedHeights() {
  const levels = 4;
  const safety = clamp(readNumber(els.safetyRadius), 0.2, 1.4);
  const minY = Math.max(0.65, safety + 0.2);
  let maxY = sim.room.height - safety - 0.28;
  if (sim.ceilingHazard) {
    maxY = Math.min(maxY, sim.ceilingHazard.bottomY - safety - 0.18);
  }
  maxY = Math.max(maxY, minY + 0.35);
  return Array.from({ length: levels }, (_, index) => {
    if (levels === 1) {
      return minY;
    }
    return lerp(minY, maxY, index / (levels - 1));
  });
}

function buildBootstrapPlan() {
  const sensor = currentSensorConfig();
  const heights = buildSeedHeights();
  const count = 6;
  const yaws = Array.from({ length: count }, (_, index) => (index / count) * Math.PI * 2);
  const plan = [];

  heights.forEach((height, bandIndex) => {
    yaws.forEach((yaw) => {
      plan.push({
        position: new THREE.Vector3(0, height, 0),
        yaw,
        bandIndex
      });
    });
  });

  if (sensor.mode === 'spinning') {
    return plan;
  }

  return plan.filter((_, index) => index % 2 === 0);
}

function resetMission(writeLog, clearLogs) {
  finalizeSwarmTelemetry('reset');
  sim.droneActive = false;
  sim.status = 'idle';
  sim.complete = false;
  sim.missionStarted = false;
  sim.scanCounter = 0;
  sim.pointCount = 0;
  sim.currentPhase = 'idle';
  sim.bootstrapPlan = [];
  sim.bootstrapIndex = 0;
  sim.routeSamples = [];
  sim.routeSampleIndex = 0;
  sim.currentGoal = null;
  sim.currentGoalType = 'frontier';
  sim.currentGoalWorld = null;
  sim.currentBandIndex = 0;
  sim.holdElapsedMs = 0;
  sim.holdScanElapsedMs = 0;
  sim.moveScanElapsedMs = 0;
  sim.blockedCooldownMs = 0;
  sim.plannerStats = {
    goalsReached: 0,
    replans: 0,
    lastPathLength: 0
  };

  initializeMapper();
  clearPointCloud();
  main.pathGroup.clear();
  main.beamGroup.clear();

  const startY = buildSeedHeights()[0] || 1.1;
  main.drone.position.set(0, startY, 0);
  orientDroneYaw(0);

  if (clearLogs) {
    sim.logs = [];
    els.logWindow.innerHTML = '';
  }

  refreshStatus();
  drawMissionGraph();

  if (writeLog) {
    logMessage('Mission reset. 3D occupancy map and point cloud cleared.');
  }

  syncSwarmMode();
}

function clearPointCloud() {
  sim.voxelMap.clear();
  sim.footprintCoverageMap.clear();
  sim.footprintMetrics = {
    redundancyRatio: 0,
    resolutionScore: 0,
    uniqueAreaRate: 0
  };
  sim.pointCount = 0;
  sim.cloudDirty = false;
  sim.performance.displayedPointCount = 0;
  sim.performance.lastCloudFlushMs = 0;
  sim.pointCloudBuffer.positions = new Float32Array(0);
  sim.pointCloudBuffer.colors = new Float32Array(0);
  sim.pointCloudBuffer.capacity = 0;
  sim.performance.cloudBufferCapacity = 0;
  sim.cloudGeometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
  sim.cloudGeometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
  sim.cloudGeometry.setDrawRange(0, 0);
  sim.cloudGeometry.computeBoundingSphere();
  sim.cloudMaterial.size = Math.max(0.028, clamp(readNumber(els.voxelSize), 0.015, 0.2) * 1.18);
  updatePointReadouts();
}

function startMission() {
  if (isSwarmMode()) {
    clearPointCloud();
    initializeMapper();
    sim.droneActive = true;
    sim.missionStarted = true;
    sim.complete = false;
    sim.status = 'running';
    sim.currentPhase = 'swarm';
    sim.scanCounter = 0;
    sim.pointCount = 0;
    sim.swarmLaunchElapsedMs = 0;
    sim.swarmTelemetryElapsedMs = 0;
    sim.swarmScanElapsedMs = 0;
    sim.commsRenderElapsedMs = 0;
    sim.cloudFlushElapsedMs = 0;
    sim.graphRenderElapsedMs = 0;
    main.pathGroup.clear();
    main.beamGroup.clear();
    initializeSwarm();
    startSwarmTelemetryRun();
    refreshStatus();
    drawMissionGraph();
    logMessage(
      `Swarm V1 preview started with ${sim.swarmController.agents.length} drones in ${readableFormation(els.swarmFormation.value)} mode.`
    );
    return;
  }

  clearPointCloud();
  initializeMapper();
  sim.droneActive = true;
  sim.missionStarted = true;
  sim.complete = false;
  sim.status = 'running';
  sim.currentPhase = 'bootstrap';
  sim.bootstrapPlan = buildBootstrapPlan();
  sim.bootstrapIndex = 0;
  sim.routeSamples = [];
  sim.routeSampleIndex = 0;
  sim.currentGoal = null;
  sim.currentGoalWorld = null;
  sim.currentGoalType = 'frontier';
  sim.holdElapsedMs = 0;
  sim.holdScanElapsedMs = 0;
  sim.moveScanElapsedMs = 0;
  sim.blockedCooldownMs = 0;
  main.pathGroup.clear();
  main.beamGroup.clear();

  if (sim.bootstrapPlan.length) {
    const first = sim.bootstrapPlan[0];
    main.drone.position.copy(first.position);
    orientDroneYaw(first.yaw);
  } else {
    main.drone.position.set(0, buildSeedHeights()[0] || 1.1, 0);
    orientDroneYaw(0);
  }

  refreshStatus();
  drawMissionGraph();
  logMessage(
    `Mission started with ${SENSOR_PRESETS[els.sensorPreset.value].label}. The drone will bootstrap, build a 3D voxel map, then plan by information gain.`
  );
}

function pauseMission() {
  if (!sim.missionStarted || sim.status !== 'running') {
    return;
  }
  sim.droneActive = false;
  sim.status = 'paused';
  refreshStatus();
  logMessage('Mission paused. Telemetry remains open and will save on Reset or completion.');
}

function continueMission() {
  if (!sim.missionStarted || sim.complete) {
    return;
  }
  sim.droneActive = true;
  sim.status = 'running';
  refreshStatus();
  logMessage('Mission resumed.');
}

function stopMission(reason = 'stopped') {
  if (!sim.missionStarted || sim.complete) {
    return;
  }
  sim.droneActive = false;
  sim.complete = true;
  sim.status = 'complete';
  sim.currentPhase = 'complete';
  finalizeSwarmTelemetry(reason);
  refreshStatus();
  drawMissionGraph();
  logMessage(reason === 'cancelled_batch'
    ? 'Policy batch run cancelled and saved with validity flags.'
    : 'Mission stopped. Telemetry saved; point cloud remains available for inspection and export.');
}

function togglePolicyExperimentBatch() {
  if (sim.policyExperiment.active) {
    const preset = sim.policyExperiment.currentPreset ? { ...sim.policyExperiment.currentPreset } : null;
    sim.policyExperiment.active = false;
    sim.policyExperiment.waitingForNext = false;
    sim.policyExperiment.currentPreset = null;
    sim.policyExperiment.runPlan = [];
    updatePolicyExperimentButton();
    if (preset && sim.missionStarted && !sim.complete) {
      stopMission('cancelled_batch');
    }
    refreshStatus();
    logMessage('Policy batch cancelled. Current run saved with batch_cancelled validity flag.');
    return;
  }

  sim.policyExperiment.active = true;
  sim.policyExperiment.runMs = clamp(readNumber(els.policyExperimentSeconds), 30, 900) * 1000;
  sim.policyExperiment.currentIndex = 0;
  sim.policyExperiment.runPlan = buildPolicyExperimentRunPlan(els.policyExperimentMode.value);
  sim.policyExperiment.waitingForNext = false;
  sim.policyExperiment.batchId = `policy-batch-${els.policyExperimentMode.value}-${new Date().toISOString()}`;
  updatePolicyExperimentButton();
  refreshStatus();
  logMessage(`Policy batch started: ${sim.policyExperiment.runPlan.length} ${policyExperimentModeLabel(els.policyExperimentMode.value)} runs at ${Math.round(sim.policyExperiment.runMs / 1000)}s each.`);
  runNextPolicyExperimentPreset();
}

function buildPolicyExperimentRunPlan(mode = 'nudge_calibration') {
  const nudgeProfiles = mode === 'pareto_only'
    ? [els.runtimeNudgeProfile.value ?? 'current']
    : POLICY_EXPERIMENT_NUDGE_PROFILES;
  return POLICY_EXPERIMENT_PRESETS.flatMap((preset) => nudgeProfiles.map((nudgeProfile) => ({
    ...preset,
    batchMode: mode,
    batchModeLabel: policyExperimentModeLabel(mode),
    nudgeProfile,
    nudgeProfileLabel: runtimeNudgeProfileConfig(nudgeProfile).label,
    nudgeScale: runtimeNudgeProfileConfig(nudgeProfile).scale
  })));
}

function policyExperimentModeLabel(mode) {
  return mode === 'pareto_only' ? 'Pareto-only fixed-nudge' : 'Nudge-calibration';
}

function updatePolicyExperimentButton() {
  if (!els.policyExperimentBtn) {
    return;
  }
  els.policyExperimentBtn.textContent = sim.policyExperiment.active ? 'Cancel Policy Batch' : 'Run Policy Batch';
}

function runNextPolicyExperimentPreset() {
  if (!sim.policyExperiment.active) {
    return;
  }
  if (sim.policyExperiment.currentIndex >= sim.policyExperiment.runPlan.length) {
    sim.policyExperiment.active = false;
    sim.policyExperiment.waitingForNext = false;
    sim.policyExperiment.currentPreset = null;
    sim.policyExperiment.runPlan = [];
    updatePolicyExperimentButton();
    refreshStatus();
    refreshTelemetryPanel();
    logMessage('Policy batch complete. Telemetry saved for each preset run.');
    return;
  }

  const preset = sim.policyExperiment.runPlan[sim.policyExperiment.currentIndex];
  sim.policyExperiment.currentPreset = {
    ...preset,
    index: sim.policyExperiment.currentIndex + 1,
    total: sim.policyExperiment.runPlan.length,
    runSeconds: Math.round(sim.policyExperiment.runMs / 1000),
    batchId: sim.policyExperiment.batchId
  };
  sim.policyExperiment.waitingForNext = false;
  resetMission(false, false);
  applyPolicyCoordinatesToControls(preset.policyCoordinates);
  els.runtimeNudgeProfile.value = preset.nudgeProfile;
  applyRuntimeNudgeProfileChange(`policy-experiment-nudge:${preset.nudgeProfile}`);
  applyPolicyCoordinateChange(`policy-experiment:${preset.id}`);
  startMission();
  logMessage(`Policy batch ${sim.policyExperiment.currentPreset.index}/${sim.policyExperiment.currentPreset.total}: ${preset.label}, ${preset.nudgeProfileLabel} nudge.`);
}

function completeCurrentPolicyExperimentPreset() {
  if (!sim.policyExperiment.active || sim.policyExperiment.waitingForNext) {
    return;
  }
  sim.policyExperiment.waitingForNext = true;
  const preset = sim.policyExperiment.currentPreset;
  stopMission();
  sim.policyExperiment.currentIndex += 1;
  logMessage(`Policy batch run saved: ${preset?.label ?? 'preset'}.`);
  window.setTimeout(() => runNextPolicyExperimentPreset(), 700);
}

function animate(time) {
  requestAnimationFrame(animate);

  const deltaMs = sim.lastTime ? time - sim.lastTime : 16;
  sim.lastTime = time;
  const deltaSeconds = deltaMs / 1000;

  updatePerformanceBudget(deltaMs);
  updateKeyboardNavigation(deltaSeconds);
  updateDrone(deltaSeconds, deltaMs);
  updateSwarmPreview(deltaSeconds);
  main.controls.update();
  cloud.controls.update();

  main.renderer.render(main.scene, main.camera);
  cloud.renderer.render(cloud.scene, cloud.camera);

  const budget = readPerformanceBudgetConfig();
  sim.graphRenderElapsedMs += deltaMs;
  if (sim.graphRenderElapsedMs > budgetedInterval(budget.graphRenderMs)) {
    sim.graphRenderElapsedMs = 0;
    drawMissionGraph();
    refreshStatus();
  }
}

function updateKeyboardNavigation(deltaSeconds) {
  if (deltaSeconds <= 0) {
    return;
  }

  const viewportName = sim.navigation.activeViewport === 'cloud' ? 'cloud' : 'main';
  const targetScene = viewportName === 'cloud' ? cloud : main;
  const velocity = sim.navigation.velocity[viewportName];
  const speed = sim.navigation.keys.has('shift') ? 52 : 28;
  const { camera, controls } = targetScene;
  camera.getWorldDirection(temp.cameraForward);
  temp.cameraForward.normalize();
  temp.cameraRight.setFromMatrixColumn(camera.matrixWorld, 0).normalize();

  const desired = new THREE.Vector3();
  if (sim.navigation.keys.has('w')) desired.add(temp.cameraForward);
  if (sim.navigation.keys.has('s')) desired.sub(temp.cameraForward);
  if (sim.navigation.keys.has('d')) desired.add(temp.cameraRight);
  if (sim.navigation.keys.has('a')) desired.sub(temp.cameraRight);
  if (sim.navigation.keys.has('e')) desired.y += 1;
  if (sim.navigation.keys.has('q')) desired.y -= 1;

  if (desired.lengthSq() > 1e-6) {
    desired.normalize().multiplyScalar(speed);
    velocity.lerp(desired, 1 - Math.exp(-deltaSeconds * 9.5));
  } else {
    velocity.multiplyScalar(Math.exp(-deltaSeconds * 8));
  }

  if (velocity.lengthSq() < 1e-5) {
    velocity.set(0, 0, 0);
    return;
  }

  const move = velocity.clone().multiplyScalar(deltaSeconds);
  camera.position.add(move);
  controls.target.add(move);
}

function setupFreeLookControls(element, viewportName) {
  element.addEventListener('pointerdown', (event) => {
    if (event.button !== 0) {
      return;
    }
    sim.navigation.activeViewport = viewportName;
    sim.navigation.freeLook.active = true;
    sim.navigation.freeLook.viewport = viewportName;
    sim.navigation.freeLook.lastX = event.clientX;
    sim.navigation.freeLook.lastY = event.clientY;
    const targetScene = viewportName === 'cloud' ? cloud : main;
    targetScene.controls.enabled = false;
    element.focus?.({ preventScroll: true });
    element.setPointerCapture(event.pointerId);
    event.preventDefault();
  });

  element.addEventListener('pointermove', (event) => {
    if (!sim.navigation.freeLook.active || sim.navigation.freeLook.viewport !== viewportName) {
      return;
    }
    rotateViewFromPointerDelta(event.clientX - sim.navigation.freeLook.lastX, event.clientY - sim.navigation.freeLook.lastY, viewportName);
    sim.navigation.freeLook.lastX = event.clientX;
    sim.navigation.freeLook.lastY = event.clientY;
    event.preventDefault();
  });

  const stopFreeLook = (event) => {
    if (!sim.navigation.freeLook.active || sim.navigation.freeLook.viewport !== viewportName) {
      return;
    }
    sim.navigation.freeLook.active = false;
    const targetScene = viewportName === 'cloud' ? cloud : main;
    targetScene.controls.enabled = true;
    if (event.pointerId !== undefined && element.hasPointerCapture(event.pointerId)) {
      element.releasePointerCapture(event.pointerId);
    }
  };

  element.addEventListener('pointerup', stopFreeLook);
  element.addEventListener('pointercancel', stopFreeLook);
  element.addEventListener('pointerleave', stopFreeLook);
}

function rotateViewFromPointerDelta(deltaX, deltaY, viewportName) {
  const targetScene = viewportName === 'cloud' ? cloud : main;
  const { camera, controls } = targetScene;
  const offset = controls.target.clone().sub(camera.position);
  const distance = Math.max(offset.length(), 0.001);
  const spherical = new THREE.Spherical().setFromVector3(offset);
  spherical.theta -= deltaX * 0.0035;
  spherical.phi = clamp(spherical.phi + deltaY * 0.0035, 0.05, Math.PI - 0.05);
  offset.setFromSpherical(spherical).setLength(distance);
  controls.target.copy(camera.position).add(offset);
  controls.update();
}

function updateDrone(deltaSeconds, deltaMs) {
  return;

  if (!sim.droneActive) {
    drawMissionGraph();
    return;
  }

  sim.blockedCooldownMs = Math.max(0, sim.blockedCooldownMs - deltaMs);

  if (sim.currentPhase === 'bootstrap') {
    updateBootstrap(deltaMs);
    refreshStatus();
    drawMissionGraph();
    return;
  }

  if (!sim.currentGoal && !selectNextGoal()) {
    finishMission();
    return;
  }

  updateGoalMotion(deltaSeconds, deltaMs);
  refreshStatus();
  drawMissionGraph();
}

function updateBootstrap(deltaMs) {
  if (sim.bootstrapIndex >= sim.bootstrapPlan.length) {
    sim.currentPhase = 'frontier';
    sim.holdScanElapsedMs = 0;
    sim.moveScanElapsedMs = 0;
    logMessage('Bootstrap complete. Switching to 3D frontier exploration.');
    return;
  }

  const step = sim.bootstrapPlan[sim.bootstrapIndex];
  main.drone.position.copy(step.position);
  orientDroneYaw(step.yaw);
  sim.currentBandIndex = step.bandIndex;

  sim.holdScanElapsedMs += deltaMs;
  if (sim.holdScanElapsedMs >= holdScanIntervalMs()) {
    performSensorScan('bootstrap');
    sim.holdScanElapsedMs = 0;
    sim.bootstrapIndex += 1;
  }
}

function selectNextGoal() {
  derivePlannerField();

  const frontierCandidates = buildGoalCandidates(sim.mapper.frontierIndices, 'frontier');
  const inspectionCandidates = frontierCandidates.length ? [] : buildGoalCandidates(sim.mapper.inspectionIndices, 'inspect');
  const candidates = frontierCandidates.length ? frontierCandidates : inspectionCandidates;
  if (!candidates.length) {
    return false;
  }

  const startCell = worldToVoxel(main.drone.position.x, main.drone.position.y, main.drone.position.z);
  let best = null;

  candidates.slice(0, 18).forEach((candidate) => {
    const result = runVoxelPlanner(sim.mapper, startCell, candidate.cell, els.plannerMode.value);
    if (!result.path.length) {
      return;
    }

    const travelCost = result.cost;
    const score =
      candidate.infoGain * 1.45 +
      candidate.clearance * 0.55 -
      travelCost * 0.9 -
      candidate.revisitPenalty * 1.35 -
      candidate.altitudePenalty * 0.5;

    if (!best || score > best.score) {
      best = {
        score,
        candidate,
        path: result.path,
        cost: travelCost
      };
    }
  });

  if (!best) {
    return false;
  }

  sim.currentGoal = best.candidate.cell;
  sim.currentGoalType = best.candidate.type;
  sim.currentGoalWorld = voxelToWorld(best.candidate.cell.col, best.candidate.cell.row, best.candidate.cell.layer);
  sim.currentBandIndex = best.candidate.bandIndex;
  sim.routeSamples = buildRouteSamples(main.drone.position.clone(), sim.currentGoalWorld.clone(), best.path);
  sim.routeSampleIndex = 0;
  sim.holdElapsedMs = 0;
  sim.holdScanElapsedMs = 0;
  sim.moveScanElapsedMs = 0;
  sim.currentPhase = best.candidate.type === 'frontier' ? 'frontier' : 'inspect';
  sim.plannerStats.lastPathLength = best.path.length;
  renderRoutePreview();
  logMessage(
    `New ${best.candidate.type} goal picked with info gain ${best.candidate.infoGain.toFixed(1)} and path length ${best.path.length}.`
  );
  return true;
}

function buildGoalCandidates(indices, type) {
  const heights = buildSeedHeights();
  return indices
    .map((index) => {
      const cell = unpackIndex(index);
      const clearance = sim.mapper.clearance[index];
      const infoGain = estimateInformationGain(cell, type === 'frontier' ? 2 : 1);
      const visits = sim.mapper.visits[index];
      const bandIndex = nearestSeedBand(voxelToWorld(cell.col, cell.row, cell.layer).y, heights);
      return {
        type,
        cell,
        infoGain,
        clearance,
        revisitPenalty: visits + sim.bandVisitCounts[bandIndex] * 0.6,
        altitudePenalty: Math.abs(bandIndex - sim.currentBandIndex),
        bandIndex
      };
    })
    .sort((a, b) => b.infoGain + b.clearance - a.revisitPenalty - (a.altitudePenalty * 0.2) - (a.clearance + a.infoGain - b.revisitPenalty - (b.altitudePenalty * 0.2)));
}

function estimateInformationGain(cell, radius) {
  let unknown = 0;
  let occupied = 0;
  for (let dz = -radius; dz <= radius; dz += 1) {
    for (let dy = -radius; dy <= radius; dy += 1) {
      for (let dx = -radius; dx <= radius; dx += 1) {
        const neighbor = getCellState(cell.col + dx, cell.row + dy, cell.layer + dz);
        if (neighbor === CELL_UNKNOWN) {
          unknown += 1;
        } else if (neighbor === CELL_OCCUPIED) {
          occupied += 0.4;
        }
      }
    }
  }
  return unknown + occupied;
}

function updateGoalMotion(deltaSeconds, deltaMs) {
  if (!sim.currentGoalWorld) {
    return;
  }

  const goalReach = 0.28;

  if (sim.routeSampleIndex < sim.routeSamples.length) {
    const target = sim.routeSamples[sim.routeSampleIndex];
    const stepLength = clamp(readNumber(els.moveSpeed), 0.2, 4) * deltaSeconds;

    if (!isMotionSafe(target, stepLength) && sim.blockedCooldownMs === 0) {
      sim.blockedCooldownMs = 250;
      sim.plannerStats.replans += 1;
      performSensorScan('avoidance');
      sim.currentGoal = null;
      sim.currentGoalWorld = null;
      sim.routeSamples = [];
      sim.routeSampleIndex = 0;
      main.pathGroup.clear();
      sim.currentPhase = 'avoid';
      logMessage('Clearance sphere detected a collision risk. Stopping and replanning from live voxels.');
      return;
    }

    temp.travel.copy(target).sub(main.drone.position);
    const distance = temp.travel.length();
    if (distance > stepLength) {
      temp.travel.normalize().multiplyScalar(stepLength);
      main.drone.position.add(temp.travel);
    } else {
      main.drone.position.copy(target);
      sim.routeSampleIndex += 1;
    }

    orientAlongRoute();
    sim.moveScanElapsedMs += deltaMs;
    if (sim.moveScanElapsedMs >= moveScanIntervalMs()) {
      performSensorScan('move');
      sim.moveScanElapsedMs = 0;
    }
  } else {
    orientTowardGoal();
    sim.holdElapsedMs += deltaMs;
    sim.holdScanElapsedMs += deltaMs;

    if (sim.holdScanElapsedMs >= holdScanIntervalMs()) {
      performSensorScan('goal');
      sim.holdScanElapsedMs = 0;
    }

    if (main.drone.position.distanceTo(sim.currentGoalWorld) <= goalReach || sim.holdElapsedMs >= clamp(readNumber(els.dwellMs), 120, 2200)) {
      const index = packIndex(sim.currentGoal.col, sim.currentGoal.row, sim.currentGoal.layer);
      sim.mapper.visits[index] += 1;
      sim.bandVisitCounts[sim.currentBandIndex] += 1;
      sim.plannerStats.goalsReached += 1;
      sim.currentGoal = null;
      sim.currentGoalWorld = null;
      sim.routeSamples = [];
      sim.routeSampleIndex = 0;
      sim.holdElapsedMs = 0;
      sim.holdScanElapsedMs = 0;
      renderRoutePreview();
    }
  }
}

function moveScanIntervalMs() {
  const base = clamp(readNumber(els.dwellMs), 120, 2200);
  return els.scanMode.value === 'continuous' ? Math.max(110, base * 0.42) : Math.max(180, base * 0.72);
}

function holdScanIntervalMs() {
  const base = clamp(readNumber(els.dwellMs), 120, 2200);
  return els.scanMode.value === 'continuous' ? Math.max(100, base * 0.55) : base;
}

function isMotionSafe(target, stepLength) {
  const safetyRadius = clamp(readNumber(els.safetyRadius), 0.2, 1.4);
  const nextPos = main.drone.position.clone().lerp(target, Math.min(1, stepLength / Math.max(main.drone.position.distanceTo(target), 1e-6)));
  const roomCeiling = sim.room.height - safetyRadius - 0.16;
  const roomFloor = safetyRadius + 0.18;

  if (nextPos.y > roomCeiling || nextPos.y < roomFloor) {
    return false;
  }

  temp.travel.copy(target).sub(main.drone.position);
  if (temp.travel.lengthSq() < 1e-6) {
    return true;
  }

  const forward = temp.travel.clone().normalize();
  const lookAhead = stepLength + safetyRadius + 0.18;
  const fallbackUp = Math.abs(forward.dot(new THREE.Vector3(0, 1, 0))) > 0.95 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(0, 1, 0);
  temp.right.crossVectors(forward, fallbackUp).normalize();
  temp.up.crossVectors(temp.right, forward).normalize();

  const offsets = [
    [0, 0],
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
    [0.7, 0.7],
    [-0.7, 0.7],
    [0.7, -0.7],
    [-0.7, -0.7]
  ];

  for (const [side, lift] of offsets) {
    temp.probeOrigin
      .copy(main.drone.position)
      .addScaledVector(temp.right, side * safetyRadius * 0.55)
      .addScaledVector(temp.up, lift * safetyRadius * 0.55);

    raycaster.set(temp.probeOrigin, forward);
    raycaster.near = 0.02;
    raycaster.far = lookAhead;
    const intersections = raycaster.intersectObjects(sim.scanTargets, true);
    if (intersections.length && intersections[0].distance <= lookAhead) {
      updateMapperAlongRay(temp.probeOrigin, forward, intersections[0].distance, lookAhead, true);
      return false;
    }
  }

  return true;
}

function performSensorScan(scanContext) {
  const sensor = currentSensorConfig();
  sim.cloudMaterial.size = Math.max(0.028, clamp(readNumber(els.voxelSize), 0.015, 0.2) * 1.18);

  temp.source.copy(main.drone.position);
  const forward = new THREE.Vector3();
  main.drone.getWorldDirection(forward);
  const centerYaw = Math.atan2(forward.z, forward.x);

  let hits = 0;
  const scanBeams = [];
  const beamStride = Math.max(1, Math.floor((sensor.horizontalRays * sensor.verticalRays) / 18));
  let beamIndex = 0;

  for (let v = 0; v < sensor.verticalRays; v += 1) {
    const verticalT = sensor.verticalRays === 1 ? 0.5 : v / (sensor.verticalRays - 1);
    const elevation = sensor.pitchBiasRad + lerp(-THREE.MathUtils.degToRad(sensor.verticalFov) * 0.5, THREE.MathUtils.degToRad(sensor.verticalFov) * 0.5, verticalT);

    for (let h = 0; h < sensor.horizontalRays; h += 1) {
      const horizontalT = sensor.horizontalRays === 1 ? 0.5 : h / (sensor.horizontalRays - 1);
      const yaw =
        sensor.mode === 'spinning'
          ? (horizontalT * Math.PI * 2) + centerYaw
          : centerYaw + lerp(-THREE.MathUtils.degToRad(sensor.horizontalFov) * 0.5, THREE.MathUtils.degToRad(sensor.horizontalFov) * 0.5, horizontalT);

      temp.dir.set(
        Math.cos(elevation) * Math.cos(yaw),
        Math.sin(elevation),
        Math.cos(elevation) * Math.sin(yaw)
      );

      raycaster.set(temp.source, temp.dir);
      raycaster.near = 0.05;
      raycaster.far = sensor.maxRange;
      const intersections = raycaster.intersectObjects(sim.scanTargets, true);

      if (!intersections.length) {
        updateMapperAlongRay(temp.source, temp.dir, sensor.maxRange, sensor.maxRange, false);
        beamIndex += 1;
        continue;
      }

      const hit = intersections[0];
      updateMapperAlongRay(temp.source, temp.dir, hit.distance, sensor.maxRange, true);
      addPointToCloud(hit.point, hit.distance, {}, hit);
      hits += 1;

      if (beamIndex % beamStride === 0) {
        scanBeams.push([temp.source.clone(), hit.point.clone()]);
      }
      beamIndex += 1;
    }
  }

  derivePlannerField();
  sim.scanCounter += 1;
  renderScanBeams(scanBeams, scanContext);
  flushPointCloudGeometry(sensor.maxRange);
  logScanEvent(hits, scanContext);
}

function updateMapperAlongRay(source, direction, travelDistance, maxRange, hadHit) {
  const step = Math.max(sim.mapper.res * 0.45, 0.08);
  const freeUntil = hadHit ? Math.max(travelDistance - sim.mapper.res * 0.45, 0) : travelDistance;
  for (let t = 0; t <= freeUntil; t += step) {
    markWorldVoxel(
      source.x + direction.x * t,
      source.y + direction.y * t,
      source.z + direction.z * t,
      CELL_FREE
    );
  }

  if (hadHit) {
    markWorldVoxel(
      source.x + direction.x * Math.min(travelDistance, maxRange),
      source.y + direction.y * Math.min(travelDistance, maxRange),
      source.z + direction.z * Math.min(travelDistance, maxRange),
      CELL_OCCUPIED
    );
  }
}

function markWorldVoxel(x, y, z, state) {
  sim.mapper.markWorld(x, y, z, state);
}

function markVoxelState(col, row, layer, state) {
  sim.mapper.markState(col, row, layer, state);
}

function derivePlannerField() {
  if (!sim.mapper) {
    return;
  }

  const radius = clamp(readNumber(els.safetyRadius), 0.2, 1.4);
  const radiusCells = Math.max(1, Math.ceil(radius / sim.mapper.res));
  sim.mapper.frontierIndices = [];
  sim.mapper.inspectionIndices = [];

  for (let layer = 0; layer < sim.mapper.layers; layer += 1) {
    for (let row = 0; row < sim.mapper.rows; row += 1) {
      for (let col = 0; col < sim.mapper.cols; col += 1) {
        const index = packIndex(col, row, layer);
        const state = sim.mapper.states[index];
        if (state !== CELL_FREE) {
          sim.mapper.traversable[index] = 0;
          sim.mapper.clearance[index] = 0;
          continue;
        }

        const world = voxelToWorld(col, row, layer);
        let minClearance = boundaryClearance(world);
        let occupiedNeighbors = 0;
        let unknownNeighbors = 0;
        let blocked = false;

        for (let dz = -radiusCells - 1; dz <= radiusCells + 1; dz += 1) {
          for (let dy = -radiusCells - 1; dy <= radiusCells + 1; dy += 1) {
            for (let dx = -radiusCells - 1; dx <= radiusCells + 1; dx += 1) {
              if (dx === 0 && dy === 0 && dz === 0) {
                continue;
              }
              const nCol = col + dx;
              const nRow = row + dy;
              const nLayer = layer + dz;
              const distance = Math.hypot(dx, dy, dz) * sim.mapper.res;
              if (!voxelInBounds(nCol, nRow, nLayer)) {
                minClearance = Math.min(minClearance, distance);
                if (distance <= radius) {
                  blocked = true;
                }
                continue;
              }
              const neighborState = sim.mapper.states[packIndex(nCol, nRow, nLayer)];
              if (neighborState === CELL_OCCUPIED) {
                occupiedNeighbors += 1;
                minClearance = Math.min(minClearance, distance);
                if (distance <= radius) {
                  blocked = true;
                }
              } else if (neighborState === CELL_UNKNOWN && Math.abs(dx) + Math.abs(dy) + Math.abs(dz) === 1) {
                unknownNeighbors += 1;
              }
            }
          }
        }

        sim.mapper.clearance[index] = minClearance;
        sim.mapper.traversable[index] = blocked || minClearance < radius ? 0 : 1;

        if (!sim.mapper.traversable[index]) {
          continue;
        }

        if (unknownNeighbors > 0) {
          sim.mapper.frontierIndices.push(index);
        } else if (occupiedNeighbors > 0 && sim.mapper.visits[index] < 3) {
          sim.mapper.inspectionIndices.push(index);
        }
      }
    }
  }
}

function boundaryClearance(world) {
  const distances = [
    world.x + sim.room.width * 0.5,
    sim.room.width * 0.5 - world.x,
    world.z + sim.room.depth * 0.5,
    sim.room.depth * 0.5 - world.z,
    world.y,
    sim.room.height - world.y
  ];

  if (sim.ceilingHazard) {
    const lateral = Math.hypot(world.x - sim.ceilingHazard.x, world.z - sim.ceilingHazard.z) - sim.ceilingHazard.radius;
    distances.push(lateral);
    distances.push(sim.ceilingHazard.bottomY - world.y);
  }

  return Math.max(0.01, Math.min(...distances));
}

function buildRouteSamples(start, end, path) {
  if (!path.length) {
    return [end.clone()];
  }

  const points = [start.clone(), ...path.map((cell) => voxelToWorld(cell.col, cell.row, cell.layer)), end.clone()];
  const simplified = simplifyRoute(points);
  if (els.pathStyle.value === 'manhattan') {
    return simplified.slice(1);
  }

  if (simplified.length < 3) {
    return simplified.slice(1);
  }

  const curve = new THREE.CatmullRomCurve3(simplified, false, 'centripetal');
  const samples = curve.getPoints(Math.max(12, simplified.length * (els.pathStyle.value === 'curved' ? 6 : 4)));
  return samples.slice(1);
}

function simplifyRoute(points) {
  if (points.length <= 2) {
    return points;
  }
  const simplified = [points[0]];
  for (let index = 1; index < points.length - 1; index += 1) {
    const prev = simplified[simplified.length - 1];
    const current = points[index];
    const next = points[index + 1];
    const dirA = next.clone().sub(current).normalize();
    const dirB = current.clone().sub(prev).normalize();
    if (dirA.distanceTo(dirB) > 1e-3) {
      simplified.push(current);
    }
  }
  simplified.push(points[points.length - 1]);
  return simplified;
}

function renderRoutePreview() {
  main.pathGroup.clear();
  if (!sim.routeSamples.length) {
    return;
  }

  const geometry = new THREE.BufferGeometry().setFromPoints([main.drone.position.clone(), ...sim.routeSamples]);
  const line = new THREE.Line(
    geometry,
    new THREE.LineBasicMaterial({ color: '#487cf0', transparent: true, opacity: 0.52 })
  );
  main.pathGroup.add(line);

  const goalMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.09, 12, 12),
    new THREE.MeshBasicMaterial({ color: sim.currentGoalType === 'frontier' ? '#ffd26f' : '#7dffd6' })
  );
  goalMarker.position.copy(sim.currentGoalWorld);
  main.pathGroup.add(goalMarker);
}

function orientDroneYaw(yaw) {
  temp.heading.copy(main.drone.position).add(new THREE.Vector3(Math.cos(yaw), 0, Math.sin(yaw)));
  main.drone.lookAt(temp.heading);
}

function orientAlongRoute() {
  const look = sim.routeSamples[Math.min(sim.routeSampleIndex, sim.routeSamples.length - 1)] || sim.currentGoalWorld;
  if (!look) {
    return;
  }
  main.drone.lookAt(look);
}

function orientTowardGoal() {
  if (!sim.currentGoalWorld) {
    return;
  }
  main.drone.lookAt(sim.currentGoalWorld);
}

function finishMission() {
  if (sim.complete) {
    return;
  }
  sim.droneActive = false;
  finalizeSwarmTelemetry('complete');
  sim.complete = true;
  sim.status = 'complete';
  sim.currentPhase = 'complete';
  sim.currentGoal = null;
  sim.currentGoalWorld = null;
  sim.routeSamples = [];
  sim.routeSampleIndex = 0;
  main.pathGroup.clear();
  refreshStatus();
  drawMissionGraph();
  logMessage(
    `Mission complete. Captured ${sim.pointCount.toLocaleString()} points across ${sim.scanCounter} scans with ${sim.plannerStats.replans} replans.`
  );
}

function syncSwarmMode() {
  const enabled = isSwarmMode();
  main.drone.visible = !enabled;
  main.swarmGroup.visible = enabled;
  main.commGroup.visible = enabled;

  if (enabled) {
    if (sim.missionStarted && sim.swarmController) {
      return;
    }
    initializeSwarm();
    logMessage('Swarm V1 preview enabled. Single-drone planner remains preserved as the baseline mode.');
  } else {
    main.swarmGroup.clear();
    main.commGroup.clear();
    sim.swarmController = null;
    sim.swarmTargets = [];
    sim.swarmSnapshot = null;
  }
}

function initializeSwarm() {
  const config = readSwarmConfig();
  main.swarmGroup.clear();
  main.commGroup.clear();
  sim.swarmController = new SwarmController(config);
  const launchZ = -sim.room.depth * 0.42;
  const origin = new THREE.Vector3(0, terrainClearanceY(0, launchZ, 0.28), launchZ);
  sim.swarmController.initializeAgents({
    count: config.droneCount,
    origin,
    createMesh: (index, role) => {
      const mesh = createSwarmDroneMesh(role, index);
      main.swarmGroup.add(mesh);
      return mesh;
    }
  });
  sim.swarmController.agents.forEach((agent) => {
    const grounded = clampSwarmPosition(agent.position);
    agent.setPosition(grounded);
    agent.launchPosition.copy(grounded);
  });
  updateSwarmPreview(0);
}

function updateSwarmPreview(deltaSeconds) {
  if (!isSwarmMode() || !sim.swarmController) {
    return;
  }

  const budget = readPerformanceBudgetConfig();

  if (!sim.missionStarted || sim.status !== 'running') {
    sim.commsRenderElapsedMs += deltaSeconds * 1000;
    const idleSnapshot = sim.swarmController.update({
      requestedFormation: els.swarmFormation.value,
      environmentSignals: sim.terrainProfile ?? {},
      frontiers: [],
      aois: selectedAoiTargets(),
      relayTargets: [],
      linkValidator: hasTerrainLineOfSight
    });
    if (sim.commsRenderElapsedMs > budgetedInterval(budget.commRenderMs) || !main.commGroup.children.length) {
      sim.commsRenderElapsedMs = 0;
      renderCommunicationGraph(idleSnapshot.communication.edges);
    }
    sim.swarmController.agents.forEach(refreshSwarmDroneRoleVisual);
    sim.swarmSnapshot = sim.swarmController.snapshot();
    return;
  }

  sim.swarmLaunchElapsedMs += deltaSeconds * 1000;
  sim.swarmTelemetryElapsedMs += deltaSeconds * 1000;
  sim.swarmScanElapsedMs += deltaSeconds * 1000;
  sim.commsRenderElapsedMs += deltaSeconds * 1000;
  sim.cloudFlushElapsedMs += deltaSeconds * 1000;

  if (sim.swarmScanElapsedMs > budgetedInterval(budget.scanIntervalMs)) {
    sim.swarmScanElapsedMs = 0;
    performSwarmSensorPass();
  }

  const center = computeSwarmMissionCenter(selectedAoiTargets());
  const frontierDensity = sim.mapper ? sim.mapper.frontierIndices.length / Math.max(sim.mapper.states.length, 1) : 0;
  const obstacleDensity = sim.mapper ? sim.mapper.occupiedCount / Math.max(sim.mapper.knownCount, 1) : 0;
  const terrainSignals = sim.terrainProfile ?? {};
  const activeAois = selectedAoiTargets();
  const constraintSignals = swarmConstraintSignals(activeAois);
  const snapshot = sim.swarmController.update({
    requestedFormation: els.swarmFormation.value,
    environmentSignals: {
      frontierDensity,
      obstacleDensity,
      verticality: terrainSignals.verticality ?? (sim.ceilingHazard ? 0.42 : 0.12),
      occlusion: terrainSignals.corridorScore ? 0.48 : obstacleDensity * 0.8,
      corridorScore: terrainSignals.corridorScore ?? (sim.room && sim.room.depth > sim.room.width * 1.25 ? 0.72 : 0.2),
      openness: estimateSwarmOpenness(),
      scanProgress: clamp(sim.scanCounter / 180, 0, 1),
      ...constraintSignals
    },
    frontiers: sampleSwarmFrontierTasks(18),
    aois: activeAois,
    relayTargets: sampleSwarmRelayTasks(center),
    linkValidator: hasTerrainLineOfSight
  });

  sim.swarmTargets = sim.swarmController.buildFormationTargets({
    center,
    heading: new THREE.Vector3(0, 0, 1),
    radius: Math.max(sim.room.width, sim.room.depth) * 0.18 * (snapshot.metrics.derivedControls?.formationRadiusScale ?? 1),
    verticalSpan: Math.max(0.8, sim.room.height * 0.28) * (snapshot.metrics.derivedControls?.formationVerticalScale ?? 1)
  });

  const launchProgress = clamp(sim.swarmLaunchElapsedMs / 3600, 0, 1);
  const speedScale = launchProgress < 1 ? 0.45 : 1;
  const speed = deltaSeconds > 0 ? clamp(readNumber(els.moveSpeed), 0.2, 4) * speedScale * deltaSeconds : 0;
  sim.swarmController.agents.forEach((agent) => {
    const target = sim.swarmTargets.find((candidate) => candidate.droneId === agent.id);
    if (!target) {
      return;
    }
    const takeoffTarget = agent.launchPosition.clone();
    takeoffTarget.y = terrainClearanceY(takeoffTarget.x, takeoffTarget.z, terrainCruiseClearance() * launchProgress);
    const activeTarget = launchProgress < 1 ? takeoffTarget : terrainAwareTarget(target.position, agent);
    const next = smoothSwarmAgentStep(agent, activeTarget, speed, deltaSeconds);
    const distance = agent.position.distanceTo(activeTarget);
    agent.setPosition(next);
    refreshSwarmDroneRoleVisual(agent);
    if (agent.mesh && distance > 0.01) {
      agent.mesh.lookAt(activeTarget);
    }
  });

  resolveSwarmCollisions();
  resolveSwarmObstacleAvoidance();
  enforceSwarmNetworkEnvelope();
  const communication = sim.swarmController.communicationGraph.update(sim.swarmController.agents, { canLink: hasTerrainLineOfSight });
  if (sim.commsRenderElapsedMs > budgetedInterval(budget.commRenderMs) || !main.commGroup.children.length) {
    sim.commsRenderElapsedMs = 0;
    renderCommunicationGraph(communication.edges);
  }
  if (sim.cloudDirty && sim.cloudFlushElapsedMs > budgetedInterval(budget.cloudFlushMs)) {
    sim.cloudFlushElapsedMs = 0;
    flushPointCloudGeometry(Math.max(38, clamp(readNumber(els.maxRange), 2, 80)));
  }
  sim.swarmSnapshot = sim.swarmController.snapshot();
  renderAlgorithmPanel();
  recordSwarmTelemetrySample(activeAois);
  if (
    sim.policyExperiment.active
    && !sim.policyExperiment.waitingForNext
    && sim.swarmTelemetryElapsedMs >= sim.policyExperiment.runMs
  ) {
    completeCurrentPolicyExperimentPreset();
  }
  sim.currentPhase = launchProgress < 1 ? 'launch' : 'swarm';
}

function swarmConstraintSignals(activeAois = selectedAoiTargets()) {
  const agents = sim.swarmController?.agents ?? [];
  const nearestAoiDistance = nearestSwarmAoiDistance(activeAois, agents);
  const nearestAoiRadius = activeAois.length
    ? activeAois.reduce((nearest, aoi) => {
        const distance = agents.length
          ? Math.min(...agents.map((agent) => agent.position.distanceTo(aoi.position)))
          : Infinity;
        return distance < nearest.distance ? { distance, radius: aoi.radius ?? 1 } : nearest;
      }, { distance: Infinity, radius: 1 }).radius
    : 1;
  const aoiProximityRisk = Number.isFinite(nearestAoiDistance)
    ? clamp(1 - nearestAoiDistance / Math.max(nearestAoiRadius * 2.8, 1), 0, 1)
    : 0;
  return {
    elapsedMs: sim.swarmTelemetryElapsedMs,
    aoiProximityRisk,
    batteryRemainingPct: sim.swarmTelemetry.currentRun?.resources?.batteryRemainingPct ?? 1,
    avgFrameMs: sim.performance.frameAvgMs,
    avgScanPassMs: sim.performance.scanPassMs,
    footprintRedundancy: sim.footprintMetrics.redundancyRatio,
    footprintResolutionScore: sim.footprintMetrics.resolutionScore,
    footprintUniqueAreaRate: sim.footprintMetrics.uniqueAreaRate
  };
}

function nearestSwarmAoiDistance(activeAois, agents = sim.swarmController?.agents ?? []) {
  if (!activeAois.length || !agents.length) {
    return Infinity;
  }
  return Math.min(...activeAois.flatMap((aoi) => agents.map((agent) => agent.position.distanceTo(aoi.position))));
}

function startSwarmTelemetryRun() {
  const activeAois = selectedAoiTargets();
  const launchZ = -sim.room.depth * 0.42;
  const launchOrigin = new THREE.Vector3(0, terrainClearanceY(0, launchZ, 0.28), launchZ);
  const profile = swarmBehaviorProfile();
  sim.swarmTelemetry.startRun({
    scenario: {
      datasetWorkspace: datasetWorkspaceName(),
      seed: sim.environmentSeed,
      terrain: { ...sim.terrainConfig },
      aoiPreset: els.aoiPreset.value,
      aoiTargets: activeAois.map((aoi) => ({
        id: aoi.id,
        type: aoi.type,
        label: aoi.label,
        radius: aoi.radius,
        distanceFromLaunch: aoi.position.distanceTo(launchOrigin)
      })),
      aoiNearestDistance: activeAois.length
        ? Math.min(...activeAois.map((aoi) => aoi.position.distanceTo(launchOrigin)))
        : null,
      swarm: readSwarmConfig(),
      scan: {
        density: els.swarmScanDensity.value,
        horizontalRays: clampInt(readNumber(els.horizontalRays), 4, 48),
        verticalRays: clampInt(readNumber(els.verticalRays), 2, 16),
        horizontalFov: clamp(readNumber(els.horizontalFov), 90, 360),
        verticalFov: clamp(readNumber(els.verticalFov), 30, 120),
        range: Math.max(38, clamp(readNumber(els.maxRange), 2, 80)),
        voxelSize: clamp(readNumber(els.voxelSize), 0.015, 0.2)
      },
      performanceBudget: els.performanceBudget.value,
      controlSnapshot: captureControlSnapshot(),
      policyExperiment: sim.policyExperiment.currentPreset ? { ...sim.policyExperiment.currentPreset } : null
    },
    behaviorProfile: {
      version: profile.version,
      modelFamily: profile.modelFamily,
      modelVersion: profile.modelVersion,
      objective: profile.objectives?.default,
      basePolicyCoordinates: { ...(profile.basePolicyCoordinates ?? profile.policyCoordinates ?? DEFAULT_POLICY_COORDINATES) },
      policyCoordinates: { ...(profile.policyCoordinates ?? DEFAULT_POLICY_COORDINATES) },
      runtimeNudgeProfile: profile.runtimeNudgeProfile,
      runtimeNudgeScale: profile.runtimeNudgeScale,
      runtimeNudge: { ...(profile.runtimeNudge ?? {}) },
      effectivePolicyCoordinates: { ...(profile.effectivePolicyCoordinates ?? profile.policyCoordinates ?? DEFAULT_POLICY_COORDINATES) },
      deltaPolicyCoordinates: { ...(profile.deltaPolicyCoordinates ?? {}) },
      derivedProfileSummary: { ...(profile.derivedProfileSummary ?? {}) }
    }
  });
}

function captureControlSnapshot() {
  const snapshot = {};
  document.querySelectorAll('input, select, textarea').forEach((control) => {
    const key = control.id || control.name;
    if (!key) {
      return;
    }
    snapshot[key] = {
      tag: control.tagName.toLowerCase(),
      type: control.type || null,
      value: control.type === 'checkbox' ? Boolean(control.checked) : control.value,
      min: control.min || null,
      max: control.max || null,
      step: control.step || null
    };
  });
  return snapshot;
}

function datasetWorkspaceName() {
  const value = (els.datasetWorkspace?.value ?? '').trim();
  return value || DEFAULT_DATASET_WORKSPACE;
}

function recordSwarmTelemetrySample(activeAois = selectedAoiTargets()) {
  if (!sim.swarmTelemetry.active || !sim.swarmController) {
    return;
  }
  const agents = sim.swarmController.agents;
  const aoiStats = swarmAoiProximityStats(activeAois);
  const totalPathLength = agents.reduce((sum, agent) => sum + agent.metrics.distanceTraveled, 0);
  const energyProxy = agents.reduce((sum, agent) => sum + agent.metrics.distanceTraveled * roleEnergyScale(agent.role), 0);
  const resources = estimateSwarmResourceUse({
    agents,
    elapsedMs: sim.swarmTelemetryElapsedMs,
    scanTotals: sim.swarmTelemetry.currentRun?.totals,
    network: {
      avgLinks: sim.swarmTelemetry.currentRun?.network?.avgLinks,
      linkCount: sim.swarmSnapshot?.communication?.edges?.length ?? 0
    },
    performance: sim.swarmTelemetry.currentRun?.performance,
    model: sim.droneResourceModel
  });
  sim.swarmTelemetry.recordSample({
    elapsedMs: sim.swarmTelemetryElapsedMs,
    scanCounter: sim.scanCounter,
    pointVoxels: sim.pointCount,
    coverage: sim.mapper ? sim.mapper.knownCount / Math.max(sim.mapper.states.length, 1) : 0,
    formationMode: sim.swarmSnapshot?.metrics?.formationMode ?? els.swarmFormation.value,
    behaviorWeights: sim.swarmSnapshot?.metrics?.behaviorWeights ?? null,
    normalizedSignals: sim.swarmSnapshot?.metrics?.normalizedSignals ?? null,
    derivedControls: sim.swarmSnapshot?.metrics?.derivedControls ?? null,
    modelFamily: sim.swarmSnapshot?.metrics?.modelFamily ?? GREYBOX_POLICY_MODEL.modelFamily,
    modelVersion: sim.swarmSnapshot?.metrics?.modelVersion ?? GREYBOX_POLICY_MODEL.modelVersion,
    basePolicyCoordinates: sim.swarmSnapshot?.metrics?.basePolicyCoordinates ?? swarmBehaviorProfile().basePolicyCoordinates,
    policyCoordinates: sim.swarmSnapshot?.metrics?.policyCoordinates ?? swarmBehaviorProfile().policyCoordinates,
    runtimeNudgeProfile: sim.swarmSnapshot?.metrics?.runtimeNudgeProfile ?? swarmBehaviorProfile().runtimeNudgeProfile,
    runtimeNudgeScale: sim.swarmSnapshot?.metrics?.runtimeNudgeScale ?? swarmBehaviorProfile().runtimeNudgeScale,
    runtimeNudge: sim.swarmSnapshot?.metrics?.runtimeNudge ?? swarmBehaviorProfile().runtimeNudge,
    effectivePolicyCoordinates: sim.swarmSnapshot?.metrics?.effectivePolicyCoordinates ?? swarmBehaviorProfile().effectivePolicyCoordinates,
    deltaPolicyCoordinates: sim.swarmSnapshot?.metrics?.deltaPolicyCoordinates ?? swarmBehaviorProfile().deltaPolicyCoordinates,
    derivedProfileSummary: sim.swarmSnapshot?.metrics?.derivedProfileSummary ?? swarmBehaviorProfile().derivedProfileSummary,
    behaviorProfileVersion: sim.swarmSnapshot?.metrics?.behaviorProfileVersion ?? null,
    communicationHealth: sim.swarmSnapshot?.metrics?.communicationHealth ?? 1,
    connectedComponents: sim.swarmSnapshot?.metrics?.connectedComponents ?? 0,
    linkCount: sim.swarmSnapshot?.communication?.edges?.length ?? 0,
    adaptiveNeighborTarget: sim.swarmSnapshot?.metrics?.adaptiveNeighborTarget ?? clampInt(readNumber(els.communicationNeighbors), 1, 8),
    roleCounts: swarmRoleCounts(),
    aoiInFocus: aoiStats.inFocus,
    aoiFocusedAgents: aoiStats.focusedAgents,
    nearestAoiDistance: aoiStats.nearestDistance,
    droneTrajectories: buildSparseDroneTrajectories(agents, activeAois),
    totalPathLength,
    energyProxy,
    resources,
    frameAvgMs: sim.performance.frameAvgMs,
    scanPassMs: sim.performance.scanPassMs
  });
}

function buildSparseDroneTrajectories(agents, activeAois = selectedAoiTargets()) {
  return agents.map((agent) => {
    const nearestAoiDistance = activeAois.length
      ? Math.min(...activeAois.map((aoi) => agent.position.distanceTo(aoi.position)))
      : null;
    return {
      id: agent.id,
      role: agent.role,
      status: agent.status,
      position: vectorRecord(agent.position),
      velocity: vectorRecord(agent.velocity),
      target: vectorRecord(agent.smoothedTarget),
      launchOffset: vectorRecord(agent.position.clone().sub(agent.launchPosition)),
      pathLengthM: agent.metrics.distanceTraveled,
      points: agent.metrics.points,
      scans: agent.metrics.scans,
      nearestAoiDistance,
      assignmentType: agent.assignment?.task?.type ?? agent.assignment?.type ?? null,
      behaviorFocus: agent.behavior?.sensingFocus ?? null,
      communicationHealth: agent.behavior?.communicationHealth ?? null
    };
  });
}

function vectorRecord(vector) {
  return {
    x: roundTelemetryNumber(vector?.x ?? 0),
    y: roundTelemetryNumber(vector?.y ?? 0),
    z: roundTelemetryNumber(vector?.z ?? 0)
  };
}

function roundTelemetryNumber(value) {
  return Math.round((Number(value) || 0) * 1000) / 1000;
}

function finalizeSwarmTelemetry(reason) {
  if (!sim.swarmTelemetry?.active) {
    return;
  }
  const validity = evaluateSwarmRunValidity(reason);
  const scoring = scoreSwarmRun({
    ...sim.swarmTelemetry.currentRun,
    validity
  }, swarmBehaviorProfile());
  const activeAois = selectedAoiTargets();
  const notes = [
    `Ended because ${reason}.`,
    activeAois.length ? `AOI preset ${els.aoiPreset.value} with ${activeAois.length} active target(s).` : 'No active AOI target.'
  ];
  sim.swarmTelemetry.finishRun({
    status: reason,
    endedAtMs: performance.now(),
    validity,
    scoring,
    notes
  }).then((run) => {
    if (run) {
      logMessage(`Telemetry saved: ${run.status} run, ${run.totals.rawHits.toLocaleString()} raw hits, score ${formatScore(run.scoring?.total)}, ${run.validity.flags.length} validity flag(s).`);
    }
  });
}

function evaluateSwarmRunValidity(reason) {
  const run = sim.swarmTelemetry.currentRun;
  const flags = [];
  const elapsedMs = run?.elapsedMs ?? sim.swarmTelemetryElapsedMs;
  const validity = DEFAULT_SWARM_EVALUATION_PROFILE.validity;
  const minAoiDwellMs = run?.aoi.selectedCount ? validity.minAoiDwellMs : 0;
  const minAoiHits = run?.aoi.selectedCount ? validity.minAoiHits : 0;

  if (reason === 'cancelled_batch') {
    flags.push('batch_cancelled');
  }
  if (reason !== 'complete' && elapsedMs < validity.minRunMs) {
    flags.push('insufficient_scan_time');
  }
  if (run?.aoi.selectedCount && run.aoi.firstContactMs === null) {
    flags.push('target_not_reached');
  }
  if (run?.aoi.selectedCount && run.aoi.dwellMs < minAoiDwellMs) {
    flags.push('insufficient_aoi_dwell');
  }
  if (run?.aoi.selectedCount && run.aoi.rawHits < minAoiHits) {
    flags.push('low_raw_aoi_coverage');
  }
  if (sim.performance.frameAvgMs > validity.maxStableFrameMs) {
    flags.push('performance_unstable');
  }

  return {
    complete: flags.length === 0 && reason !== 'reset',
    flags
  };
}

function swarmRoleCounts() {
  const counts = {};
  (sim.swarmController?.agents ?? []).forEach((agent) => {
    counts[agent.role] = (counts[agent.role] ?? 0) + 1;
  });
  return counts;
}

function swarmAoiProximityStats(activeAois) {
  const agents = sim.swarmController?.agents ?? [];
  if (!agents.length || !activeAois.length) {
    return { inFocus: false, focusedAgents: 0, nearestDistance: null };
  }
  const focusDistance = Math.max(18, Math.min(60, clamp(readNumber(els.maxRange), 2, 80) * 0.75));
  let nearestDistance = Infinity;
  let focusedAgents = 0;
  agents.forEach((agent) => {
    const distance = Math.min(...activeAois.map((aoi) => agent.position.distanceTo(aoi.position)));
    nearestDistance = Math.min(nearestDistance, distance);
    if (distance <= focusDistance) {
      focusedAgents += 1;
    }
  });
  return {
    inFocus: focusedAgents >= Math.min(3, agents.length),
    focusedAgents,
    nearestDistance
  };
}

function roleEnergyScale(role) {
  switch (role) {
    case DRONE_ROLES.RELAY:
      return 0.92;
    case DRONE_ROLES.SCOUT:
      return 1.08;
    case DRONE_ROLES.VERIFIER:
      return 1.04;
    default:
      return 1;
  }
}

function computeSwarmMissionCenter(activeAois) {
  if (activeAois.length && els.aoiPreset.value !== 'auto') {
    const aoi = activeAois[0];
    const radius = Math.max(10, aoi.radius ?? 16);
    const sweepPhase = sim.swarmLaunchElapsedMs * 0.00012 + sim.scanCounter * 0.035;
    const x = aoi.position.x - radius * 0.34 + Math.sin(sweepPhase * 0.43) * radius * 0.16;
    const z = aoi.position.z + Math.sin(sweepPhase) * radius * 0.72;
    const y = Math.max(
      aoi.position.y + 3.4 + Math.sin(sweepPhase * 0.55) * 2.2,
      terrainClearanceY(x, z, terrainCruiseClearance())
    );
    return terrainAwareTarget(new THREE.Vector3(x, y, z));
  }

  const progress = clamp(sim.scanCounter / 180, 0, 1);
  const z = lerp(-sim.room.depth * 0.36, sim.room.depth * 0.34, progress);
  const x = Math.sin(progress * Math.PI * 2.2) * 5.5;
  return new THREE.Vector3(x, terrainClearanceY(x, z, terrainCruiseClearance()), z);
}

function terrainAwareTarget(position, agent = null) {
  const target = position.clone();
  target.y = Math.max(target.y, terrainClearanceY(target.x, target.z, terrainCruiseClearance()));
  const assignmentGoal = agent?.assignment?.goal ?? agent?.assignment;
  if (assignmentGoal?.position) {
    const goal = assignmentGoal.position;
    const blend = roleGoalBlend(agent.role, assignmentGoal.type);
    target.x = lerp(target.x, goal.x, blend);
    target.z = lerp(target.z, goal.z, blend);
    target.y = lerp(target.y, goal.y ?? target.y, blend * 0.45);
    target.y = Math.max(target.y, terrainClearanceY(target.x, target.z, terrainCruiseClearance()));
  }
  return clampSwarmPosition(target);
}

function roleGoalBlend(role, taskType) {
  const movement = swarmBehaviorProfile().movement;
  const controls = swarmDerivedControls();
  let key = 'default';
  let base = movement.assignmentBlend.default;
  if (role === DRONE_ROLES.RELAY || taskType === 'relay') {
    key = 'relay';
    base = movement.assignmentBlend.relay;
  } else if (role === DRONE_ROLES.VERIFIER || taskType === 'verify') {
    key = 'verify';
    base = movement.assignmentBlend.verify;
  } else if (role === DRONE_ROLES.MAPPER || taskType === 'aoi') {
    key = taskType === 'aoi' ? 'aoi' : 'mapper';
    base = movement.assignmentBlend[key];
  }
  return clamp(base * (controls.assignmentBlendScale?.[key] ?? 1), 0.04, 0.72);
}

function smoothSwarmAgentStep(agent, target, maxStep, deltaSeconds) {
  const movement = swarmBehaviorProfile().movement;
  const controls = swarmDerivedControls();
  if (deltaSeconds <= 0 || maxStep <= 0) {
    return agent.position.clone();
  }

  if (!agent.smoothedTarget) {
    agent.smoothedTarget = target.clone();
  }
  const targetBlend = 1 - Math.exp(-deltaSeconds * movement.targetSmoothingPerSecond);
  agent.smoothedTarget.lerp(target, targetBlend);

  const desired = agent.smoothedTarget.clone().sub(agent.position);
  const distance = desired.length();
  if (distance < 0.02) {
    agent.steeringVelocity.multiplyScalar(Math.exp(-deltaSeconds * movement.stopDampingPerSecond));
    return clampSwarmPosition(agent.position.clone().addScaledVector(agent.steeringVelocity, deltaSeconds));
  }

  const cruiseSpeed = maxStep / Math.max(deltaSeconds, 1e-6);
  const roleSpeed = roleSpeedScale(agent.role);
  desired.normalize().multiplyScalar(
    cruiseSpeed * roleSpeed * (controls.movementFreedom ?? 1) * clamp(distance / movement.slowdownDistance, movement.minApproachScale, 1)
  );

  const steeringBlend = 1 - Math.exp(-deltaSeconds * movement.steeringSmoothingPerSecond);
  agent.steeringVelocity.lerp(desired, steeringBlend);
  const step = agent.steeringVelocity.clone().multiplyScalar(deltaSeconds);
  if (step.length() > maxStep * roleSpeed) {
    step.setLength(maxStep * roleSpeed);
  }
  return clampSwarmPosition(agent.position.clone().add(step));
}

function roleSpeedScale(role) {
  return (swarmBehaviorProfile().movement.roleSpeedScale[role] ?? 1) * (swarmDerivedControls().roleSpeedScale?.[role] ?? 1);
}

function swarmBehaviorProfile() {
  return sim.swarmController?.behaviorProfile ?? activeBehaviorProfile();
}

function swarmDerivedControls() {
  return sim.swarmSnapshot?.metrics?.derivedControls ?? sim.swarmController?.metrics?.derivedControls ?? {};
}

function terrainClearanceY(x, z, extraClearance = 0) {
  return terrainHeightAt(x, z) + Math.max(0.38, extraClearance);
}

function terrainCruiseClearance() {
  return Math.max(clamp(readNumber(els.terrainFlightClearance), 2, 18), swarmCollisionRadius() * 4.2);
}

function hasTerrainLineOfSight(from, to) {
  const start = from.position;
  const end = to.position;
  const distance = start.distanceTo(end);
  const samples = clampInt(Math.ceil(distance / 4), 4, 28);
  const clearance = Math.max(0.7, swarmCollisionRadius() * 1.4);

  for (let index = 1; index < samples; index += 1) {
    const t = index / samples;
    const x = lerp(start.x, end.x, t);
    const y = lerp(start.y, end.y, t);
    const z = lerp(start.z, end.z, t);
    if (y < terrainHeightAt(x, z) + clearance) {
      return false;
    }
  }

  return true;
}

function sampleSwarmFrontierTasks(limit) {
  const tasks = [];
  const scanProgress = clamp(sim.scanCounter / 180, 0, 1);
  const baseZ = lerp(-sim.room.depth * 0.35, sim.room.depth * 0.35, scanProgress);
  const lateralBands = [-18, -9, 0, 9, 18];
  for (let index = 0; index < limit; index += 1) {
    const x = lateralBands[index % lateralBands.length] + Math.sin(index * 1.7) * 2.5;
    const z = baseZ + Math.floor(index / lateralBands.length) * 8;
    if (z > sim.room.depth * 0.42) {
      break;
    }
    tasks.push({
      id: `terrain-frontier-${index}`,
      position: new THREE.Vector3(x, terrainClearanceY(x, z, terrainCruiseClearance()), z),
      priority: 1 + (index % lateralBands.length === 2 ? 0.2 : 0),
      informationGain: 1.2
    });
  }
  return tasks;
}

function sampleSwarmRelayTasks(missionCenter) {
  const relay = swarmBehaviorProfile().relay;
  const agents = sim.swarmController?.agents ?? [];
  if (agents.length < 3) {
    return [];
  }

  const range = clamp(readNumber(els.communicationRange), 2, 80);
  const baseZ = sim.room.depth * relay.baseZFactor;
  const base = new THREE.Vector3(0, terrainClearanceY(0, baseZ, terrainCruiseClearance()), baseZ);
  const center = missionCenter ?? computeSwarmMissionCenter(selectedAoiTargets());
  const distance = base.distanceTo(center);
  const relayCount = clampInt(
    Math.ceil(distance / Math.max(range * relay.spacingRangeFactor, relay.minimumSpacing)) - 1,
    1,
    Math.min(relay.maxTasks, agents.length - 2)
  );
  const tasks = [];

  for (let index = 0; index < relayCount; index += 1) {
    const t = (index + 1) / (relayCount + 1);
    const x = lerp(base.x, center.x, t);
    const z = lerp(base.z, center.z, t);
    const y = Math.max(
      lerp(base.y, center.y, t) + Math.sin(t * Math.PI) * relay.altitudeArc,
      terrainClearanceY(x, z, terrainCruiseClearance() * relay.clearanceScale)
    );
    tasks.push({
      id: `relay-${index}`,
      position: new THREE.Vector3(x, y, z),
      priority: relay.priorityBase + t * relay.priorityDistanceGain,
      informationGain: relay.informationGain
    });
  }

  return tasks;
}

function estimateSwarmOpenness() {
  const agents = sim.swarmController?.agents ?? [];
  if (!agents.length) {
    return sim.terrainProfile?.openness ?? 0.5;
  }
  const averagePoints = agents.reduce((sum, agent) => sum + agent.metrics.points, 0) / agents.length;
  return clamp(1 - averagePoints / 600, 0.18, 0.9);
}

function performSwarmSensorPass() {
  const agents = sim.swarmController?.agents ?? [];
  if (!agents.length) {
    return;
  }

  const passStart = performance.now();
  const budget = readPerformanceBudgetConfig();
  const beams = [];
  const directions = buildSwarmScanDirections();
  const activeAois = selectedAoiTargets();
  const sensorRange = Math.max(38, clamp(readNumber(els.maxRange), 2, 80));
  let rayCount = 0;
  let hitCount = 0;
  let aoiHitCount = 0;
  let focusedHitCount = 0;
  let footprintAreaM2 = 0;
  let uniqueFootprintAreaM2 = 0;
  let redundantFootprintAreaM2 = 0;
  let resolutionScoreSum = 0;

  agents.forEach((agent, agentIndex) => {
    let hits = 0;
    directions.forEach((direction, directionIndex) => {
      temp.source.copy(agent.position);
      temp.dir.copy(direction).normalize();
      rayCount += 1;
      const hit = castSwarmRay(temp.source, temp.dir, sensorRange);
      if (!hit) {
        return;
      }
      hits += 1;
      hitCount += 1;
      const metadata = pointAoiMetadata(hit.point, activeAois, false);
      if (metadata.aoiId) {
        aoiHitCount += 1;
      }
      const footprint = estimateRayFootprint(hit.distance, false);
      const footprintCoverage = recordFootprintCoverage(hit.point, footprint);
      footprintAreaM2 += footprint.areaM2;
      uniqueFootprintAreaM2 += footprintCoverage.newAreaM2;
      redundantFootprintAreaM2 += footprintCoverage.redundantAreaM2;
      resolutionScoreSum += footprint.resolutionScore;
      addPointToCloud(hit.point, hit.distance, metadata, hit);
      agent.recordPointObservation({
        position: hit.point.clone(),
        count: 1,
        source: agent.id,
        distance: hit.distance,
        aoiId: metadata.aoiId
      });
      if (
        agentIndex < budget.beamDroneCap &&
        directionIndex % Math.max(1, Math.floor(directions.length / budget.beamSamples)) === 0
      ) {
        beams.push([temp.source.clone(), hit.point.clone()]);
      }
    });

    const focus = nearestAoiFocus(agent, activeAois, sensorRange, agentIndex);
    if (focus) {
      const focusDirections = buildAoiFocusDirections(agent.position, focus.focusPoint, focus.strength * roleAoiFocusScale(agent.role));
      focusDirections.forEach((direction, directionIndex) => {
        temp.source.copy(agent.position);
        temp.dir.copy(direction).normalize();
        rayCount += 1;
        const hit = castSwarmRay(temp.source, temp.dir, sensorRange);
        if (!hit) {
          return;
        }
        hits += 1;
        hitCount += 1;
        const metadata = pointAoiMetadata(hit.point, [focus], true);
        if (metadata.aoiId) {
          aoiHitCount += 1;
        }
        focusedHitCount += 1;
        const footprint = estimateRayFootprint(hit.distance, true);
        const footprintCoverage = recordFootprintCoverage(hit.point, footprint);
        footprintAreaM2 += footprint.areaM2;
        uniqueFootprintAreaM2 += footprintCoverage.newAreaM2;
        redundantFootprintAreaM2 += footprintCoverage.redundantAreaM2;
        resolutionScoreSum += footprint.resolutionScore;
        addPointToCloud(hit.point, hit.distance, metadata, hit);
        agent.recordPointObservation({
          position: hit.point.clone(),
          count: 1,
          source: agent.id,
          distance: hit.distance,
          aoiId: metadata.aoiId,
          focused: true
        });
        if (
          agentIndex < budget.beamDroneCap &&
          directionIndex % Math.max(1, Math.floor(focusDirections.length / 3)) === 0
        ) {
          beams.push([temp.source.clone(), hit.point.clone()]);
        }
      });
    }

    agent.metrics.scans += 1;
    agent.recordMapDelta({
      type: 'swarm-scan',
      position: agent.position.clone(),
      hits
    });
  });

  sim.performance.scanPassMs = performance.now() - passStart;
  sim.performance.lastScanRayCount = rayCount;
  sim.performance.lastScanHits = hitCount;
  sim.footprintMetrics = {
    redundancyRatio: footprintAreaM2 > 0 ? redundantFootprintAreaM2 / Math.max(footprintAreaM2, 1e-6) : 0,
    resolutionScore: hitCount ? resolutionScoreSum / hitCount : sim.footprintMetrics.resolutionScore,
    uniqueAreaRate: uniqueFootprintAreaM2 / Math.max(sim.performance.scanPassMs / 1000, 1e-3)
  };
  sim.swarmTelemetry.recordScan({
    rayCount,
    hitCount,
    focusedHitCount,
    aoiHitCount,
    scanPassMs: sim.performance.scanPassMs,
    pointVoxels: sim.pointCount,
    footprintAreaM2,
    uniqueFootprintAreaM2,
    redundantFootprintAreaM2,
    avgFootprintResolutionScore: hitCount ? resolutionScoreSum / hitCount : 0
  });
  sim.scanCounter += 1;
  renderScanBeams(beams, 'swarm');
}

function roleAoiFocusScale(role) {
  return (swarmBehaviorProfile().sensing.roleAoiFocusScale[role] ?? 1) * (swarmDerivedControls().sensingFocusScale?.[role] ?? 1);
}

function nearestAoiFocus(agent, activeAois, sensorRange, agentIndex) {
  if (!activeAois.length) {
    return null;
  }

  let best = null;
  const sensing = swarmBehaviorProfile().sensing;
  activeAois.forEach((aoi) => {
    const distance = agent.position.distanceTo(aoi.position);
    const focusRange = Math.min(
      sensorRange * sensing.focusRangeSensorFactor,
      Math.max(sensing.focusRangeMin, (aoi.radius ?? 14) * sensing.focusRangeRadiusFactor)
    );
    if (distance > focusRange) {
      return;
    }
    const strength = clamp(1 - distance / focusRange, sensing.focusStrengthMin, 1);
    if (!best || strength > best.strength) {
      best = {
        ...aoi,
        distance,
        strength,
        focusPoint: aoiDistributedFocusPoint(aoi, agentIndex, sim.scanCounter)
      };
    }
  });
  return best;
}

function aoiDistributedFocusPoint(aoi, agentIndex, scanIndex) {
  const radius = Math.max(6, aoi.radius ?? 14);
  const laneCount = 11;
  const lane = (agentIndex * 2 + scanIndex) % laneCount;
  const zT = laneCount === 1 ? 0.5 : lane / (laneCount - 1);
  const zOffset = lerp(-radius * 0.92, radius * 0.92, zT);
  const xLane = ((agentIndex * 5 + scanIndex * 2) % 7) - 3;
  const xOffset = xLane * radius * 0.16;
  const yBand = ((agentIndex + scanIndex * 3) % 5) - 2;
  const yOffset = yBand * radius * 0.06;
  const x = aoi.position.x + xOffset;
  const z = aoi.position.z + zOffset;
  const y = Math.max(
    aoi.position.y + yOffset,
    terrainClearanceY(x, z, terrainCruiseClearance() * 0.55)
  );
  return new THREE.Vector3(x, y, z);
}

function buildAoiFocusDirections(source, target, strength) {
  const direction = target.clone().sub(source).normalize();
  if (direction.lengthSq() < 1e-6) {
    return [];
  }

  const upReference = Math.abs(direction.y) > 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
  const right = new THREE.Vector3().crossVectors(direction, upReference).normalize();
  const up = new THREE.Vector3().crossVectors(right, direction).normalize();
  const rings = [
    { radius: 0, samples: 1 },
    { radius: 0.026, samples: strength > 0.68 ? 8 : 4 },
    { radius: 0.056, samples: strength > 0.68 ? 10 : 5 },
    { radius: 0.092, samples: strength > 0.82 ? 14 : strength > 0.5 ? 8 : 4 },
    { radius: 0.13, samples: strength > 0.86 ? 14 : 0 }
  ];
  const directions = [];
  rings.forEach((ring, ringIndex) => {
    if (ring.samples <= 0) {
      return;
    }
    for (let sample = 0; sample < ring.samples; sample += 1) {
      const angle = (sample / ring.samples) * Math.PI * 2 + ringIndex * 0.37;
      const offset = right.clone().multiplyScalar(Math.cos(angle) * ring.radius).add(
        up.clone().multiplyScalar(Math.sin(angle) * ring.radius)
      );
      directions.push(direction.clone().add(offset).normalize());
    }
  });
  return directions;
}

function buildSwarmScanDirections() {
  const horizontalCount = clampInt(readNumber(els.horizontalRays), 4, 48);
  const verticalCount = clampInt(readNumber(els.verticalRays), 2, 16);
  const horizontalFovDeg = clamp(readNumber(els.horizontalFov), 90, 360);
  const verticalFovDeg = clamp(readNumber(els.verticalFov), 30, 120);
  const cacheKey = `${horizontalCount}:${verticalCount}:${horizontalFovDeg}:${verticalFovDeg}`;
  if (sim.scanDirectionCache.key === cacheKey) {
    return sim.scanDirectionCache.directions;
  }

  const horizontalFov = THREE.MathUtils.degToRad(horizontalFovDeg);
  const verticalFov = THREE.MathUtils.degToRad(verticalFovDeg);
  const directions = [new THREE.Vector3(0, -1, 0)];

  for (let v = 0; v < verticalCount; v += 1) {
    const verticalT = verticalCount === 1 ? 0.5 : v / (verticalCount - 1);
    const elevation = lerp(-verticalFov * 0.55, verticalFov * 0.22, verticalT);
    for (let h = 0; h < horizontalCount; h += 1) {
      const horizontalT = horizontalCount === 1 ? 0.5 : h / (horizontalCount - 1);
      const yaw = lerp(-horizontalFov * 0.5, horizontalFov * 0.5, horizontalT) + Math.PI * 0.5;
      directions.push(new THREE.Vector3(
        Math.cos(elevation) * Math.cos(yaw),
        Math.sin(elevation),
        Math.cos(elevation) * Math.sin(yaw)
      ));
    }
  }

  sim.scanDirectionCache.key = cacheKey;
  sim.scanDirectionCache.directions = directions;
  return directions;
}

function swarmScanDensityConfig() {
  switch (els.swarmScanDensity.value) {
    case 'economy':
      return { horizontal: 4, vertical: 2, writeInputs: true };
    case 'light':
      return { horizontal: 4, vertical: 4, writeInputs: true };
    case 'dense':
      return { horizontal: 18, vertical: 5, writeInputs: true };
    case 'survey':
      return { horizontal: 24, vertical: 7, writeInputs: true };
    case 'max':
      return { horizontal: 34, vertical: 10, writeInputs: true };
    case 'custom':
      return {
        horizontal: clampInt(readNumber(els.horizontalRays), 4, 48),
        vertical: clampInt(readNumber(els.verticalRays), 2, 16),
        writeInputs: false
      };
    case 'balanced':
    default:
      return { horizontal: 14, vertical: 4, writeInputs: true };
  }
}

function castSwarmRay(source, direction, maxRange) {
  return castSceneRay(source, direction, maxRange);
}

function castSceneRay(source, direction, maxRange) {
  if (!sim.scanTargets.length || maxRange <= 0.05) {
    return null;
  }
  raycaster.near = 0.05;
  raycaster.far = maxRange;
  raycaster.set(source, direction);
  const intersections = raycaster.intersectObjects(sim.scanTargets, true);
  const hit = intersections[0] ?? null;
  if (hit) {
    hit.rayOrigin = source.clone();
  }
  return hit;
}

function enforceSwarmNetworkEnvelope() {
  const network = swarmBehaviorProfile().network;
  const agents = sim.swarmController?.agents ?? [];
  if (agents.length < 2) {
    return;
  }

  const range = clamp(readNumber(els.communicationRange), 2, 80);
  const requiredNeighbors = Math.min(
    sim.swarmController?.communicationGraph?.maxNeighbors ?? clampInt(readNumber(els.communicationNeighbors), network.minNeighbors, network.maxNeighbors),
    agents.length - 1
  );
  const softMax = range * network.softRangeFactor;

  for (let pass = 0; pass < network.correctionPasses; pass += 1) {
    agents.forEach((agent) => {
      const nearest = agents
        .filter((candidate) => candidate.id !== agent.id)
        .map((candidate) => ({
          agent: candidate,
          distance: agent.position.distanceTo(candidate.position)
        }))
        .sort((a, b) => a.distance - b.distance)
        .slice(0, requiredNeighbors);

      nearest.forEach((neighbor) => {
        if (neighbor.distance <= softMax || neighbor.distance < 1e-6) {
          return;
        }
        const correction = (neighbor.distance - softMax) * network.correctionStrength * roleNetworkCompliance(agent.role);
        temp.travel.copy(neighbor.agent.position).sub(agent.position).normalize();
        agent.setPosition(clampSwarmPosition(agent.position.clone().addScaledVector(temp.travel, correction)));
        neighbor.agent.setPosition(clampSwarmPosition(neighbor.agent.position.clone().addScaledVector(
          temp.travel,
          -correction * network.neighborCounterPush * roleNetworkCompliance(neighbor.agent.role)
        )));
      });
    });
  }
}

function roleNetworkCompliance(role) {
  const network = swarmBehaviorProfile().network;
  const base = role === DRONE_ROLES.RELAY ? network.relayCompliance : role === DRONE_ROLES.SCOUT ? network.scoutCompliance : 1;
  return base * (swarmDerivedControls().networkComplianceScale?.[role] ?? 1);
}

function resolveSwarmCollisions() {
  const agents = sim.swarmController?.agents ?? [];
  const radius = swarmCollisionRadius();
  const minDistance = radius * 2;

  for (let a = 0; a < agents.length; a += 1) {
    for (let b = a + 1; b < agents.length; b += 1) {
      const first = agents[a];
      const second = agents[b];
      temp.travel.copy(first.position).sub(second.position);
      let distance = temp.travel.length();
      if (distance < 1e-6) {
        temp.travel.set(1, 0, 0);
        distance = 1;
      }
      if (distance >= minDistance) {
        continue;
      }
      const push = (minDistance - distance) * 0.5;
      temp.travel.normalize();
      first.setPosition(first.position.clone().addScaledVector(temp.travel, push));
      second.setPosition(second.position.clone().addScaledVector(temp.travel, -push));
    }
  }

  agents.forEach((agent) => agent.setPosition(clampSwarmPosition(agent.position)));
}

function resolveSwarmObstacleAvoidance() {
  const agents = sim.swarmController?.agents ?? [];
  const radius = swarmCollisionRadius();
  agents.forEach((agent) => {
    const next = agent.position.clone();
    sim.floorFootprints.forEach((footprint) => {
      const insideX = next.x > footprint.minX - radius && next.x < footprint.maxX + radius;
      const insideZ = next.z > footprint.minZ - radius && next.z < footprint.maxZ + radius;
      const insideY = next.y < footprint.height + radius;
      if (!insideX || !insideZ || !insideY) {
        return;
      }

      const left = Math.abs(next.x - (footprint.minX - radius));
      const right = Math.abs((footprint.maxX + radius) - next.x);
      const back = Math.abs(next.z - (footprint.minZ - radius));
      const front = Math.abs((footprint.maxZ + radius) - next.z);
      const min = Math.min(left, right, back, front);
      if (min === left) {
        next.x = footprint.minX - radius;
      } else if (min === right) {
        next.x = footprint.maxX + radius;
      } else if (min === back) {
        next.z = footprint.minZ - radius;
      } else {
        next.z = footprint.maxZ + radius;
      }
    });

    if (sim.ceilingHazard) {
      const lateral = Math.hypot(next.x - sim.ceilingHazard.x, next.z - sim.ceilingHazard.z);
      const minLateral = sim.ceilingHazard.radius + radius;
      if (lateral < minLateral && next.y > sim.ceilingHazard.bottomY - radius) {
        const angle = Math.atan2(next.z - sim.ceilingHazard.z, next.x - sim.ceilingHazard.x);
        next.x = sim.ceilingHazard.x + Math.cos(angle) * minLateral;
        next.z = sim.ceilingHazard.z + Math.sin(angle) * minLateral;
      }
    }

    agent.setPosition(clampSwarmPosition(next));
  });
}

function clampSwarmPosition(position) {
  const radius = swarmCollisionRadius();
  const floorY = terrainClearanceY(position.x, position.z, radius + 0.22);
  return new THREE.Vector3(
    clamp(position.x, -sim.room.width * 0.5 + radius, sim.room.width * 0.5 - radius),
    clamp(position.y, floorY, sim.room.height - radius - 0.12),
    clamp(position.z, -sim.room.depth * 0.5 + radius, sim.room.depth * 0.5 - radius)
  );
}

function swarmCollisionRadius() {
  return Math.max(0.32, clamp(readNumber(els.safetyRadius), 0.2, 1.4) * 0.55);
}

function renderCommunicationGraph(edges) {
  main.commGroup.clear();
  if (!edges.length || !sim.swarmController) {
    return;
  }

  const material = new THREE.LineBasicMaterial({
    color: '#ffcf5a',
    transparent: true,
    opacity: 0.72
  });
  const agents = new Map(sim.swarmController.agents.map((agent) => [agent.id, agent]));

  edges.forEach((edge) => {
    const from = agents.get(edge.from);
    const to = agents.get(edge.to);
    if (!from || !to) {
      return;
    }
    const geometry = new THREE.BufferGeometry().setFromPoints([from.position, to.position]);
    main.commGroup.add(new THREE.Line(geometry, material));
  });
}

function sampleFrontierTasks(limit) {
  if (!sim.mapper || !sim.mapper.frontierIndices.length) {
    return [];
  }

  return sim.mapper.frontierIndices.slice(0, limit).map((index) => {
    const cell = unpackIndex(index);
    return {
      id: `frontier-${index}`,
      position: voxelToWorld(cell.col, cell.row, cell.layer),
      priority: 1,
      informationGain: 1
    };
  });
}

function selectedAoiTargets() {
  if (!sim.aoiTargets.length) {
    return [];
  }
  if (els.aoiPreset.value === 'auto') {
    return sim.aoiTargets.map(aoiToTask);
  }
  return sim.aoiTargets.filter((aoi) => aoi.type === els.aoiPreset.value).map(aoiToTask);
}

function aoiToTask(aoi) {
  return {
    id: aoi.id,
    type: 'aoi',
    position: aoi.position,
    priority: aoi.priority,
    informationGain: 1.4,
    label: aoi.label,
    radius: aoi.radius ?? 14
  };
}

function readSwarmConfig() {
  return {
    droneCount: clampInt(readNumber(els.swarmSize), 3, 24),
    communicationRange: clamp(readNumber(els.communicationRange), 2, 80),
    maxNeighbors: clampInt(readNumber(els.communicationNeighbors), 1, 8),
    communicationDropout: clamp(readNumber(els.communicationDropout), 0, 0.45),
    formationMode: els.swarmFormation.value,
    behaviorProfile: activeBehaviorProfile(),
    spacing: 5.6,
    verticalSpan: Math.max(0.8, sim.room.height * 0.28)
  };
}

function activeBehaviorProfile() {
  const objectiveKey = els.objectiveProfile?.value ?? DEFAULT_SWARM_BEHAVIOR_PROFILE.objectives.default;
  const policyCoordinates = readPolicyCoordinates();
  return {
    ...DEFAULT_SWARM_BEHAVIOR_PROFILE,
    modelFamily: GREYBOX_POLICY_MODEL.modelFamily,
    modelVersion: GREYBOX_POLICY_MODEL.modelVersion,
    policyCoordinates,
    runtimeNudgeProfile: els.runtimeNudgeProfile?.value ?? 'current',
    runtimeNudgeScale: runtimeNudgeProfileConfig(els.runtimeNudgeProfile?.value ?? 'current').scale,
    objectives: {
      ...DEFAULT_SWARM_BEHAVIOR_PROFILE.objectives,
      default: objectiveKey
    }
  };
}

function readPolicyCoordinates() {
  return normalizePolicyCoordinates({
    coverage_area: readPolicyCoordinateValue(els.policyCoverageArea, DEFAULT_POLICY_COORDINATES.coverage_area),
    aoi_detail: readPolicyCoordinateValue(els.policyAoiDetail, DEFAULT_POLICY_COORDINATES.aoi_detail),
    risk_safety: readPolicyCoordinateValue(els.policyRiskSafety, DEFAULT_POLICY_COORDINATES.risk_safety),
    resource_efficiency: readPolicyCoordinateValue(els.policyResourceEfficiency, DEFAULT_POLICY_COORDINATES.resource_efficiency)
  });
}

function readPolicyCoordinateValue(input, fallback) {
  const value = input ? readNumber(input) : fallback;
  return Number.isFinite(value) ? clamp(value, 0, 1) : fallback;
}

function applyPolicyCoordinatesToControls(policyCoordinates) {
  const policy = normalizePolicyCoordinates(policyCoordinates);
  const pairs = [
    [els.policyCoverageArea, policy.coverage_area],
    [els.policyAoiDetail, policy.aoi_detail],
    [els.policyRiskSafety, policy.risk_safety],
    [els.policyResourceEfficiency, policy.resource_efficiency]
  ];
  pairs.forEach(([input, value]) => {
    if (input) {
      input.value = value.toFixed(2);
    }
  });
  syncPolicyCoordinateOutputs();
}

function syncPolicyCoordinateOutputs() {
  const policy = readPolicyCoordinates();
  const outputPairs = [
    [els.policyCoverageAreaValue, policy.coverage_area],
    [els.policyAoiDetailValue, policy.aoi_detail],
    [els.policyRiskSafetyValue, policy.risk_safety],
    [els.policyResourceEfficiencyValue, policy.resource_efficiency]
  ];
  outputPairs.forEach(([output, value]) => {
    if (output) {
      output.value = `${Math.round(value * 100)}%`;
      output.textContent = `${Math.round(value * 100)}%`;
    }
  });
}

function applyObjectiveProfileChange() {
  const profile = activeBehaviorProfile();
  if (sim.swarmController) {
    sim.swarmController.setBehaviorProfile(profile);
  }
  noteTelemetryControlChange('objective-profile');
  logMessage(`Objective profile set to ${profile.objectives.profiles[profile.objectives.default]?.label ?? profile.objectives.default}.`);
}

function applyPolicyCoordinateChange(controlId) {
  syncPolicyCoordinateOutputs();
  const profile = activeBehaviorProfile();
  if (sim.swarmController) {
    sim.swarmController.setBehaviorProfile(profile);
  }
  noteTelemetryControlChange(controlId ?? 'policy-coordinates');
  logMessage(`Policy coordinates updated: coverage ${formatScore(profile.policyCoordinates.coverage_area)}, AOI ${formatScore(profile.policyCoordinates.aoi_detail)}, safety ${formatScore(profile.policyCoordinates.risk_safety)}, resource ${formatScore(profile.policyCoordinates.resource_efficiency)}.`);
}

function applyRuntimeNudgeProfileChange(controlId = 'runtime-nudge-profile') {
  const profile = activeBehaviorProfile();
  if (sim.swarmController) {
    sim.swarmController.setBehaviorProfile(profile);
  }
  noteTelemetryControlChange(controlId);
  logMessage(`Runtime nudge profile set to ${runtimeNudgeProfileConfig(profile.runtimeNudgeProfile).label} (${profile.runtimeNudgeScale.toFixed(2)}x).`);
}

function isSwarmMode() {
  return true;
}

function renderScanBeams(segments, scanContext) {
  main.beamGroup.clear();
  if (!segments.length) {
    return;
  }

  const material = new THREE.LineBasicMaterial({
    color:
      scanContext === 'move'
        ? '#6ab7ff'
        : scanContext === 'avoidance'
          ? '#ffb36b'
          : scanContext === 'bootstrap'
            ? '#b99cff'
            : '#63d7ff',
    transparent: true,
    opacity: scanContext === 'swarm' ? 0.22 : 0.34
  });

  segments.forEach(([from, to]) => {
    const geometry = new THREE.BufferGeometry().setFromPoints([from, to]);
    main.beamGroup.add(new THREE.Line(geometry, material));
  });
}

function pointAoiMetadata(point, activeAois, focused) {
  let best = null;
  activeAois.forEach((aoi) => {
    const radius = aoi.radius ?? 14;
    const distance = point.distanceTo(aoi.position);
    if (distance > radius * 1.35) {
      return;
    }
    const strength = clamp(1 - distance / (radius * 1.35), 0, 1);
    if (!best || strength > best.strength) {
      best = { aoiId: aoi.id, strength };
    }
  });

  if (!best) {
    return { aoiId: null, aoiStrength: 0, focused: focused ? 1 : 0 };
  }
  return {
    aoiId: best.aoiId,
    aoiStrength: focused ? Math.max(best.strength, 0.72) : best.strength,
    focused: focused ? 1 : 0
  };
}

function estimateRayFootprint(distance, focused = false) {
  const horizontalCount = clampInt(readNumber(els.horizontalRays), 4, 48);
  const verticalCount = clampInt(readNumber(els.verticalRays), 2, 16);
  const horizontalStep = THREE.MathUtils.degToRad(clamp(readNumber(els.horizontalFov), 90, 360)) / Math.max(horizontalCount - 1, 1);
  const verticalStep = THREE.MathUtils.degToRad(clamp(readNumber(els.verticalFov), 30, 120)) / Math.max(verticalCount - 1, 1);
  const angularStep = Math.max(horizontalStep, verticalStep);
  const focusScale = focused ? 0.62 : 1;
  const radius = clamp(Math.tan(angularStep * 0.5) * distance * focusScale, 0.08, 2.6);
  const targetDetailRadius = 0.38;
  return {
    radius,
    areaM2: Math.PI * radius * radius,
    resolutionScore: clamp(targetDetailRadius / Math.max(radius, targetDetailRadius), 0, 1)
  };
}

function recordFootprintCoverage(point, footprint) {
  const bucket = Math.max(0, Math.min(5, Math.ceil(Math.log2(Math.max(footprint.radius / 0.16, 1)))));
  const cellSize = Math.max(0.45, footprint.radius * 1.35);
  const key = [
    bucket,
    Math.round(point.x / cellSize),
    Math.round(point.y / cellSize),
    Math.round(point.z / cellSize)
  ].join(':');
  const entry = sim.footprintCoverageMap.get(key);
  if (!entry) {
    sim.footprintCoverageMap.set(key, {
      count: 1,
      areaM2: footprint.areaM2,
      radius: footprint.radius
    });
    return {
      newAreaM2: footprint.areaM2,
      redundantAreaM2: 0
    };
  }

  entry.count += 1;
  entry.areaM2 = Math.max(entry.areaM2, footprint.areaM2);
  const novelty = 1 / Math.max(entry.count, 1);
  return {
    newAreaM2: footprint.areaM2 * novelty,
    redundantAreaM2: footprint.areaM2 * (1 - novelty)
  };
}

function addPointToCloud(point, distance, metadata = {}, hit = null) {
  const voxel = clamp(readNumber(els.voxelSize), 0.015, 0.2);
  const key = [
    Math.round(point.x / voxel),
    Math.round(point.y / voxel),
    Math.round(point.z / voxel)
  ].join(':');

  const entry = sim.voxelMap.get(key);
  if (!entry) {
    sim.voxelMap.set(key, {
      sum: point.clone(),
      count: 1,
      distanceSum: distance,
      aoiScore: metadata.aoiStrength ?? 0,
      focusScore: metadata.focused ?? 0
    });
    sim.pointCount = sim.voxelMap.size;
    sim.cloudDirty = true;
    return;
  }

  entry.sum.add(point);
  entry.count += 1;
  entry.distanceSum += distance;
  entry.aoiScore = (entry.aoiScore ?? 0) + (metadata.aoiStrength ?? 0);
  entry.focusScore = (entry.focusScore ?? 0) + (metadata.focused ?? 0);
  sim.cloudDirty = true;
}

function updatePointReadouts() {
  const total = sim.pointCount;
  const displayed = sim.performance.displayedPointCount || total;
  els.pointsReadout.textContent = total.toLocaleString();
  const shown = displayed < total ? displayed : total;
  els.cloudMeta.textContent = `${compactPointCount(shown)} / ${compactPointCount(total)} pts`;
  els.cloudMeta.title = displayed < total
    ? `${displayed.toLocaleString()} / ${total.toLocaleString()} points visible`
    : `${total.toLocaleString()} / ${total.toLocaleString()} points visible`;
}

function compactPointCount(value) {
  const numeric = Math.max(0, Number(value) || 0);
  if (numeric >= 1000000) {
    return `${(numeric / 1000000).toFixed(numeric >= 10000000 ? 0 : 1)}M`;
  }
  if (numeric >= 10000) {
    return `${(numeric / 1000).toFixed(numeric >= 100000 ? 0 : 1)}k`;
  }
  return numeric.toLocaleString();
}

function ensurePointCloudBufferCapacity(pointCount) {
  if (sim.pointCloudBuffer.capacity >= pointCount) {
    return;
  }

  const nextCapacity = Math.max(1024, Math.ceil(pointCount * 1.18));
  sim.pointCloudBuffer.positions = new Float32Array(nextCapacity * 3);
  sim.pointCloudBuffer.colors = new Float32Array(nextCapacity * 3);
  sim.pointCloudBuffer.capacity = nextCapacity;
  sim.performance.cloudBufferCapacity = nextCapacity;
  sim.cloudGeometry.setAttribute('position', new THREE.BufferAttribute(sim.pointCloudBuffer.positions, 3));
  sim.cloudGeometry.setAttribute('color', new THREE.BufferAttribute(sim.pointCloudBuffer.colors, 3));
}

function flushPointCloudGeometry(maxRange) {
  const totalPointCount = sim.voxelMap.size;
  if (!sim.cloudDirty && totalPointCount > 0) {
    return;
  }
  const flushStart = performance.now();
  const pointBudget = clampInt(readNumber(els.visiblePointBudget), 10000, 1200000);
  const sampleStep = Math.max(1, Math.ceil(totalPointCount / Math.max(pointBudget, 1)));
  const targetVisibleCount = Math.ceil(totalPointCount / sampleStep);
  ensurePointCloudBufferCapacity(targetVisibleCount);
  const positions = sim.pointCloudBuffer.positions;
  const colors = sim.pointCloudBuffer.colors;
  let index = 0;
  let displayedPoints = 0;

  sim.voxelMap.forEach((entry) => {
    const shouldDisplay = index % sampleStep === 0;
    index += 1;
    if (!shouldDisplay) {
      return;
    }

    temp.worldPoint.copy(entry.sum).multiplyScalar(1 / entry.count);
    const bufferIndex = displayedPoints * 3;
    positions[bufferIndex] = temp.worldPoint.x;
    positions[bufferIndex + 1] = temp.worldPoint.y;
    positions[bufferIndex + 2] = temp.worldPoint.z;

    const color = pointCloudColor(temp.worldPoint, entry, maxRange);
    colors[bufferIndex] = color.r;
    colors[bufferIndex + 1] = color.g;
    colors[bufferIndex + 2] = color.b;
    displayedPoints += 1;
  });

  const positionAttribute = sim.cloudGeometry.getAttribute('position');
  const colorAttribute = sim.cloudGeometry.getAttribute('color');
  if (positionAttribute) {
    positionAttribute.needsUpdate = true;
  }
  if (colorAttribute) {
    colorAttribute.needsUpdate = true;
  }
  sim.cloudGeometry.setDrawRange(0, displayedPoints);
  sim.cloudGeometry.boundingSphere = new THREE.Sphere(
    new THREE.Vector3(0, sim.room.height * 0.42, 0),
    Math.max(sim.room.width, sim.room.depth, sim.room.height) * 0.9
  );
  sim.performance.displayedPointCount = displayedPoints;
  sim.performance.lastCloudFlushMs = performance.now() - flushStart;
  updatePointReadouts();
  sim.cloudDirty = false;
}

function pointCloudColor(point, entry, maxRange) {
  const heightT = THREE.MathUtils.clamp(point.y / Math.max(sim.room.height, 0.001), 0, 1);
  const distanceT = THREE.MathUtils.clamp((entry.distanceSum / entry.count) / Math.max(maxRange, 0.001), 0, 1);
  const confidenceT = THREE.MathUtils.clamp(entry.count / 8, 0, 1);
  const aoiT = THREE.MathUtils.clamp((entry.aoiScore ?? 0) / Math.max(entry.count, 1), 0, 1);
  const focusT = THREE.MathUtils.clamp((entry.focusScore ?? 0) / Math.max(entry.count, 1), 0, 1);
  const lateralT = THREE.MathUtils.clamp(point.x / Math.max(sim.room.width, 0.001) + 0.5, 0, 1);
  const depthT = THREE.MathUtils.clamp(point.z / Math.max(sim.room.depth, 0.001) + 0.5, 0, 1);
  const microVariation = (Math.sin(point.x * 0.16 + point.z * 0.22) + Math.sin(point.y * 0.34 - point.z * 0.12)) * 0.018;
  const hue = THREE.MathUtils.euclideanModulo(
    0.54 - heightT * 0.3 + lateralT * 0.11 - depthT * 0.15 + (1 - distanceT) * 0.055 + microVariation,
    1
  );
  const saturation = THREE.MathUtils.clamp(0.54 + (1 - distanceT) * 0.2 + Math.abs(lateralT - depthT) * 0.12, 0.44, 0.84);
  const lightness = THREE.MathUtils.clamp(0.28 + heightT * 0.3 + confidenceT * 0.17 + depthT * 0.08 + (1 - distanceT) * 0.05, 0.24, 0.74);
  const baseColor = new THREE.Color().setHSL(hue, saturation, lightness);
  if (aoiT <= 0.01 && focusT <= 0.01) {
    return baseColor;
  }

  const focusColor = aoiObjectScaleColor(point, confidenceT, focusT);
  const highlightStrength = THREE.MathUtils.clamp(aoiT * 0.56 + focusT * 0.18, 0.18, 0.68);
  return baseColor.lerp(focusColor, highlightStrength);
}

function aoiObjectScaleColor(point, confidenceT, focusT) {
  const nearest = nearestPointAoi(point);
  if (!nearest) {
    return new THREE.Color().setHSL(0.9 - focusT * 0.05, 0.9, 0.58 + confidenceT * 0.16);
  }

  const radius = Math.max(nearest.radius ?? 14, 1);
  const localZ = clamp((point.z - nearest.position.z) / radius, -1, 1);
  const localDepth = (localZ + 1) * 0.5;
  const sharpDepth = THREE.MathUtils.smoothstep(localDepth, 0.02, 0.98);
  const band = Math.sin(localZ * Math.PI * 5.4) * 0.035;
  const hue = THREE.MathUtils.euclideanModulo(0.0 + band * 0.08, 1);
  const saturation = THREE.MathUtils.clamp(0.82 + focusT * 0.1 + sharpDepth * 0.1, 0.72, 1);
  const lightness = THREE.MathUtils.clamp(0.2 + sharpDepth * 0.52 + confidenceT * 0.06, 0.18, 0.8);
  return new THREE.Color().setHSL(hue, saturation, lightness);
}

function nearestPointAoi(point) {
  let best = null;
  sim.aoiTargets.forEach((aoi) => {
    const radius = aoi.radius ?? 14;
    const distance = point.distanceTo(aoi.position);
    if (distance > radius * 1.45) {
      return;
    }
    if (!best || distance < best.distance) {
      best = { ...aoi, distance };
    }
  });
  return best;
}

function drawMissionGraph() {
  const { width, height } = els.orbitCanvas;
  orbitCtx.clearRect(0, 0, width, height);
  orbitCtx.fillStyle = '#09111a';
  orbitCtx.fillRect(0, 0, width, height);

  if (!sim.room || !sim.mapper) {
    return;
  }

  const map = { x: 12, y: 12, w: 224, h: height - 24 };
  const ladder = { x: 252, y: 18, w: width - 264, h: height - 36 };

  orbitCtx.strokeStyle = 'rgba(132, 158, 214, 0.16)';
  orbitCtx.lineWidth = 1;
  orbitCtx.strokeRect(map.x, map.y, map.w, map.h);
  orbitCtx.strokeRect(ladder.x, ladder.y, ladder.w, ladder.h);

  const scale = Math.min(map.w / sim.room.width, map.h / sim.room.depth) * 0.94;
  const cx = map.x + map.w * 0.5;
  const cy = map.y + map.h * 0.5;
  const cellScale = sim.mapper.res * scale;

  orbitCtx.fillStyle = 'rgba(101, 128, 178, 0.08)';
  orbitCtx.fillRect(cx - sim.room.width * scale * 0.5, cy - sim.room.depth * scale * 0.5, sim.room.width * scale, sim.room.depth * scale);
  orbitCtx.strokeStyle = 'rgba(105, 138, 210, 0.5)';
  orbitCtx.strokeRect(cx - sim.room.width * scale * 0.5, cy - sim.room.depth * scale * 0.5, sim.room.width * scale, sim.room.depth * scale);

  const projection = new Map();
  for (let layer = 0; layer < sim.mapper.layers; layer += 1) {
    for (let row = 0; row < sim.mapper.rows; row += 1) {
      for (let col = 0; col < sim.mapper.cols; col += 1) {
        const index = packIndex(col, row, layer);
        const state = sim.mapper.states[index];
        if (state === CELL_UNKNOWN) {
          continue;
        }
        const key = `${col}:${row}`;
        const existing = projection.get(key);
        const weight = layer / Math.max(sim.mapper.layers - 1, 1);
        if (!existing || state > existing.state || weight > existing.weight) {
          projection.set(key, { state, weight });
        }
      }
    }
  }

  projection.forEach((entry, key) => {
    const [col, row] = key.split(':').map(Number);
    const x = cx + (sim.mapper.originX + col * sim.mapper.res) * scale;
    const y = cy + (sim.mapper.originZ + row * sim.mapper.res) * scale;
    orbitCtx.fillStyle =
      entry.state === CELL_OCCUPIED
        ? `rgba(255, ${Math.round(120 + entry.weight * 60)}, 168, 0.72)`
        : `rgba(110, 174, 255, ${0.14 + entry.weight * 0.18})`;
    orbitCtx.fillRect(x - cellScale * 0.5, y - cellScale * 0.5, Math.max(cellScale - 1, 1), Math.max(cellScale - 1, 1));
  });

  sim.mapper.frontierIndices.slice(0, 120).forEach((index) => {
    const cell = unpackIndex(index);
    const world = voxelToWorld(cell.col, cell.row, cell.layer);
    orbitCtx.fillStyle = 'rgba(255, 214, 115, 0.94)';
    orbitCtx.beginPath();
    orbitCtx.arc(cx + world.x * scale, cy + world.z * scale, 1.8, 0, Math.PI * 2);
    orbitCtx.fill();
  });

  if (sim.routeSamples.length) {
    orbitCtx.strokeStyle = 'rgba(96, 182, 255, 0.82)';
    orbitCtx.lineWidth = 2;
    orbitCtx.beginPath();
    [main.drone.position, ...sim.routeSamples].forEach((point, index) => {
      const px = cx + point.x * scale;
      const py = cy + point.z * scale;
      if (index === 0) {
        orbitCtx.moveTo(px, py);
      } else {
        orbitCtx.lineTo(px, py);
      }
    });
    orbitCtx.stroke();
  }

  if (sim.currentGoalWorld) {
    orbitCtx.fillStyle = sim.currentGoalType === 'frontier' ? '#ffd26f' : '#7dffd6';
    orbitCtx.beginPath();
    orbitCtx.arc(cx + sim.currentGoalWorld.x * scale, cy + sim.currentGoalWorld.z * scale, 4.6, 0, Math.PI * 2);
    orbitCtx.fill();
  }

  orbitCtx.fillStyle = sim.status === 'paused' ? '#ffd26f' : '#73f2cb';
  orbitCtx.beginPath();
  orbitCtx.arc(cx + main.drone.position.x * scale, cy + main.drone.position.z * scale, 5.5, 0, Math.PI * 2);
  orbitCtx.fill();

  orbitCtx.fillStyle = '#dce8ff';
  orbitCtx.font = '12px Inter, sans-serif';
  orbitCtx.fillText('Top-down voxel projection', map.x + 10, map.y + 18);

  const heights = buildSeedHeights();
  for (let index = 0; index < heights.length; index += 1) {
    const t = heights.length === 1 ? 0.5 : index / (heights.length - 1);
    const y = ladder.y + ladder.h - t * ladder.h;
    orbitCtx.strokeStyle = 'rgba(146, 176, 238, 0.28)';
    orbitCtx.beginPath();
    orbitCtx.moveTo(ladder.x + 12, y);
    orbitCtx.lineTo(ladder.x + ladder.w - 12, y);
    orbitCtx.stroke();

    orbitCtx.fillStyle = index === sim.currentBandIndex ? '#8ef8cf' : '#a0b7e4';
    orbitCtx.beginPath();
    orbitCtx.arc(ladder.x + ladder.w * 0.58, y, index === sim.currentBandIndex ? 5.5 : 3.5, 0, Math.PI * 2);
    orbitCtx.fill();
    orbitCtx.fillText(`L${index + 1}`, ladder.x + 14, y - 8);
  }

  orbitCtx.fillStyle = '#dce8ff';
  orbitCtx.fillText('Altitude bands', ladder.x + 12, ladder.y + 18);
  orbitCtx.fillText(`Frontiers: ${sim.mapper.frontierIndices.length}`, ladder.x + 12, ladder.y + ladder.h - 24);
  orbitCtx.fillText(`Replans: ${sim.plannerStats.replans}`, ladder.x + 12, ladder.y + ladder.h - 8);
}

function refreshStatus() {
  els.modeReadout.textContent = isSwarmMode() ? 'Swarm V1' : els.scanMode.value === 'continuous' ? 'Continuous' : 'Stepped';
  els.stateReadout.textContent = readableStatus(sim.status);
  updatePointReadouts();
  els.scansReadout.textContent = sim.scanCounter.toLocaleString();
  els.seedReadout.textContent = String(sim.environmentSeed);
  els.phaseReadout.textContent = readablePhase(sim.currentPhase);
  els.plannerReadout.textContent = readablePlanner(els.plannerMode.value);
  els.swarmReadout.textContent = isSwarmMode() ? `${clampInt(readNumber(els.swarmSize), 3, 24)} drones` : 'Off';
  els.formationReadout.textContent = readableFormation(sim.swarmSnapshot?.metrics?.formationMode ?? els.swarmFormation.value);
  els.commsReadout.textContent = sim.swarmSnapshot
    ? `${sim.swarmSnapshot.communication.edges.length} links / k${sim.swarmSnapshot.metrics?.adaptiveNeighborTarget ?? readNumber(els.communicationNeighbors)}`
    : '0 links';
  const budget = readPerformanceBudgetConfig();
  const fps = Math.round(1000 / Math.max(sim.performance.frameAvgMs, 1));
  els.budgetReadout.textContent = `${budget.label} ${fps} fps`;
  els.perfReadout.textContent =
    `${fps} fps / ${sim.performance.frameAvgMs.toFixed(1)} ms frame / ${sim.performance.lastCloudFlushMs.toFixed(1)} ms cloud`;
  els.scanPerfReadout.textContent =
    `${sim.performance.scanPassMs.toFixed(1)} ms / ${sim.performance.lastScanRayCount.toLocaleString()} rays / ${sim.performance.lastScanHits.toLocaleString()} hits`;

  const total = sim.mapper ? sim.mapper.states.length : 1;
  const coverage = sim.mapper ? Math.round((sim.mapper.knownCount / Math.max(total, 1)) * 100) : 0;
  els.coverageReadout.textContent = `${coverage}%`;
  els.goalReadout.textContent = sim.currentGoal ? `${sim.plannerStats.goalsReached} + active` : `${sim.plannerStats.goalsReached} done`;
  updatePolicyExperimentButton();
  renderAlgorithmPanel();
}

function logMessage(message) {
  sim.logs.unshift({
    time: new Date().toLocaleTimeString(),
    message
  });
  sim.logs = sim.logs.slice(0, 90);
  els.logWindow.innerHTML = sim.logs
    .map(
      (entry) => `
        <div class="log-entry">
          <time>${entry.time}</time>
          <p>${entry.message}</p>
        </div>
      `
    )
    .join('');
}

function logScanEvent(hits, scanContext) {
  const coverage = sim.mapper ? Math.round((sim.mapper.knownCount / Math.max(sim.mapper.states.length, 1)) * 100) : 0;
  logMessage(
    `Scan ${sim.scanCounter}: ${hits} returns during ${scanContext}. Coverage ${coverage}% with ${sim.mapper.frontierIndices.length} frontiers in 3D space.`
  );
}

function toggleAlgorithmPanel() {
  els.algorithmPanel.hidden = !els.algorithmPanel.hidden;
  if (!els.algorithmPanel.hidden) {
    renderAlgorithmPanel();
  }
}

function renderAlgorithmPanel() {
  if (!els.algorithmPanel || els.algorithmPanel.hidden) {
    return;
  }

  const snapshot = sim.swarmSnapshot ?? sim.swarmController?.snapshot();
  const metrics = snapshot?.metrics ?? {};
  const profile = swarmBehaviorProfile();
  const objectiveKey = profile.objectives.default;
  const objective = profile.objectives.profiles[objectiveKey];
  renderMetricList(els.algorithmWeights, metrics.behaviorWeights ?? {}, { percent: true, sort: true });
  renderMetricList(els.algorithmSignals, metrics.normalizedSignals ?? {}, { percent: true, maxRows: 9 });
  renderMetricList(els.algorithmControls, {
    ...prefixMetricKeys('basePsi', metrics.basePolicyCoordinates ?? profile.basePolicyCoordinates ?? profile.policyCoordinates ?? DEFAULT_POLICY_COORDINATES),
    ...prefixMetricKeys('nudge', metrics.runtimeNudge ?? profile.runtimeNudge ?? {}),
    ...prefixMetricKeys('effectivePsi', metrics.effectivePolicyCoordinates ?? profile.effectivePolicyCoordinates ?? {}),
    ...prefixMetricKeys('deltaPsi', metrics.deltaPolicyCoordinates ?? profile.deltaPolicyCoordinates ?? {}),
    ...flattenAlgorithmControls(metrics.derivedProfileSummary ?? profile.derivedProfileSummary ?? {}),
    ...flattenAlgorithmControls(metrics.derivedControls ?? {})
  }, { maxRows: 16 });
  renderMetricList(els.algorithmRoles, swarmRoleCounts(), { barMax: Math.max(sim.swarmController?.agents?.length ?? 1, 1) });
  renderMetricList(els.algorithmObjective, {
    modelFamily: profile.modelFamily ?? GREYBOX_POLICY_MODEL.modelFamily,
    modelVersion: profile.modelVersion ?? GREYBOX_POLICY_MODEL.modelVersion,
    nudgeProfile: profile.runtimeNudgeProfile ?? 'current',
    nudgeScale: profile.runtimeNudgeScale ?? 1,
    objective: objective?.label ?? objectiveKey,
    profile: profile.version,
    formation: metrics.formationMode ?? els.swarmFormation.value,
    adaptiveK: metrics.adaptiveNeighborTarget ?? readNumber(els.communicationNeighbors)
  });
}

function prefixMetricKeys(prefix, values) {
  return Object.fromEntries(Object.entries(values ?? {}).map(([key, value]) => [`${prefix}.${key}`, value]));
}

function flattenAlgorithmControls(controls) {
  const flattened = {};
  Object.entries(controls ?? {}).forEach(([key, value]) => {
    if (key === 'primitivePressure') {
      return;
    }
    if (value && typeof value === 'object' && !Array.isArray(value)) {
      Object.entries(value).forEach(([childKey, childValue]) => {
        flattened[`${key}.${childKey}`] = childValue;
      });
      return;
    }
    flattened[key] = value;
  });
  return flattened;
}

function renderMetricList(container, values, { percent = false, sort = false, maxRows = 20, barMax = 1 } = {}) {
  if (!container) {
    return;
  }

  const entries = Object.entries(values ?? {})
    .filter(([, value]) => value !== null && value !== undefined)
    .sort((a, b) => sort ? Number(b[1]) - Number(a[1]) : 0)
    .slice(0, maxRows);

  if (!entries.length) {
    container.innerHTML = '<div class="metric-row"><span>No controller data yet</span><strong>--</strong></div>';
    return;
  }

  container.innerHTML = entries.map(([key, value]) => {
    const numeric = typeof value === 'number' && Number.isFinite(value);
    const normalized = numeric ? (percent ? clamp(value, 0, 1) : clamp(value / Math.max(barMax, 1), 0, 1)) : 0;
    return `
      <div class="metric-row">
        <span>${escapeHtml(readableMetricName(key))}</span>
        <strong>${escapeHtml(formatMetricValue(value, percent))}</strong>
        ${numeric ? `<div class="metric-bar"><span style="width:${(normalized * 100).toFixed(1)}%"></span></div>` : ''}
      </div>
    `;
  }).join('');
}

function readableMetricName(key) {
  return String(key)
    .replace(/([a-z])([A-Z])/g, '$1 $2')
    .replace(/[._-]+/g, ' ')
    .replace(/\b\w/g, (letter) => letter.toUpperCase());
}

function formatMetricValue(value, percent = false) {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return String(value ?? '--');
  }
  if (percent) {
    return `${Math.round(clamp(value, 0, 1) * 100)}%`;
  }
  if (Math.abs(value) >= 100) {
    return value.toFixed(0);
  }
  if (Math.abs(value) >= 10) {
    return value.toFixed(1);
  }
  return value.toFixed(2);
}

function formatScore(value) {
  return typeof value === 'number' && Number.isFinite(value) ? `${Math.round(value * 100)}%` : '--';
}

function toggleTelemetryPanel() {
  els.telemetryPanel.hidden = !els.telemetryPanel.hidden;
  if (!els.telemetryPanel.hidden) {
    refreshTelemetryPanel();
  }
}

async function refreshTelemetryPanel() {
  const allRuns = await sim.swarmTelemetry.listRuns({ limit: 50, modelFamily: GREYBOX_POLICY_MODEL.modelFamily });
  const workspaceFilteredRuns = filterTelemetryRunsByWorkspace(allRuns, els.telemetryWorkspaceFilter.value);
  const validCount = workspaceFilteredRuns.filter((run) => run.validity?.complete).length;
  const scenarioFilteredRuns = filterTelemetryRunsByScenario(workspaceFilteredRuns, els.telemetryScenarioFilter.value);
  const runs = sortTelemetryRuns(
    els.telemetryValidOnly.checked ? scenarioFilteredRuns.filter((run) => run.validity?.complete) : scenarioFilteredRuns,
    els.telemetrySort.value
  );
  const latest = workspaceFilteredRuns[0] ?? null;
  els.telemetryRunCount.textContent = workspaceFilteredRuns.length.toLocaleString();
  els.telemetryValidCount.textContent = validCount.toLocaleString();
  els.telemetryLatestHits.textContent = latest ? (latest.totals?.rawHits ?? 0).toLocaleString() : '0';

  if (!runs.length) {
    els.telemetryRuns.innerHTML = workspaceFilteredRuns.length
      ? '<div class="telemetry-run"><strong>No matching runs</strong><span>Try disabling Valid only or changing the sort mode.</span></div>'
      : `<div class="telemetry-run"><strong>No ${escapeHtml(GREYBOX_POLICY_MODEL.modelFamily)} runs yet</strong><span>Old telemetry remains saved but is hidden from this model-family view.</span></div>`;
    return;
  }

  els.telemetryRuns.innerHTML = renderTelemetryRunGroups(runs);
}

function renderTelemetryRunGroups(runs) {
  const groups = [];
  runs.forEach((run) => {
    const experiment = run.scenario?.policyExperiment ?? null;
    const groupKey = experiment?.batchId ?? `single:${run.id}`;
    let group = groups.find((candidate) => candidate.key === groupKey);
    if (!group) {
      group = {
        key: groupKey,
        experiment,
        runs: []
      };
      groups.push(group);
    }
    group.runs.push(run);
  });

  return groups.map((group) => {
    const header = group.experiment
      ? renderTelemetryBatchSummary(group)
      : '';
    return `${header}${group.runs.map(renderTelemetryRunCard).join('')}`;
  }).join('');
}

function renderTelemetryBatchSummary(group) {
  const summary = summarizeTelemetryBatch(group.runs);
  const bestRun = summary.bestRun;
  const lowestLossRun = summary.lowestLossRun;
  return `
    <div class="telemetry-run telemetry-run--batch">
      <strong>${escapeHtml(group.experiment.batchId)}</strong>
      <span>${group.runs.length} / ${group.experiment.total ?? '?'} policy/nudge run(s), ${group.experiment.runSeconds ?? '?'}s each. Valid ${summary.validCount}, invalid ${summary.invalidCount}, cancelled ${summary.cancelledCount}. Averages use valid runs only.</span>
      <span>Batch mode: ${escapeHtml(group.experiment.batchModeLabel ?? policyExperimentModeLabel(group.experiment.batchMode))}</span>
      <span>Avg score ${formatScore(summary.avgScore)}, avg loss ${formatScore(summary.avgLoss)}, avg Pareto: coverage ${formatScore(summary.avgPareto.coverage_area)}, AOI ${formatScore(summary.avgPareto.aoi_detail)}, safety ${formatScore(summary.avgPareto.risk_safety)}, resource ${formatScore(summary.avgPareto.resource_efficiency)}</span>
      <span>Best score: ${bestRun ? escapeHtml(batchRunLabel(bestRun.run)) : '--'} at ${formatScore(bestRun?.scoring.total)}. Lowest loss: ${lowestLossRun ? escapeHtml(batchRunLabel(lowestLossRun.run)) : '--'} at ${formatScore(lowestLossRun?.scoring.paretoLoss)}</span>
      <span>Preset averages: ${renderBatchAverageList(summary.byPreset)}</span>
      <span>Nudge averages: ${renderBatchAverageList(summary.byNudge)}</span>
    </div>
  `;
}

function summarizeTelemetryBatch(runs) {
  const scoredRuns = runs.map((run) => ({
    run,
    scoring: scoreRunForActiveObjective(run)
  }));
  const validScoredRuns = scoredRuns.filter(({ run }) => run.validity?.complete);
  const analysisRuns = validScoredRuns.length ? validScoredRuns : scoredRuns;
  const cancelledCount = runs.filter((run) => run.status === 'cancelled_batch' || run.validity?.flags?.includes('batch_cancelled')).length;
  return {
    validCount: validScoredRuns.length,
    invalidCount: runs.length - validScoredRuns.length,
    cancelledCount,
    avgScore: average(analysisRuns.map(({ scoring }) => scoring.total)),
    avgLoss: average(analysisRuns.map(({ scoring }) => scoring.paretoLoss)),
    avgPareto: averagePareto(analysisRuns),
    bestRun: bestBy(analysisRuns, ({ scoring }) => scoring.total),
    lowestLossRun: bestBy(analysisRuns, ({ scoring }) => -scoring.paretoLoss),
    byPreset: summarizeBatchDimension(analysisRuns, (run) => run.scenario?.policyExperiment?.label ?? 'Manual'),
    byNudge: summarizeBatchDimension(analysisRuns, (run) => run.scenario?.policyExperiment?.nudgeProfileLabel ?? run.runtimeNudgeProfile ?? 'Unknown')
  };
}

function summarizeBatchDimension(scoredRuns, labelForRun) {
  const groups = new Map();
  scoredRuns.forEach((item) => {
    const label = labelForRun(item.run);
    const group = groups.get(label) ?? [];
    group.push(item);
    groups.set(label, group);
  });
  return [...groups.entries()]
    .map(([label, items]) => ({
      label,
      count: items.length,
      avgScore: average(items.map(({ scoring }) => scoring.total)),
      avgLoss: average(items.map(({ scoring }) => scoring.paretoLoss)),
      avgPareto: averagePareto(items)
    }))
    .sort((a, b) => b.avgScore - a.avgScore);
}

function averagePareto(scoredRuns) {
  return {
    coverage_area: average(scoredRuns.map(({ scoring }) => scoring.paretoVector?.coverage_area)),
    aoi_detail: average(scoredRuns.map(({ scoring }) => scoring.paretoVector?.aoi_detail)),
    risk_safety: average(scoredRuns.map(({ scoring }) => scoring.paretoVector?.risk_safety)),
    resource_efficiency: average(scoredRuns.map(({ scoring }) => scoring.paretoVector?.resource_efficiency))
  };
}

function renderBatchAverageList(items) {
  if (!items.length) {
    return '--';
  }
  return items
    .map((item) => `${escapeHtml(item.label)} ${formatScore(item.avgScore)} / loss ${formatScore(item.avgLoss)} / n${item.count}`)
    .join('; ');
}

function batchRunLabel(run) {
  const experiment = run.scenario?.policyExperiment ?? {};
  const label = experiment.label ?? run.status ?? 'run';
  const nudge = experiment.nudgeProfileLabel ?? run.runtimeNudgeProfile ?? run.behaviorProfile?.runtimeNudgeProfile ?? 'nudge';
  return `${label} / ${nudge}`;
}

function bestBy(items, scoreFn) {
  return items.reduce((best, item) => {
    if (!best || scoreFn(item) > scoreFn(best)) {
      return item;
    }
    return best;
  }, null);
}

function average(values) {
  const finite = values.filter((value) => typeof value === 'number' && Number.isFinite(value));
  return finite.length ? finite.reduce((sum, value) => sum + value, 0) / finite.length : 0;
}

function renderTelemetryRunCard(run) {
    const flags = run.validity?.flags?.length ? run.validity.flags.join(', ') : 'valid';
    const aoi = run.scenario?.aoiPreset ?? 'auto';
    const elapsedSeconds = ((run.elapsedMs ?? 0) / 1000).toFixed(1);
    const behavior = run.behaviorProfile?.version ?? 'unknown profile';
    const experiment = run.scenario?.policyExperiment ?? null;
    const scoring = scoreRunForActiveObjective(run);
    const components = scoring.components ?? {};
    const pareto = scoring.paretoVector ?? {};
    const temporal = scoring.temporalMeasures ?? {};
    return `
      <div class="telemetry-run">
        <strong>${escapeHtml(experiment ? `${experiment.index}/${experiment.total} ${experiment.label} / ${experiment.nudgeProfileLabel}` : run.status)} / ${escapeHtml(aoi)} / ${elapsedSeconds}s / score ${formatScore(scoring.total)} / loss ${formatScore(scoring.paretoLoss)} / confidence ${formatScore(scoring.confidence)}</strong>
        <span>${(run.totals?.rawHits ?? 0).toLocaleString()} raw hits, ${(run.totals?.pointVoxels ?? 0).toLocaleString()} point voxels, ${(run.totals?.focusedHits ?? 0).toLocaleString()} focused, k${run.samples?.at?.(-1)?.adaptiveNeighborTarget ?? run.network?.avgAdaptiveNeighborTarget?.toFixed?.(1) ?? '--'}</span>
        <span>Pareto coverage ${formatScore(pareto.coverage_area)}, AOI detail ${formatScore(pareto.aoi_detail)}, safety ${formatScore(pareto.risk_safety)}, resource ${formatScore(pareto.resource_efficiency)}</span>
        <span>Nudge ${escapeHtml(run.runtimeNudgeProfile ?? run.behaviorProfile?.runtimeNudgeProfile ?? '--')} (${formatMetricValue(run.runtimeNudgeScale ?? run.behaviorProfile?.runtimeNudgeScale ?? 0)}x), avg |delta psi| ${formatMetricValue(run.nudgeSummary?.avgAbsDeltaPsi ?? 0)}, max ${formatMetricValue(run.nudgeSummary?.maxAbsDeltaPsi ?? 0)}</span>
        ${experiment ? `<span>Policy batch ${escapeHtml(experiment.id)} / ${escapeHtml(experiment.batchId)}</span>` : ''}
        <span>AOI ${formatScore(components.aoiQuality)}, coverage ${formatScore(components.coverage)}, network ${formatScore(components.networkResilience)}, compute ${formatScore(components.computeEfficiency)}, energy ${formatScore(components.energyProxy)}, smooth ${formatScore(components.adaptationSmoothness)}, safety ${formatScore(components.constraintSafety)}</span>
        <span>${formatMetricValue(temporal.avgNewPointVoxelsPerSecond ?? 0)} new vox/s, ${formatMetricValue(temporal.avgUniqueFootprintAreaPerSecond ?? 0)} m2/s footprint, ${formatScore(1 - (temporal.avgFootprintRedundancyRatio ?? 0))} non-overlap</span>
        <span>${formatMetricValue(temporal.usefulAoiHitRateAfterContact ?? 0)} AOI hits/s after contact, ${formatMetricValue(temporal.avgBehaviorWeightChangePerSecond ?? 0)} weight-change/s, ${formatMetricValue(temporal.riskExposureSeconds ?? 0)} risk s</span>
        <span>${escapeHtml(flags)} / ${escapeHtml(behavior)}</span>
      </div>
    `;
}

function filterTelemetryRunsByScenario(runs, scenarioFilter) {
  if (!scenarioFilter || scenarioFilter === 'all') {
    return runs;
  }
  return runs.filter((run) => (run.scenario?.aoiPreset ?? 'auto') === scenarioFilter);
}

function filterTelemetryRunsByWorkspace(runs, workspaceFilter = 'active') {
  if (!workspaceFilter || workspaceFilter === 'active') {
    const activeWorkspace = datasetWorkspaceName();
    return runs.filter((run) => (run.scenario?.datasetWorkspace ?? 'legacy') === activeWorkspace);
  }
  if (workspaceFilter === 'all') {
    return runs;
  }
  if (workspaceFilter === 'legacy') {
    return runs.filter((run) => !run.scenario?.datasetWorkspace);
  }
  return runs.filter((run) => run.scenario?.datasetWorkspace === workspaceFilter);
}

function scoreRunForActiveObjective(run) {
  return scoreSwarmRun({
    ...run,
    behaviorProfile: {
      ...(run.behaviorProfile ?? {}),
      objective: els.objectiveProfile.value
    }
  }, swarmBehaviorProfile());
}

function sortTelemetryRuns(runs, sortMode) {
  const copy = [...runs];
  if (sortMode === 'recent') {
    return copy;
  }
  return copy.sort((a, b) => {
    const scoreA = scoreRunForActiveObjective(a);
    const scoreB = scoreRunForActiveObjective(b);
    const valueA = sortMode === 'score'
      ? scoreA.total
      : sortMode === 'confidence'
        ? scoreA.confidence
        : scoreA.components?.[sortMode] ?? 0;
    const valueB = sortMode === 'score'
      ? scoreB.total
      : sortMode === 'confidence'
        ? scoreB.confidence
        : scoreB.components?.[sortMode] ?? 0;
    return valueB - valueA;
  });
}

async function exportTelemetryRuns() {
  const runs = await telemetryRunsForExport();
  if (!runs.length) {
    logMessage('No telemetry runs saved yet.');
    return;
  }

  const blob = new Blob([JSON.stringify(runs, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = `swarm-telemetry-${safeFilenamePart(datasetWorkspaceName())}-${GREYBOX_POLICY_MODEL.modelFamily}-${GREYBOX_POLICY_MODEL.modelVersion}-${sim.environmentSeed}.json`;
  anchor.click();
  URL.revokeObjectURL(url);
  logMessage(`Telemetry exported with ${runs.length.toLocaleString()} saved run(s).`);
}

async function exportTelemetryRunsCsv() {
  const runs = await telemetryRunsForExport();
  if (!runs.length) {
    logMessage('No telemetry runs saved yet.');
    return;
  }

  const rows = runs.map((run) => telemetryRunCsvRow(run));
  const csv = csvStringify(rows);
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = `swarm-runs-${safeFilenamePart(datasetWorkspaceName())}-${GREYBOX_POLICY_MODEL.modelFamily}-${GREYBOX_POLICY_MODEL.modelVersion}-${sim.environmentSeed}.csv`;
  anchor.click();
  URL.revokeObjectURL(url);
  logMessage(`Telemetry CSV exported with ${runs.length.toLocaleString()} run row(s).`);
}

async function telemetryRunsForExport() {
  const runs = await sim.swarmTelemetry.exportRuns({ modelFamily: GREYBOX_POLICY_MODEL.modelFamily });
  const workspaceFiltered = filterTelemetryRunsByWorkspace(runs, els.telemetryWorkspaceFilter.value);
  return filterTelemetryRunsByScenario(workspaceFiltered, els.telemetryScenarioFilter.value);
}

function telemetryRunCsvRow(run) {
  const scoring = scoreRunForActiveObjective(run);
  const experiment = run.scenario?.policyExperiment ?? {};
  const pareto = scoring.paretoVector ?? {};
  const components = scoring.components ?? {};
  const temporal = run.temporalSummary ?? scoring.temporalMeasures ?? {};
  const nudge = run.nudgeSummary ?? {};
  return {
    run_id: run.id,
    dataset_workspace: run.scenario?.datasetWorkspace ?? datasetWorkspaceName(),
    model_family: run.modelFamily ?? GREYBOX_POLICY_MODEL.modelFamily,
    model_version: run.modelVersion ?? GREYBOX_POLICY_MODEL.modelVersion,
    status: run.status,
    valid: Boolean(run.validity?.complete),
    validity_flags: (run.validity?.flags ?? []).join('|'),
    batch_id: experiment.batchId ?? '',
    batch_mode: experiment.batchMode ?? '',
    batch_index: experiment.index ?? '',
    batch_total: experiment.total ?? '',
    policy_preset: experiment.label ?? '',
    nudge_profile: experiment.nudgeProfile ?? run.runtimeNudgeProfile ?? '',
    nudge_scale: experiment.nudgeScale ?? run.runtimeNudgeScale ?? '',
    objective_profile: run.behaviorProfile?.objective ?? '',
    aoi_preset: run.scenario?.aoiPreset ?? '',
    aoi_count: run.aoi?.selectedCount ?? 0,
    nearest_aoi_distance: run.scenario?.aoiNearestDistance ?? '',
    elapsed_s: (run.elapsedMs ?? 0) / 1000,
    raw_hits: run.totals?.rawHits ?? 0,
    focused_hits: run.totals?.focusedHits ?? 0,
    aoi_hits: run.totals?.aoiHits ?? 0,
    point_voxels: run.totals?.pointVoxels ?? 0,
    unique_footprint_area_m2: run.totals?.uniqueFootprintAreaM2 ?? 0,
    redundant_footprint_area_m2: run.totals?.redundantFootprintAreaM2 ?? 0,
    score: scoring.total,
    pareto_loss: scoring.paretoLoss,
    confidence: scoring.confidence,
    pareto_coverage_area: pareto.coverage_area ?? 0,
    pareto_aoi_detail: pareto.aoi_detail ?? 0,
    pareto_risk_safety: pareto.risk_safety ?? 0,
    pareto_resource_efficiency: pareto.resource_efficiency ?? 0,
    component_aoi_quality: components.aoiQuality ?? 0,
    component_coverage: components.coverage ?? 0,
    component_network_resilience: components.networkResilience ?? 0,
    component_compute_efficiency: components.computeEfficiency ?? 0,
    component_energy_proxy: components.energyProxy ?? 0,
    component_adaptation_smoothness: components.adaptationSmoothness ?? 0,
    component_constraint_safety: components.constraintSafety ?? 0,
    psi_coverage_area: run.policyCoordinates?.coverage_area ?? '',
    psi_aoi_detail: run.policyCoordinates?.aoi_detail ?? '',
    psi_risk_safety: run.policyCoordinates?.risk_safety ?? '',
    psi_resource_efficiency: run.policyCoordinates?.resource_efficiency ?? '',
    effective_psi_coverage_area: run.effectivePolicyCoordinates?.coverage_area ?? '',
    effective_psi_aoi_detail: run.effectivePolicyCoordinates?.aoi_detail ?? '',
    effective_psi_risk_safety: run.effectivePolicyCoordinates?.risk_safety ?? '',
    effective_psi_resource_efficiency: run.effectivePolicyCoordinates?.resource_efficiency ?? '',
    avg_abs_delta_psi: nudge.avgAbsDeltaPsi ?? temporal.avgAbsDeltaPsi ?? 0,
    max_abs_delta_psi: nudge.maxAbsDeltaPsi ?? temporal.maxAbsDeltaPsi ?? 0,
    trajectory_samples: run.samples?.filter((sample) => sample.droneTrajectories?.length).length ?? 0,
    avg_drone_path_length_m: run.trajectorySummary?.avgDronePathLength ?? run.movement?.avgDronePathLength ?? 0,
    max_drone_path_length_m: run.trajectorySummary?.maxDronePathLength ?? run.movement?.maxDronePathLength ?? 0,
    path_imbalance: run.trajectorySummary?.pathImbalance ?? run.movement?.pathImbalance ?? 0,
    avg_new_voxels_per_s: temporal.avgNewPointVoxelsPerSecond ?? 0,
    avg_aoi_hits_per_s: temporal.avgAoiHitsPerSecond ?? 0,
    useful_aoi_hits_per_s_after_contact: temporal.usefulAoiHitRateAfterContact ?? 0,
    network_fragmented_s: temporal.networkFragmentedSeconds ?? 0,
    risk_exposure_s: temporal.riskExposureSeconds ?? 0,
    battery_remaining_pct: run.resources?.batteryRemainingPct ?? '',
    total_energy_wh: run.resources?.totalEnergyWh ?? '',
    path_length_m: run.movement?.pathLength ?? ''
  };
}

function csvStringify(rows) {
  if (!rows.length) {
    return '';
  }
  const headers = Object.keys(rows[0]);
  const lines = [
    headers.join(','),
    ...rows.map((row) => headers.map((header) => csvCell(row[header])).join(','))
  ];
  return lines.join('\n');
}

function csvCell(value) {
  if (value === null || value === undefined) {
    return '';
  }
  const text = String(value);
  return /[",\n\r]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
}

function safeFilenamePart(value) {
  return String(value || 'dataset')
    .trim()
    .replace(/[^a-z0-9._-]+/gi, '-')
    .replace(/^-+|-+$/g, '')
    .slice(0, 80) || 'dataset';
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function exportPointCloudAsPly() {
  if (!sim.voxelMap.size) {
    logMessage('No point cloud data yet. Run a mission before exporting.');
    return;
  }

  const maxRange = currentSensorConfig().maxRange;
  const lines = [];
  sim.voxelMap.forEach((entry) => {
    const point = entry.sum.clone().multiplyScalar(1 / entry.count);
    const color = pointCloudColor(point, entry, maxRange);
    lines.push(
      `${point.x.toFixed(5)} ${point.y.toFixed(5)} ${point.z.toFixed(5)} ${Math.round(color.r * 255)} ${Math.round(color.g * 255)} ${Math.round(color.b * 255)}`
    );
  });

  const header = [
    'ply',
    'format ascii 1.0',
    `element vertex ${lines.length}`,
    'property float x',
    'property float y',
    'property float z',
    'property uchar red',
    'property uchar green',
    'property uchar blue',
    'end_header'
  ].join('\n');

  const blob = new Blob([`${header}\n${lines.join('\n')}\n`], { type: 'text/plain' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = `terrain-swarm-point-cloud-${sim.environmentSeed}.ply`;
  anchor.click();
  URL.revokeObjectURL(url);
  logMessage(`PLY exported with ${lines.length.toLocaleString()} raw points.`);
}

function makeOverlayDraggable() {
  let dragState = null;

  els.cloudHeader.addEventListener('pointerdown', (event) => {
    const rect = els.cloudOverlay.getBoundingClientRect();
    dragState = {
      offsetX: event.clientX - rect.left,
      offsetY: event.clientY - rect.top
    };
    els.cloudHeader.setPointerCapture(event.pointerId);
  });

  els.cloudHeader.addEventListener('pointermove', (event) => {
    if (!dragState) {
      return;
    }

    const nextLeft = clamp(event.clientX - dragState.offsetX, 8, window.innerWidth - els.cloudOverlay.offsetWidth - 8);
    const nextTop = clamp(event.clientY - dragState.offsetY, 8, window.innerHeight - els.cloudOverlay.offsetHeight - 8);

    els.cloudOverlay.style.left = `${nextLeft}px`;
    els.cloudOverlay.style.top = `${nextTop}px`;
    els.cloudOverlay.style.right = 'auto';
    els.cloudOverlay.style.bottom = 'auto';
  });

  const endDrag = (event) => {
    if (dragState) {
      els.cloudHeader.releasePointerCapture(event.pointerId);
    }
    dragState = null;
  };

  els.cloudHeader.addEventListener('pointerup', endDrag);
  els.cloudHeader.addEventListener('pointercancel', endDrag);
}

function pickFurnitureTemplate(rng, templates) {
  const totalWeight = templates.reduce((sum, template) => sum + template.weight, 0);
  let threshold = rng() * totalWeight;
  for (const template of templates) {
    threshold -= template.weight;
    if (threshold <= 0) {
      return template;
    }
  }
  return templates[templates.length - 1];
}

function sampleFurniturePosition(rng, dims, wallBias, halfW, halfD) {
  const edgeInset = 0.38;
  if (rng() < wallBias) {
    const wall = Math.floor(rng() * 4);
    if (wall === 0) {
      return {
        x: lerp(-halfW + dims.width * 0.5 + edgeInset, halfW - dims.width * 0.5 - edgeInset, rng()),
        z: -halfD + dims.depth * 0.5 + edgeInset
      };
    }
    if (wall === 1) {
      return {
        x: lerp(-halfW + dims.width * 0.5 + edgeInset, halfW - dims.width * 0.5 - edgeInset, rng()),
        z: halfD - dims.depth * 0.5 - edgeInset
      };
    }
    if (wall === 2) {
      return {
        x: -halfW + dims.width * 0.5 + edgeInset,
        z: lerp(-halfD + dims.depth * 0.5 + edgeInset, halfD - dims.depth * 0.5 - edgeInset, rng())
      };
    }
    return {
      x: halfW - dims.width * 0.5 - edgeInset,
      z: lerp(-halfD + dims.depth * 0.5 + edgeInset, halfD - dims.depth * 0.5 - edgeInset, rng())
    };
  }

  return {
    x: lerp(-halfW + dims.width * 0.5 + edgeInset, halfW - dims.width * 0.5 - edgeInset, rng()),
    z: lerp(-halfD + dims.depth * 0.5 + edgeInset, halfD - dims.depth * 0.5 - edgeInset, rng())
  };
}

function overlapsFootprints(candidate, footprints, padding) {
  return footprints.some((footprint) => {
    return !(
      candidate.maxX + padding < footprint.minX ||
      candidate.minX - padding > footprint.maxX ||
      candidate.maxZ + padding < footprint.minZ ||
      candidate.minZ - padding > footprint.maxZ
    );
  });
}

function rotatedSize(width, depth, rotation) {
  const quarterTurn = Math.abs(rotation - Math.PI / 2) < 1e-6;
  return quarterTurn ? { width: depth, depth: width } : { width, depth };
}

function collectMeshes(group) {
  const meshes = [];
  group.traverse((child) => {
    if (child.isMesh && !child.userData.ignoreRaycast) {
      meshes.push(child);
    }
  });
  return meshes;
}

function prepareRaycastAcceleration(meshes) {
  meshes.forEach((mesh) => {
    if (!mesh.geometry || mesh.geometry.boundsTree) {
      return;
    }
    mesh.geometry.computeBoundsTree();
  });
}

function worldToVoxel(x, y, z) {
  if (!sim.mapper) {
    return null;
  }
  return sim.mapper.worldToVoxel(x, y, z);
}

function voxelToWorld(col, row, layer) {
  return sim.mapper.voxelToWorld(col, row, layer);
}

function packIndex(col, row, layer) {
  return sim.mapper.packIndex(col, row, layer);
}

function unpackIndex(index) {
  return sim.mapper.unpackIndex(index);
}

function voxelInBounds(col, row, layer) {
  return sim.mapper.inBounds(col, row, layer);
}

function getCellState(col, row, layer) {
  return sim.mapper.getState(col, row, layer);
}

function nearestSeedBand(y, heights) {
  let best = 0;
  let bestDistance = Infinity;
  heights.forEach((height, index) => {
    const distance = Math.abs(height - y);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = index;
    }
  });
  return best;
}

function readableStatus(status) {
  switch (status) {
    case 'running':
      return 'Running';
    case 'paused':
      return 'Paused';
    case 'complete':
      return 'Complete';
    default:
      return 'Idle';
  }
}

function readablePhase(phase) {
  switch (phase) {
    case 'bootstrap':
      return 'Bootstrap';
    case 'frontier':
      return 'Frontier';
    case 'inspect':
      return 'Inspect';
    case 'avoid':
      return 'Avoid';
    case 'launch':
      return 'Launch';
    case 'swarm':
      return 'Swarm';
    case 'complete':
      return 'Complete';
    default:
      return 'Idle';
  }
}

function readablePlanner(mode) {
  switch (mode) {
    case 'weighted':
      return 'WA*';
    case 'greedy':
      return 'Greedy';
    default:
      return 'A*';
  }
}

function readableFormation(mode) {
  switch (mode) {
    case FORMATION_MODES.SCATTER:
      return 'Scatter';
    case FORMATION_MODES.LINE_SWEEP:
      return 'Line sweep';
    case FORMATION_MODES.WEDGE:
      return 'Wedge';
    case FORMATION_MODES.RELAY_CHAIN:
      return 'Relay chain';
    case FORMATION_MODES.PERIMETER_RING:
      return 'Perimeter ring';
    default:
      return 'Adaptive';
  }
}

function roleColor(role) {
  switch (role) {
    case 'mapper':
      return '#7dffd6';
    case 'relay':
      return '#ffd26f';
    case 'verifier':
      return '#ff9ccc';
    default:
      return '#79b8ff';
  }
}
