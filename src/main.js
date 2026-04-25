import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import './styles.css';
import { CELL_FREE, CELL_OCCUPIED, CELL_UNKNOWN, SENSOR_PRESETS } from './core/constants.js';
import { VoxelGrid } from './core/voxel-grid.js';
import { searchPath3D as runVoxelPlanner } from './core/planner.js';
import { almostEqual, clamp, clampInt, createRng, generateSeed, lerp, readNumber } from './core/math.js';

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
  resetBtn: document.querySelector('#resetBtn'),
  randomizeBtn: document.querySelector('#randomizeBtn'),
  exportBtn: document.querySelector('#exportBtn'),
  roomPreset: document.querySelector('#roomPreset'),
  scanMode: document.querySelector('#scanMode'),
  sensorPreset: document.querySelector('#sensorPreset'),
  plannerMode: document.querySelector('#plannerMode'),
  pathStyle: document.querySelector('#pathStyle'),
  ceilingObstacles: document.querySelector('#ceilingObstacles'),
  roomWidth: document.querySelector('#roomWidth'),
  roomDepth: document.querySelector('#roomDepth'),
  roomHeight: document.querySelector('#roomHeight'),
  furnitureCount: document.querySelector('#furnitureCount'),
  heightBands: document.querySelector('#heightBands'),
  bootstrapScans: document.querySelector('#bootstrapScans'),
  safetyRadius: document.querySelector('#safetyRadius'),
  goalReachRadius: document.querySelector('#goalReachRadius'),
  gridSize: document.querySelector('#gridSize'),
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
  scanTargets: [],
  floorFootprints: [],
  ceilingHazard: null,
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
  voxelMap: new Map(),
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
  probeOrigin: new THREE.Vector3()
};

const main = createMainScene();
const cloud = createCloudScene();

setupUI();
createPointCloud();
applySensorPreset(false);
regenerateEnvironment(true);
resetMission(false, true);
logMessage('Room ready. The drone now maps in 3D voxel space and plans with clearance-aware exploration goals.');
animate(0);

function createMainScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#08111d');
  scene.fog = new THREE.Fog('#08111d', 14, 38);

  const camera = new THREE.PerspectiveCamera(52, 1, 0.1, 140);
  camera.position.set(8.5, 6.4, 9.4);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  els.viewport.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
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
  scene.add(roomGroup, pathGroup, beamGroup);

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
  window.addEventListener('resize', resize);
  resize();

  return { scene, camera, renderer, controls, roomGroup, pathGroup, beamGroup, drone };
}

function createCloudScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#060a12');

  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 160);
  camera.position.set(6, 5, 6);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  els.cloudViewport.append(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.target.set(0, 1.2, 0);

  scene.add(new THREE.HemisphereLight('#f5fbff', '#223049', 1.15));

  const fill = new THREE.DirectionalLight('#91bfff', 0.78);
  fill.position.set(6, 8, 5);
  scene.add(fill);

  scene.add(new THREE.GridHelper(16, 32, '#5176af', '#1e2f49'));
  scene.add(new THREE.AxesHelper(1.8));

  const resize = () => {
    const width = els.cloudViewport.clientWidth;
    const height = els.cloudViewport.clientHeight;
    camera.aspect = width / Math.max(height, 1);
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  };

  new ResizeObserver(resize).observe(els.cloudOverlay);
  window.addEventListener('resize', resize);
  resize();

  return { scene, camera, renderer, controls };
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

function createPointCloud() {
  const cloudPoints = new THREE.Points(sim.cloudGeometry, sim.cloudMaterial);
  cloud.scene.add(cloudPoints);
  sim.overlayCloud = cloudPoints;
}

function setupUI() {
  els.startBtn.addEventListener('click', () => startMission());
  els.pauseBtn.addEventListener('click', () => pauseMission());
  els.continueBtn.addEventListener('click', () => continueMission());
  els.resetBtn.addEventListener('click', () => resetMission(true, false));
  els.randomizeBtn.addEventListener('click', () => {
    regenerateEnvironment(true);
    resetMission(false, false);
    logMessage(`Room layout randomized. Seed ${sim.environmentSeed}.`);
  });
  els.exportBtn.addEventListener('click', () => exportPointCloudAsPly());

  els.sensorPreset.addEventListener('change', () => {
    applySensorPreset(true);
    resetMission(false, false);
    logMessage(`Sensor preset set to ${SENSOR_PRESETS[els.sensorPreset.value].label}.`);
  });

  [
    els.roomPreset,
    els.roomWidth,
    els.roomDepth,
    els.roomHeight,
    els.furnitureCount,
    els.ceilingObstacles
  ].forEach((input) => {
    input.addEventListener('change', () => {
      regenerateEnvironment(false);
      resetMission(false, false);
      logMessage('Room geometry updated from the controls.');
    });
  });

  [
    els.scanMode,
    els.plannerMode,
    els.pathStyle,
    els.heightBands,
    els.bootstrapScans,
    els.safetyRadius,
    els.goalReachRadius,
    els.gridSize
  ].forEach((input) => {
    input.addEventListener('change', () => {
      initializeMapper();
      resetMission(false, false);
      logMessage('Planner and mapping settings updated.');
    });
  });

  [els.moveSpeed, els.horizontalRays, els.verticalRays, els.horizontalFov, els.verticalFov, els.maxRange, els.voxelSize, els.dwellMs].forEach(
    (input) => {
      input.addEventListener('change', () => {
        refreshStatus();
        drawMissionGraph();
      });
    }
  );

  makeOverlayDraggable();
  refreshStatus();
}

function applySensorPreset(writeLog) {
  const preset = SENSOR_PRESETS[els.sensorPreset.value] || SENSOR_PRESETS.spinning;
  const custom = els.sensorPreset.value === 'custom';
  if (!custom) {
    els.horizontalRays.value = String(preset.horizontalRays);
    els.verticalRays.value = String(preset.verticalRays);
    els.horizontalFov.value = String(preset.horizontalFov);
    els.verticalFov.value = String(preset.verticalFov);
    els.maxRange.value = String(preset.maxRange);
  }
  [els.horizontalRays, els.verticalRays, els.horizontalFov, els.verticalFov, els.maxRange].forEach((input) => {
    input.disabled = !custom;
  });
  if (writeLog) {
    logMessage(`Sensor tuned for ${preset.label}.`);
  }
}

function currentSensorConfig() {
  const preset = SENSOR_PRESETS[els.sensorPreset.value] || SENSOR_PRESETS.spinning;
  return {
    label: preset.label,
    mode: preset.mode,
    horizontalRays: clampInt(readNumber(els.horizontalRays), 16, 220),
    verticalRays: clampInt(readNumber(els.verticalRays), 8, 96),
    horizontalFov: clamp(readNumber(els.horizontalFov), 45, 360),
    verticalFov: clamp(readNumber(els.verticalFov), 12, 130),
    maxRange: clamp(readNumber(els.maxRange), 2, 20),
    pitchBiasRad: THREE.MathUtils.degToRad(preset.pitchBiasDeg)
  };
}

function regenerateEnvironment(randomizeSeed) {
  if (randomizeSeed || !sim.environmentSeed) {
    sim.environmentSeed = generateSeed();
  }

  const rng = createRng(sim.environmentSeed);
  sim.room = {
    preset: els.roomPreset.value,
    width: clamp(readNumber(els.roomWidth), 5, 14),
    depth: clamp(readNumber(els.roomDepth), 5, 14),
    height: clamp(readNumber(els.roomHeight), 2.6, 5)
  };

  sim.floorFootprints = [];
  sim.ceilingHazard = null;
  sim.scanTargets = [];
  main.roomGroup.clear();
  main.pathGroup.clear();
  main.beamGroup.clear();

  const roomScene = new THREE.Group();
  buildRoomShell(sim.room).forEach((mesh) => roomScene.add(mesh));

  const furnitureGroup = new THREE.Group();
  const furnitureCount = clampInt(readNumber(els.furnitureCount), 3, 14);
  buildFurnitureLayout(rng, furnitureCount).forEach((item) => {
    furnitureGroup.add(item.group);
    sim.floorFootprints.push(item.footprint);
  });
  roomScene.add(furnitureGroup);

  if (els.ceilingObstacles.value === 'on') {
    const hazard = buildCeilingHazard(rng);
    if (hazard) {
      roomScene.add(hazard.group);
      sim.ceilingHazard = hazard;
    }
  }

  main.roomGroup.add(roomScene);
  sim.scanTargets = collectMeshes(roomScene);
  initializeMapper();
  fitCamerasToRoom();
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
  const res = clamp(readNumber(els.gridSize), 0.45, 1.6);
  const margin = res;
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
  main.controls.target.set(0, sim.room.height * 0.42, 0);
  main.camera.position.set(radius, sim.room.height * 1.4, radius * 1.08);
  main.controls.update();

  cloud.controls.target.set(0, sim.room.height * 0.35, 0);
  cloud.camera.position.set(radius * 0.78, sim.room.height * 1.22, radius * 0.78);
  cloud.controls.update();
}

function buildSeedHeights() {
  const levels = clampInt(readNumber(els.heightBands), 2, 8);
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
  const count = clampInt(readNumber(els.bootstrapScans), 1, 12);
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
}

function clearPointCloud() {
  sim.voxelMap.clear();
  sim.pointCount = 0;
  sim.cloudGeometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
  sim.cloudGeometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
  sim.cloudGeometry.computeBoundingSphere();
  sim.cloudMaterial.size = Math.max(0.028, clamp(readNumber(els.voxelSize), 0.015, 0.2) * 1.18);
  els.pointsReadout.textContent = '0';
  els.cloudMeta.textContent = '0 pts';
}

function startMission() {
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
  logMessage('Mission paused.');
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

function animate(time) {
  requestAnimationFrame(animate);

  const deltaMs = sim.lastTime ? time - sim.lastTime : 16;
  sim.lastTime = time;
  const deltaSeconds = deltaMs / 1000;

  updateDrone(deltaSeconds, deltaMs);
  main.controls.update();
  cloud.controls.update();

  main.renderer.render(main.scene, main.camera);
  cloud.renderer.render(cloud.scene, cloud.camera);
}

function updateDrone(deltaSeconds, deltaMs) {
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

  const goalReach = clamp(readNumber(els.goalReachRadius), 0.1, 0.9);

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
      addPointToCloud(hit.point, hit.distance);
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
            : '#7dffd6',
    transparent: true,
    opacity: 0.34
  });

  segments.forEach(([from, to]) => {
    const geometry = new THREE.BufferGeometry().setFromPoints([from, to]);
    main.beamGroup.add(new THREE.Line(geometry, material));
  });
}

function addPointToCloud(point, distance) {
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
      distanceSum: distance
    });
    sim.pointCount = sim.voxelMap.size;
    return;
  }

  entry.sum.add(point);
  entry.count += 1;
  entry.distanceSum += distance;
}

function flushPointCloudGeometry(maxRange) {
  const positions = [];
  const colors = [];

  sim.voxelMap.forEach((entry) => {
    temp.worldPoint.copy(entry.sum).multiplyScalar(1 / entry.count);
    positions.push(temp.worldPoint.x, temp.worldPoint.y, temp.worldPoint.z);

    const heightT = THREE.MathUtils.clamp(temp.worldPoint.y / Math.max(sim.room.height, 0.001), 0, 1);
    const distanceT = THREE.MathUtils.clamp((entry.distanceSum / entry.count) / Math.max(maxRange, 0.001), 0, 1);
    const confidenceT = THREE.MathUtils.clamp(entry.count / 6, 0, 1);
    const hue = 0.62 - heightT * 0.46 + (1 - distanceT) * 0.08;
    const saturation = 0.52 + (1 - distanceT) * 0.33;
    const lightness = 0.28 + confidenceT * 0.22 + heightT * 0.18;
    const color = new THREE.Color().setHSL(hue, saturation, lightness);
    colors.push(color.r, color.g, color.b);
  });

  sim.cloudGeometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  sim.cloudGeometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  sim.cloudGeometry.computeBoundingSphere();
  els.pointsReadout.textContent = sim.pointCount.toLocaleString();
  els.cloudMeta.textContent = `${sim.pointCount.toLocaleString()} pts`;
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
  orbitCtx.fillText('Vertical seed levels', ladder.x + 12, ladder.y + 18);
  orbitCtx.fillText(`Frontiers: ${sim.mapper.frontierIndices.length}`, ladder.x + 12, ladder.y + ladder.h - 24);
  orbitCtx.fillText(`Replans: ${sim.plannerStats.replans}`, ladder.x + 12, ladder.y + ladder.h - 8);
}

function refreshStatus() {
  els.modeReadout.textContent = els.scanMode.value === 'continuous' ? 'Continuous' : 'Stepped';
  els.stateReadout.textContent = readableStatus(sim.status);
  els.pointsReadout.textContent = sim.pointCount.toLocaleString();
  els.scansReadout.textContent = sim.scanCounter.toLocaleString();
  els.cloudMeta.textContent = `${sim.pointCount.toLocaleString()} pts`;
  els.seedReadout.textContent = String(sim.environmentSeed);
  els.phaseReadout.textContent = readablePhase(sim.currentPhase);
  els.plannerReadout.textContent = readablePlanner(els.plannerMode.value);

  const total = sim.mapper ? sim.mapper.states.length : 1;
  const coverage = sim.mapper ? Math.round((sim.mapper.knownCount / Math.max(total, 1)) * 100) : 0;
  els.coverageReadout.textContent = `${coverage}%`;
  els.goalReadout.textContent = sim.currentGoal ? `${sim.plannerStats.goalsReached} + active` : `${sim.plannerStats.goalsReached} done`;
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

function exportPointCloudAsPly() {
  if (!sim.voxelMap.size) {
    logMessage('No point cloud data yet. Run a mission before exporting.');
    return;
  }

  const maxRange = currentSensorConfig().maxRange;
  const lines = [];
  sim.voxelMap.forEach((entry) => {
    const point = entry.sum.clone().multiplyScalar(1 / entry.count);
    const heightT = THREE.MathUtils.clamp(point.y / Math.max(sim.room.height, 0.001), 0, 1);
    const distanceT = THREE.MathUtils.clamp((entry.distanceSum / entry.count) / Math.max(maxRange, 0.001), 0, 1);
    const confidenceT = THREE.MathUtils.clamp(entry.count / 6, 0, 1);
    const color = new THREE.Color().setHSL(
      0.62 - heightT * 0.46 + (1 - distanceT) * 0.08,
      0.52 + (1 - distanceT) * 0.33,
      0.28 + confidenceT * 0.22 + heightT * 0.18
    );
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
  anchor.download = `room-point-cloud-${sim.environmentSeed}.ply`;
  anchor.click();
  URL.revokeObjectURL(url);
  logMessage(`PLY exported with ${lines.length.toLocaleString()} points.`);
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
    if (child.isMesh) {
      meshes.push(child);
    }
  });
  return meshes;
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
