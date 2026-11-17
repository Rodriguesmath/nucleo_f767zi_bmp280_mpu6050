import './style.css';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { createAircraft } from './aircraft';
import { FlightHud } from './hud';
import { TelemetryClient, type BridgeStatus, type TransportMode } from './telemetry';

type CameraPreset = 'chase' | 'orbit' | 'cockpit' | 'top';

const sceneCanvas = document.getElementById('scene-canvas') as HTMLCanvasElement;
const pfdCanvas = document.getElementById('pfd-canvas') as HTMLCanvasElement;

const renderer = new THREE.WebGLRenderer({
  canvas: sceneCanvas,
  antialias: true,
  powerPreference: 'high-performance',
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x07101f);
scene.fog = new THREE.FogExp2(0x07101f, 0.011);

const camera = new THREE.PerspectiveCamera(58, window.innerWidth / window.innerHeight, 0.1, 400);
camera.position.set(0, 3.2, 8.2);

const controls = new OrbitControls(camera, sceneCanvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.enabled = false;

// Lighting
const hemiLight = new THREE.HemisphereLight(0x7dd3fc, 0x0f172a, 1.1);
scene.add(hemiLight);

const sunLight = new THREE.DirectionalLight(0xffffff, 2.2);
sunLight.position.set(14, 24, 12);
sunLight.castShadow = true;
scene.add(sunLight);

// Ground grid & altitude reference rings
const groundGrid = new THREE.GridHelper(220, 55, 0x38bdf8, 0x1e293b);
groundGrid.position.y = -6;
scene.add(groundGrid);

const altRingsGroup = new THREE.Group();
const ringGeo = new THREE.RingGeometry(6.8, 7.0, 48);
ringGeo.rotateX(-Math.PI / 2);
const ringMat = new THREE.MeshBasicMaterial({
  color: 0x0ea5e9,
  side: THREE.DoubleSide,
  transparent: true,
  opacity: 0.25,
});
for (let altM = -10; altM <= 80; altM += 5) {
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.position.y = altM * 0.4;
  altRingsGroup.add(ring);
}
scene.add(altRingsGroup);

// Procedural aircraft
const aircraft = createAircraft();
scene.add(aircraft.root);

// HUD & Sparklines
const hud = new FlightHud(
  pfdCanvas,
  document.getElementById('chart-alt') as HTMLCanvasElement,
  document.getElementById('chart-gyro') as HTMLCanvasElement,
  document.getElementById('chart-accel') as HTMLCanvasElement
);
hud.resizePfd(window.innerWidth, window.innerHeight);

// Telemetry transport
const telemetry = new TelemetryClient();

// UI Elements
const statusDot = document.getElementById('status-dot') as HTMLElement;
const portSelect = document.getElementById('port-select') as HTMLSelectElement;
const baudSelect = document.getElementById('baud-select') as HTMLSelectElement;
const btnConnect = document.getElementById('btn-connect') as HTMLButtonElement;
const btnWebSerial = document.getElementById('btn-webserial') as HTMLButtonElement;
const btnSim = document.getElementById('btn-sim') as HTMLButtonElement;
const btnTarePose = document.getElementById('btn-tare-pose') as HTMLButtonElement;
const btnZeroAlt = document.getElementById('btn-zero-alt') as HTMLButtonElement;
const camSelect = document.getElementById('cam-select') as HTMLSelectElement;
const btnHudToggle = document.getElementById('btn-hud-toggle') as HTMLButtonElement;
const qnhInput = document.getElementById('qnh-input') as HTMLInputElement;
const serialLog = document.getElementById('serial-log') as HTMLPreElement;

const domVals = {
  roll: document.getElementById('val-roll')!,
  pitch: document.getElementById('val-pitch')!,
  yaw: document.getElementById('val-yaw')!,
  ax: document.getElementById('val-ax')!,
  ay: document.getElementById('val-ay')!,
  az: document.getElementById('val-az')!,
  gx: document.getElementById('val-gx')!,
  gy: document.getElementById('val-gy')!,
  gz: document.getElementById('val-gz')!,
  gload: document.getElementById('val-gload')!,
  mpuTemp: document.getElementById('val-mpu-temp')!,
  press: document.getElementById('val-press')!,
  alt: document.getElementById('val-alt')!,
  relAlt: document.getElementById('val-rel-alt')!,
  vsi: document.getElementById('val-vsi')!,
  bmpTemp: document.getElementById('val-bmp-temp')!,
  rho: document.getElementById('val-rho')!,
  dt: document.getElementById('val-dt')!,
  mode: document.getElementById('val-mode')!,
  rate: document.getElementById('val-rate')!,
  fps: document.getElementById('val-fps')!,
};

let cameraPreset: CameraPreset = 'chase';
let bridgeConnected = false;

const appendLog = (line: string): void => {
  const lines = serialLog.textContent ? serialLog.textContent.split('\n') : [];
  lines.push(line);
  if (lines.length > 40) lines.shift();
  serialLog.textContent = lines.join('\n');
  serialLog.scrollTop = serialLog.scrollHeight;
};

telemetry.onLog = appendLog;

telemetry.onModeChange = (mode: TransportMode) => {
  statusDot.className = 'dot';
  if (mode === 'bridge' || mode === 'webserial') {
    statusDot.classList.add('ok');
  } else if (mode === 'simulator') {
    statusDot.classList.add('sim');
  }
  btnSim.classList.toggle('active', mode === 'simulator');
  domVals.mode.textContent = mode.toUpperCase();
};

telemetry.onStatus = (st: BridgeStatus) => {
  bridgeConnected = st.connected;
  btnConnect.textContent = st.connected ? 'Desconectar' : 'Conectar Bridge';
  const prev = portSelect.value;
  portSelect.innerHTML = '';
  if (st.ports.length === 0) {
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = 'Sem porta serial';
    portSelect.appendChild(opt);
  } else {
    for (const p of st.ports) {
      const opt = document.createElement('option');
      opt.value = p.device;
      opt.textContent = p.description ? `${p.device} (${p.description.slice(0, 18)})` : p.device;
      portSelect.appendChild(opt);
    }
    if (st.port) portSelect.value = st.port;
    else if (prev) portSelect.value = prev;
  }
  if (st.error) appendLog(`[ERR] ${st.error}`);
};

btnConnect.addEventListener('click', () => {
  if (bridgeConnected) {
    telemetry.sendBridgeCommand({ type: 'disconnect' });
  } else {
    telemetry.sendBridgeCommand({
      type: 'connect',
      port: portSelect.value || null,
      baud: Number(baudSelect.value),
    });
  }
});

btnWebSerial.addEventListener('click', () => {
  void telemetry.connectWebSerial(Number(baudSelect.value));
});

btnSim.addEventListener('click', () => telemetry.toggleSimulator());
btnTarePose.addEventListener('click', () => telemetry.tarePose());
btnZeroAlt.addEventListener('click', () => telemetry.zeroAltitude());
camSelect.addEventListener('change', () => {
  cameraPreset = camSelect.value as CameraPreset;
  controls.enabled = cameraPreset === 'orbit';
});
btnHudToggle.addEventListener('click', () => {
  hud.showPfd = !hud.showPfd;
  btnHudToggle.classList.toggle('active', hud.showPfd);
});
qnhInput.addEventListener('change', () => {
  telemetry.setQnh(parseFloat(qnhInput.value));
});

window.addEventListener('resize', () => {
  const w = window.innerWidth;
  const h = window.innerHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
  hud.resizePfd(w, h);
});

// Start WebSocket bridge listener using dynamic /ws-config.json port
async function initTransport(): Promise<void> {
  const qsPort = new URLSearchParams(window.location.search).get('ws');
  let wsPort = qsPort ? Number(qsPort) : 8765;
  if (!qsPort) {
    try {
      const resp = await fetch('/ws-config.json', { cache: 'no-store' });
      if (resp.ok) {
        const cfg = (await resp.json()) as { wsPort?: number };
        if (typeof cfg.wsPort === 'number') {
          wsPort = cfg.wsPort;
        }
      }
    } catch {
      // Keep default 8765 during standalone vite dev
    }
  }
  const wsHost = window.location.hostname || '127.0.0.1';
  telemetry.startWebSocket(`ws://${wsHost}:${wsPort}`);
}
void initTransport();

// Preallocated math objects (zero GC in animation loop)
const targetEuler = new THREE.Euler(0, 0, 0, 'YXZ');
const targetQuat = new THREE.Quaternion();
const camDesiredPos = new THREE.Vector3();
const camLookTarget = new THREE.Vector3();
const cockpitOffset = new THREE.Vector3(0, 0.55, -0.35);
const cockpitForward = new THREE.Vector3(0, 0.35, -14);

let prevTime = performance.now();
let chartTimer = 0;
let domTimer = 0;
let fpsFrames = 0;
let fpsAccum = 0;
let currentFps = 60;

function animate(nowMs: number): void {
  requestAnimationFrame(animate);
  const dtSec = Math.min(0.1, Math.max(0.001, (nowMs - prevTime) / 1000));
  prevTime = nowMs;

  telemetry.stepSimulator(dtSec);
  const d = telemetry.data;

  // Map aerospace (Roll, Pitch, Yaw) to Three.js aircraft frame (-Z forward, +Y up, +X right)
  targetEuler.set(
    THREE.MathUtils.degToRad(d.pitchDeg),
    THREE.MathUtils.degToRad(-d.yawDeg),
    THREE.MathUtils.degToRad(-d.rollDeg),
    'YXZ'
  );
  targetQuat.setFromEuler(targetEuler);

  const slerpAlpha = 1 - Math.exp(-dtSec * 22);
  aircraft.root.quaternion.slerp(targetQuat, slerpAlpha);

  // Move aircraft vertically according to BMP280 relative hypsometric altitude (clamped visual scale)
  const targetAltY = THREE.MathUtils.clamp(d.relAltM * 0.4, -4.5, 45);
  aircraft.root.position.y += (targetAltY - aircraft.root.position.y) * slerpAlpha;

  // Scroll ground grid lines smoothly forward to simulate flight motion
  groundGrid.position.z = ((nowMs * 0.008) % 4) - 2;

  aircraft.updateControlSurfaces(
    dtSec,
    d.gyroXDps,
    d.gyroYDps,
    d.gyroZDps,
    d.accelXG,
    d.accelYG,
    d.accelZG,
    d.vsiMs,
    nowMs / 1000
  );

  // Camera tracking modes
  const planePos = aircraft.root.position;
  if (cameraPreset === 'chase') {
    camDesiredPos.set(0, 2.4, 7.2).applyQuaternion(aircraft.root.quaternion).add(planePos);
    camDesiredPos.y = Math.max(groundGrid.position.y + 0.8, camDesiredPos.y);
    camera.position.lerp(camDesiredPos, 1 - Math.exp(-dtSec * 8));
    camLookTarget.set(0, 0.2, -3).applyQuaternion(aircraft.root.quaternion).add(planePos);
    camera.lookAt(camLookTarget);
  } else if (cameraPreset === 'cockpit') {
    camDesiredPos.copy(cockpitOffset).applyQuaternion(aircraft.root.quaternion).add(planePos);
    camera.position.copy(camDesiredPos);
    camLookTarget.copy(cockpitForward).applyQuaternion(aircraft.root.quaternion).add(planePos);
    camera.lookAt(camLookTarget);
  } else if (cameraPreset === 'top') {
    camDesiredPos.set(planePos.x, planePos.y + 12, planePos.z + 0.01);
    camera.position.lerp(camDesiredPos, 1 - Math.exp(-dtSec * 8));
    camera.lookAt(planePos);
  } else {
    controls.target.copy(planePos);
    controls.update();
  }

  renderer.render(scene, camera);

  // Push chart samples at ~30 Hz and HUD PFD at full FPS
  chartTimer += dtSec;
  if (chartTimer >= 0.033) {
    chartTimer = 0;
    hud.pushSample(d);
  }
  hud.render(d, window.innerWidth, window.innerHeight);

  // Batch DOM updates at ~15 Hz to keep main thread lean
  fpsFrames++;
  fpsAccum += dtSec;
  if (fpsAccum >= 0.5) {
    currentFps = fpsFrames / fpsAccum;
    fpsFrames = 0;
    fpsAccum = 0;
  }

  domTimer += dtSec;
  if (domTimer >= 0.066) {
    domTimer = 0;
    domVals.roll.textContent = `${d.rollDeg.toFixed(1)}°`;
    domVals.pitch.textContent = `${d.pitchDeg.toFixed(1)}°`;
    domVals.yaw.textContent = `${(((d.yawDeg % 360) + 360) % 360).toFixed(1)}°`;
    domVals.ax.textContent = `${d.accelXG >= 0 ? '+' : ''}${d.accelXG.toFixed(2)} g`;
    domVals.ay.textContent = `${d.accelYG >= 0 ? '+' : ''}${d.accelYG.toFixed(2)} g`;
    domVals.az.textContent = `${d.accelZG >= 0 ? '+' : ''}${d.accelZG.toFixed(2)} g`;
    domVals.gx.textContent = `${d.gyroXDps >= 0 ? '+' : ''}${d.gyroXDps.toFixed(1)} dps`;
    domVals.gy.textContent = `${d.gyroYDps >= 0 ? '+' : ''}${d.gyroYDps.toFixed(1)} dps`;
    domVals.gz.textContent = `${d.gyroZDps >= 0 ? '+' : ''}${d.gyroZDps.toFixed(1)} dps`;
    domVals.gload.textContent = `${d.gLoad.toFixed(2)} g`;
    domVals.mpuTemp.textContent = `${d.mpuTempC.toFixed(1)} degC`;
    domVals.press.textContent = `${d.bmpPressHpa.toFixed(2)} hPa`;
    domVals.alt.textContent = `${d.hypsometricAltM.toFixed(2)} m`;
    domVals.relAlt.textContent = `${d.relAltM >= 0 ? '+' : ''}${d.relAltM.toFixed(2)} m`;
    domVals.vsi.textContent = `${d.vsiMs >= 0 ? '+' : ''}${d.vsiMs.toFixed(2)} m/s`;
    domVals.bmpTemp.textContent = `${d.bmpTempC.toFixed(2)} degC`;
    domVals.rho.textContent = `${d.airDensityKgM3.toFixed(3)} kg/m³`;
    domVals.dt.textContent = `${d.deltaTempC >= 0 ? '+' : ''}${d.deltaTempC.toFixed(1)} degC`;
    domVals.rate.textContent = `${d.packetRateHz.toFixed(0)} Hz`;
    domVals.fps.textContent = `${currentFps.toFixed(0)} FPS`;
  }
}

requestAnimationFrame(animate);
