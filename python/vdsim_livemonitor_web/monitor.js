// VDSim live monitor page: draws the state streamed by vdsim_livemonitor.
// World frame = simulation frame: x east, y north, z up [m].
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const SKY = 0x87ceeb;
const CHECKER_M = 5;            // one square [m]
const GROUND_M = 1200;          // ground plane edge [m]
const TRAIL_MAX = 3000;         // trail points kept
// Draw order of the flat layers on the ground. They are drawn without depth
// test, one after the other, so a coarse depth buffer (software renderers) can
// not make them flicker into each other; the vehicle is drawn after them.
const ORDER = { ground: -4, road: -3, line: -2, trail: -1 };
const RECONNECT_MS = 1000;      // retry period while the server is down
const SILENT_MS = 3000;         // no message this long -> reconnect

const view = document.getElementById('view');
const hud = document.getElementById('hud');
const statusEl = document.getElementById('status');
const wait = document.getElementById('wait');
const waitMsg = document.getElementById('waitmsg');

// Exposed for automated checks (headless browser).
window.__monitor = { states: 0, live: false, connected: false, sceneVersion: -1, roads: 0 };

// ---- renderer / scene ----------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(SKY);
scene.fog = new THREE.Fog(SKY, 250, 600);
scene.add(new THREE.HemisphereLight(0xffffff, 0x9a9a9a, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(-40, -60, 120);
scene.add(sun);

const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 2000);
camera.position.set(-14, 0, 6);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 1);
controls.maxPolarAngle = Math.PI * 0.495;
controls.update();

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

// ---- ground: white / grey checker, follows the vehicle on a 2-square grid --
function checkerTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 2;
  const g = c.getContext('2d');
  g.fillStyle = '#ffffff'; g.fillRect(0, 0, 2, 2);
  g.fillStyle = '#bdbdbd'; g.fillRect(0, 0, 1, 1); g.fillRect(1, 1, 1, 1);
  const t = new THREE.CanvasTexture(c);
  t.magFilter = THREE.NearestFilter;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(GROUND_M / (2 * CHECKER_M), GROUND_M / (2 * CHECKER_M));
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return t;
}
const ground = new THREE.Mesh(new THREE.PlaneGeometry(GROUND_M, GROUND_M),
                              new THREE.MeshLambertMaterial({ map: checkerTexture() }));
ground.renderOrder = ORDER.ground;
scene.add(ground);

// ---- roads (OSM) ---------------------------------------------------------
const roads = new THREE.Group();
scene.add(roads);

function ribbon(pts, width, z) {
  const pos = [];
  const half = width / 2;
  for (let i = 0; i + 1 < pts.length; i++) {
    const [x0, y0] = pts[i], [x1, y1] = pts[i + 1];
    const dx = x1 - x0, dy = y1 - y0, L = Math.hypot(dx, dy);
    if (L < 1e-6) continue;
    const nx = -dy / L * half, ny = dx / L * half;
    pos.push(x0 + nx, y0 + ny, z, x0 - nx, y0 - ny, z, x1 + nx, y1 + ny, z,
             x1 + nx, y1 + ny, z, x0 - nx, y0 - ny, z, x1 - nx, y1 - ny, z);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  return g;
}

function polyline(pts, z) {
  return new THREE.BufferGeometry().setFromPoints(pts.map(([x, y]) => new THREE.Vector3(x, y, z)));
}

const flat = { depthTest: false, depthWrite: false };
const roadMat = new THREE.MeshBasicMaterial({ color: 0x55595f, side: THREE.DoubleSide, ...flat });
const centerMat = new THREE.LineBasicMaterial({ color: 0xe0b000, ...flat });
const laneMat = new THREE.LineBasicMaterial({ color: 0x202020, ...flat });
const virtualMat = new THREE.LineDashedMaterial({ color: 0x6a6a6a, dashSize: 1.5, gapSize: 1.5, ...flat });

function layer(obj, order) { obj.renderOrder = order; return obj; }

function buildRoads(list) {
  for (const o of [...roads.children]) { roads.remove(o); o.geometry.dispose(); }
  for (const r of list) {
    if (r.kind === 'highway') {
      roads.add(layer(new THREE.Mesh(ribbon(r.pts, r.width, 0.02), roadMat), ORDER.road));
      roads.add(layer(new THREE.Line(polyline(r.pts, 0.04), centerMat), ORDER.line));
    } else {
      const line = new THREE.Line(polyline(r.pts, 0.04), r.cls === 'virtual' ? virtualMat : laneMat);
      if (r.cls === 'virtual') line.computeLineDistances();
      roads.add(layer(line, ORDER.line));
    }
  }
}

// ---- vehicle -------------------------------------------------------------
const car = new THREE.Group();      // x, y, yaw
const body = new THREE.Group();     // roll, pitch
car.add(body);
scene.add(car);
const frontPivots = [];

function buildCar(v) {
  for (const o of [...body.children]) body.remove(o);
  for (const o of [...car.children]) if (o !== body) car.remove(o);
  frontPivots.length = 0;
  const front = v.cg_to_front + v.front_overhang, rear = v.cg_to_rear + v.rear_overhang;
  const len = front + rear, cx = (front - rear) / 2;
  // Body sits on the axle height and is narrower than the wheel tracks, so
  // the wheels (and the front steer) stay visible from the chase camera.
  const bodyW = Math.min(v.width, v.track) * 0.9, base = v.wheel_radius;
  const lowerH = Math.max(v.height - base, 0.3) * 0.55;
  const lower = new THREE.Mesh(new THREE.BoxGeometry(len, bodyW, lowerH),
                               new THREE.MeshLambertMaterial({ color: 0xd23b2f }));
  lower.position.set(cx, 0, base + lowerH / 2);
  const cabinH = Math.max(v.height - base, 0.3) * 0.45;
  const cabin = new THREE.Mesh(new THREE.BoxGeometry(len * 0.55, bodyW * 0.9, cabinH),
                               new THREE.MeshLambertMaterial({ color: 0x2b3a4a }));
  cabin.position.set(cx - len * 0.05, 0, base + lowerH + cabinH / 2);
  body.add(lower, cabin);
  const wheelGeo = new THREE.CylinderGeometry(v.wheel_radius, v.wheel_radius, v.wheel_width, 24);
  const wheelMat = new THREE.MeshLambertMaterial({ color: 0x1b1b1b });
  for (const [x, isFront] of [[v.cg_to_front, true], [-v.cg_to_rear, false]]) {
    for (const side of [1, -1]) {
      const pivot = new THREE.Group();
      pivot.position.set(x, side * v.track / 2, v.wheel_radius);
      pivot.add(new THREE.Mesh(wheelGeo, wheelMat));   // cylinder axis = y = axle
      car.add(pivot);
      if (isFront) frontPivots.push(pivot);
    }
  }
}

// trail
const trailPos = new Float32Array(TRAIL_MAX * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trailGeo.setDrawRange(0, 0);
scene.add(layer(new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x1f5fbf, ...flat })),
                ORDER.trail));
let trailN = 0;

function addTrail(x, y) {
  if (trailN > 0) {
    const i = (trailN - 1) * 3;
    if (Math.hypot(trailPos[i] - x, trailPos[i + 1] - y) < 0.2) return;
  }
  if (trailN === TRAIL_MAX) { trailPos.copyWithin(0, 3); trailN--; }
  trailPos.set([x, y, 0.06], trailN * 3);
  trailN++;
  trailGeo.setDrawRange(0, trailN);
  trailGeo.attributes.position.needsUpdate = true;
  trailGeo.computeBoundingSphere();
}

// ---- camera follow -------------------------------------------------------
let chase = true;
const last = new THREE.Vector3();
let haveState = false;
controls.addEventListener('start', () => { chase = false; });
document.getElementById('follow').addEventListener('click', () => { chase = true; });
window.addEventListener('keydown', (e) => { if (e.key === 'f' || e.key === 'F') chase = true; });

function followCamera() {
  const p = car.position, yaw = car.rotation.z;
  if (chase) {
    camera.position.set(p.x - 14 * Math.cos(yaw), p.y - 14 * Math.sin(yaw), p.z + 6);
    controls.target.set(p.x, p.y, p.z + 1);
  } else {
    const d = new THREE.Vector3().subVectors(p, last);
    camera.position.add(d);
    controls.target.add(d);
  }
  last.copy(p);
  ground.position.set(Math.round(p.x / (2 * CHECKER_M)) * 2 * CHECKER_M,
                      Math.round(p.y / (2 * CHECKER_M)) * 2 * CHECKER_M, 0);
  controls.update();
}

// ---- state ---------------------------------------------------------------
function num(v, d) { return (typeof v === 'number' && isFinite(v)) ? v.toFixed(d) : '--'; }

function applyState(s) {
  if (typeof s.x !== 'number' || typeof s.y !== 'number' || typeof s.yaw !== 'number') return;
  if (!haveState) { last.set(s.x, s.y, 0); haveState = true; }
  car.position.set(s.x, s.y, 0);
  car.rotation.set(0, 0, s.yaw);
  body.rotation.set(s.roll || 0, s.pitch || 0, 0, 'ZYX');
  for (const p of frontPivots) p.rotation.z = s.delta || 0;
  addTrail(s.x, s.y);
  const v = s.vx;
  hud.textContent = `t ${num(s.t, 2)} s | v ${num(v, 2)} m/s (${num(typeof v === 'number' ? v * 3.6 : v, 1)} km/h)`
                  + ` | steer ${num(typeof s.delta === 'number' ? s.delta * 180 / Math.PI : s.delta, 2)} deg`
                  + (osmError ? `\nOSM: ${osmError}` : '');
  window.__monitor.states++;
}

let osmError = null;
let sceneVersion = -1;

async function loadScene(version) {
  if (version === sceneVersion) return;
  try {
    const r = await fetch('./scene.json', { cache: 'no-store' });
    const sc = await r.json();
    sceneVersion = sc.version;
    buildCar(sc.vehicle);
    buildRoads(sc.roads || []);
    osmError = sc.osm_error;
    window.__monitor.sceneVersion = sc.version;
    window.__monitor.roads = (sc.roads || []).length;
  } catch (e) { /* retried on the next status */ }
}

function setStatus(connected, live) {
  window.__monitor.connected = connected;
  window.__monitor.live = live;
  statusEl.className = connected ? (live ? 'live' : '') : 'down';
  statusEl.textContent = connected ? (live ? 'live' : 'connected, no simulator') : 'reconnecting';
  wait.classList.toggle('hidden', connected && live);
  waitMsg.textContent = connected ? 'waiting for simulator' : 'monitor server unreachable, retrying';
}

// ---- WebSocket with automatic reconnect ------------------------------------
let ws = null, lastMsg = 0;

function connect() {
  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
  ws = new WebSocket(url);
  ws.onopen = () => { lastMsg = performance.now(); setStatus(true, false); };
  ws.onmessage = (ev) => {
    lastMsg = performance.now();
    let m;
    try { m = JSON.parse(ev.data); } catch (e) { return; }
    if (m.type === 'hello' || m.type === 'status') {
      setStatus(true, !!m.live);
      loadScene(m.scene_version);
    } else if (m.type === 'state') {
      if (!window.__monitor.live) setStatus(true, true);
      applyState(m.state);
    }
  };
  ws.onclose = () => { ws = null; setStatus(false, false); setTimeout(connect, RECONNECT_MS); };
  ws.onerror = () => { try { ws.close(); } catch (e) { /* closing */ } };
}

setInterval(() => {
  if (ws && ws.readyState === WebSocket.OPEN && performance.now() - lastMsg > SILENT_MS) ws.close();
}, 1000);

buildCar({ cg_to_front: 1.35, cg_to_rear: 1.65, track: 1.6, front_overhang: 0.9,
           rear_overhang: 0.95, width: 1.85, height: 1.45, wheel_radius: 0.33, wheel_width: 0.24 });
setStatus(false, false);
connect();

renderer.setAnimationLoop(() => {
  followCamera();
  renderer.render(scene, camera);
});
