// VDSim live monitor page: draws the vehicles streamed by vdsim_livemonitor.
// World frame = simulation frame: x east, y north, z up [m].
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const SKY = 0x87ceeb;
const CHECKER_M = 5;            // one square [m]
const GROUND_M = 1200;          // ground plane edge [m]
const TRAIL_MAX = 3000;         // trail points kept per vehicle
// Draw order of the flat layers on the ground. They are drawn without depth
// test, one after the other, so a coarse depth buffer (software renderers) can
// not make them flicker into each other; the vehicles are drawn after them.
const ORDER = { ground: -4, road: -3, line: -2, trail: -1 };
const RECONNECT_MS = 1000;      // retry period while the server is down
const SILENT_MS = 3000;         // no message this long -> reconnect
const LEGACY_ID = 'ego';        // vehicle of a single-state publisher
// Body colour per palette index (world "color"); index 0 is the
// single-vehicle red. 16 distinct hues, then it repeats.
const PALETTE = [0xd23b2f, 0x2f6fd2, 0x2fa84f, 0xe09a1a, 0x8e44ad, 0x17a2b8,
                 0xe84393, 0x6d4c41, 0x34495e, 0xc0ca33, 0x00897b, 0x5c6bc0,
                 0xff7043, 0x7cb342, 0xab47bc, 0x9e9d24];
const DEFAULT_VEHICLE = { cg_to_front: 1.35, cg_to_rear: 1.65, track: 1.6, front_overhang: 0.9,
                          rear_overhang: 0.95, width: 1.85, height: 1.45, wheel_radius: 0.33,
                          wheel_width: 0.24 };

const view = document.getElementById('view');
const hud = document.getElementById('hud');
const statusEl = document.getElementById('status');
const wait = document.getElementById('wait');
const waitMsg = document.getElementById('waitmsg');
const params = new URLSearchParams(location.search);

// Exposed for automated checks (headless browser).
window.__monitor = { states: 0, live: false, connected: false, sceneVersion: -1, roads: 0,
                     world: null, vehicles: 0, ids: [], trails: {}, rate: null, follow: null,
                     view: 'chase' };

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

// ---- ground: white / grey checker, follows the camera on a 2-square grid ---
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

// ---- vehicles ------------------------------------------------------------
// id -> { car (x, y, yaw), body (roll, pitch), pivots, trail, color, offset,
//         resets, has (a state arrived), s (last state) }
const vehicles = new Map();
let geometry = { ...DEFAULT_VEHICLE };
const cabinMat = new THREE.MeshLambertMaterial({ color: 0x2b3a4a });
const wheelMat = new THREE.MeshLambertMaterial({ color: 0x1b1b1b });

function buildCar(V) {
  const v = geometry, car = V.car, body = V.body;
  for (const o of [...body.children]) body.remove(o);
  for (const o of [...car.children]) if (o !== body) car.remove(o);
  V.pivots = [];
  const front = v.cg_to_front + v.front_overhang, rear = v.cg_to_rear + v.rear_overhang;
  const len = front + rear, cx = (front - rear) / 2;
  // Body sits on the axle height and is narrower than the wheel tracks, so
  // the wheels (and the front steer) stay visible from the chase camera.
  const bodyW = Math.min(v.width, v.track) * 0.9, base = v.wheel_radius;
  const lowerH = Math.max(v.height - base, 0.3) * 0.55;
  const lower = new THREE.Mesh(new THREE.BoxGeometry(len, bodyW, lowerH),
                               new THREE.MeshLambertMaterial({ color: PALETTE[V.color % PALETTE.length] }));
  lower.position.set(cx, 0, base + lowerH / 2);
  const cabinH = Math.max(v.height - base, 0.3) * 0.45;
  const cabin = new THREE.Mesh(new THREE.BoxGeometry(len * 0.55, bodyW * 0.9, cabinH), cabinMat);
  cabin.position.set(cx - len * 0.05, 0, base + lowerH + cabinH / 2);
  body.add(lower, cabin);
  const wheelGeo = new THREE.CylinderGeometry(v.wheel_radius, v.wheel_radius, v.wheel_width, 24);
  for (const [x, isFront] of [[v.cg_to_front, true], [-v.cg_to_rear, false]]) {
    for (const side of [1, -1]) {
      const pivot = new THREE.Group();
      pivot.position.set(x, side * v.track / 2, v.wheel_radius);
      pivot.add(new THREE.Mesh(wheelGeo, wheelMat));   // cylinder axis = y = axle
      car.add(pivot);
      if (isFront) V.pivots.push(pivot);
    }
  }
}

function newTrail(color) {
  const pos = new Float32Array(TRAIL_MAX * 3);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setDrawRange(0, 0);
  const line = layer(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, ...flat })), ORDER.trail);
  scene.add(line);
  return { pos, geo, line, n: 0 };
}

function ensureVehicle(id, color, offset) {
  let V = vehicles.get(id);
  if (V) return V;
  V = { car: new THREE.Group(), body: new THREE.Group(), pivots: [], color: color || 0,
        offset: offset || [0, 0], resets: 0, has: false, s: null };
  V.car.add(V.body);
  V.car.visible = false;           // shown with its first state
  scene.add(V.car);
  // The single vehicle keeps its blue trail; a fleet trail takes the body colour.
  V.trail = newTrail(id === LEGACY_ID && !world ? 0x1f5fbf : PALETTE[V.color % PALETTE.length]);
  buildCar(V);
  vehicles.set(id, V);
  return V;
}

function removeVehicle(id) {
  const V = vehicles.get(id);
  if (!V) return;
  scene.remove(V.car);
  scene.remove(V.trail.line);
  V.trail.geo.dispose();
  vehicles.delete(id);
}

function clearTrail(V) {
  V.trail.n = 0;
  V.trail.geo.setDrawRange(0, 0);
}

function addTrail(V, x, y) {
  const T = V.trail;
  if (T.n > 0) {
    const i = (T.n - 1) * 3;
    if (Math.hypot(T.pos[i] - x, T.pos[i + 1] - y) < 0.2) return;
  }
  if (T.n === TRAIL_MAX) { T.pos.copyWithin(0, 3); T.n--; }
  T.pos.set([x, y, 0.06], T.n * 3);
  T.n++;
  T.geo.setDrawRange(0, T.n);
  T.geo.attributes.position.needsUpdate = true;
  T.geo.computeBoundingSphere();
}

// ---- world (fleet snapshot) ----------------------------------------------
let world = null;           // last world message, null = single-vehicle publisher
let finished = false;
let clock = null;

function applyWorld(w) {
  if (!world || w.session !== world.session) {
    for (const id of [...vehicles.keys()]) removeVehicle(id);
    clock = null;
  }
  world = w;
  const want = new Set();
  for (const v of w.vehicles || []) {
    want.add(v.id);
    const existed = vehicles.has(v.id);
    const V = ensureVehicle(v.id, v.color, v.offset);
    V.offset = v.offset || [0, 0];
    if (existed && V.resets !== v.resets) clearTrail(V);
    V.resets = v.resets;
  }
  for (const id of [...vehicles.keys()]) if (!want.has(id)) removeVehicle(id);
  finished = !!w.finished;
  if (!follow || !vehicles.has(follow)) follow = (w.vehicles && w.vehicles.length) ? w.vehicles[0].id : null;
  if (params.get('follow') && vehicles.has(params.get('follow'))) follow = params.get('follow');
  window.__monitor.world = { session: w.session, mode: w.mode, n_total: w.n_total,
                             n_shown: (w.vehicles || []).length, finished };
  refreshStatus();
}

// ---- camera: chase one vehicle, or an overview of all ----------------------
let chase = true;
let overview = params.get('view') === 'overview';
let follow = null;
const last = new THREE.Vector3();
let haveLast = false;
controls.addEventListener('start', () => { chase = false; });
document.getElementById('follow').addEventListener('click', () => { chase = true; overview = false; });
document.getElementById('overview').addEventListener('click', () => { overview = !overview; chase = true; });
window.addEventListener('keydown', (e) => {
  if (e.key === 'f' || e.key === 'F') { chase = true; overview = false; }
  if (e.key === 'o' || e.key === 'O') { overview = !overview; chase = true; }
  if (e.key === 'n' || e.key === 'N') {           // next vehicle to follow
    const ids = [...vehicles.keys()];
    if (ids.length) follow = ids[(ids.indexOf(follow) + 1) % ids.length];
    chase = true; overview = false;
  }
});

function shownVehicles() { return [...vehicles.values()].filter((V) => V.has); }

function followCamera() {
  window.__monitor.view = overview ? 'overview' : 'chase';
  window.__monitor.follow = follow;
  const shown = shownVehicles();
  if (overview && shown.length) {
    const box = new THREE.Box3();
    for (const V of shown) box.expandByPoint(V.car.position);
    const c = box.getCenter(new THREE.Vector3()), sz = box.getSize(new THREE.Vector3());
    const span = Math.max(sz.x, sz.y * camera.aspect, 30) + 20;
    const h = span / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2)) / Math.max(camera.aspect, 1);
    if (chase) {
      camera.position.set(c.x - 0.35 * h, c.y, h);
      controls.target.set(c.x, c.y, 0);
    }
    ground.position.set(Math.round(c.x / (2 * CHECKER_M)) * 2 * CHECKER_M,
                        Math.round(c.y / (2 * CHECKER_M)) * 2 * CHECKER_M, 0);
    controls.update();
    return;
  }
  const V = vehicles.get(follow) || shown[0];
  if (!V) { controls.update(); return; }
  const p = V.car.position, yaw = V.car.rotation.z;
  if (!haveLast) { last.copy(p); haveLast = true; }
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

function applyVehicleState(id, s) {
  if (typeof s.x !== 'number' || typeof s.y !== 'number' || typeof s.yaw !== 'number') return;
  const V = vehicles.get(id);
  if (!V) return;
  const x = s.x + V.offset[0], y = s.y + V.offset[1];
  V.car.position.set(x, y, 0);
  V.car.rotation.set(0, 0, s.yaw);
  V.body.rotation.set(s.roll || 0, s.pitch || 0, 0, 'ZYX');
  for (const p of V.pivots) p.rotation.z = s.delta || 0;
  addTrail(V, x, y);
  V.s = s;
  if (!V.has) { V.has = true; V.car.visible = true; if (!follow) follow = id; }
}

function updateHud() {
  const V = vehicles.get(follow) || shownVehicles()[0];
  const s = V ? V.s : null;
  let text = '';
  if (s) {
    const v = s.vx;
    const tag = world && vehicles.size > 1 ? `[${follow}] ` : '';
    text = `${tag}t ${num(s.t, 2)} s | v ${num(v, 2)} m/s (${num(typeof v === 'number' ? v * 3.6 : v, 1)} km/h)`
         + ` | steer ${num(typeof s.delta === 'number' ? s.delta * 180 / Math.PI : s.delta, 2)} deg`;
  } else {
    text = 't -- s | v -- m/s | steer -- deg';
  }
  if (world) {
    const c = clock || {};
    text += `\nsim ${num(c.t_sim, 1)} s | wall ${num(c.t_wall, 1)} s | x${num(c.rate, 1)}`
          + ` | ${(world.vehicles || []).length} / ${world.n_total} shown | ${world.mode}`;
  }
  if (osmError) text += `\nOSM: ${osmError}`;
  hud.textContent = text;
  const tr = {};
  for (const [id, W] of vehicles) tr[id] = W.trail.n;
  window.__monitor.trails = tr;
  window.__monitor.ids = [...vehicles.keys()];
  window.__monitor.vehicles = shownVehicles().length;
  window.__monitor.rate = clock && typeof clock.rate === 'number' ? clock.rate : null;
}

let osmError = null;
let sceneVersion = -1;

async function loadScene(version) {
  if (version === sceneVersion) return;
  try {
    const r = await fetch('./scene.json', { cache: 'no-store' });
    const sc = await r.json();
    sceneVersion = sc.version;
    geometry = { ...DEFAULT_VEHICLE, ...(sc.vehicle || {}) };
    for (const V of vehicles.values()) buildCar(V);
    buildRoads(sc.roads || []);
    osmError = sc.osm_error;
    window.__monitor.sceneVersion = sc.version;
    window.__monitor.roads = (sc.roads || []).length;
  } catch (e) { /* retried on the next status */ }
}

let connected = false, live = false;

function refreshStatus() {
  window.__monitor.connected = connected;
  window.__monitor.live = live;
  const worldOpen = !!world && !finished;
  let cls = 'down', text = 'reconnecting', show = true;
  if (finished) {
    cls = 'done'; show = false;
    text = connected ? 'simulation finished' : 'simulation finished (monitor closed)';
  } else if (connected && live) {
    cls = 'live'; text = 'live'; show = false;
  } else if (connected && worldOpen) {
    cls = ''; text = 'world ready, waiting for first state'; show = false;
  } else if (connected) {
    cls = ''; text = 'connected, no simulator';
  }
  statusEl.className = cls;
  statusEl.textContent = text;
  wait.classList.toggle('hidden', !show);
  waitMsg.textContent = connected ? 'waiting for simulator' : 'monitor server unreachable, retrying';
}

function setStatus(c, l) { connected = c; live = l; refreshStatus(); }

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
    } else if (m.type === 'world') {
      applyWorld(m.world);
    } else if (m.type === 'state') {
      if (world && finished) {        // a single-vehicle run after a finished world
        for (const id of [...vehicles.keys()]) removeVehicle(id);
        world = null; finished = false; clock = null;
        window.__monitor.world = null;
      }
      if (!live) setStatus(true, true);
      ensureVehicle(LEGACY_ID, 0, [0, 0]);
      applyVehicleState(LEGACY_ID, m.state);
      window.__monitor.states++;
    } else if (m.type === 'states') {
      if (!live) setStatus(true, true);
      if (m.clock) clock = m.clock;
      for (const s of m.states || []) applyVehicleState(s.id, s);
      window.__monitor.states++;
    }
  };
  ws.onclose = () => { ws = null; setStatus(false, false); setTimeout(connect, RECONNECT_MS); };
  ws.onerror = () => { try { ws.close(); } catch (e) { /* closing */ } };
}

setInterval(() => {
  if (ws && ws.readyState === WebSocket.OPEN && performance.now() - lastMsg > SILENT_MS) ws.close();
}, 1000);

setStatus(false, false);
connect();

renderer.setAnimationLoop(() => {
  followCamera();
  updateHud();
  renderer.render(scene, camera);
});
