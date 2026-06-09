// Leaf UI / HTTP / doc helpers extracted from app.js (B2a).
// Pure: no dependency on the shared three.js scene / module-scope state, so they
// live in their own module. Behaviour is identical to the former inline defs.

export const $ = id => document.getElementById(id);

export function paintConnState(s) {
  const conn = $('conn');
  if (!conn || !s) return;
  if (s.kinematics_warnings?.length) {
    conn.textContent = '● kinematics warning';
    conn.style.color = '#ff9500';
    return;
  }
  if (s.plant_error) {
    conn.textContent = '● ' + s.plant_error;
    conn.style.color = '#ff9500';
  } else if (s.setup_mode || (!s.running && !s.plant_error)) {
    conn.textContent = '● setup — press ▶ Play';
    conn.style.color = 'var(--dim)';
  } else if (s.running && s.source === 'waiting') {
    conn.textContent = '● waiting for plant… (retry ▶ Play)';
    conn.style.color = '#ff9500';
  } else if (s.running) {
    conn.textContent = '● running';
    conn.style.color = '#34c759';
  }
}

export const DOCS = 'https://onlyti.github.io/VDSim/main/theory/';
export const DOC_MAP = {
  'Preset': '', 'Mass & inertia': '02_rigid_body_dynamics', 'Geometry': '04_ld1_bicycle',
  'Suspension': '05_ld2_seven_dof', 'Drivetrain': '05_ld2_seven_dof', 'Steering': '05_ld2_seven_dof',
  'Aero': '04_ld1_bicycle', 'Tire': '03_tire_pacejka_mf96', 'LuGre': '19_lugre_dynamic_tire',
  'Actuator & feedback': '17_actuator_and_sensing',
  'model': '', 'numerics': '11_numerical_integration', 'run': '09_pure_pursuit_path', 'cosim': '18_runtime_and_cosim',
  'frames': '01_frames_and_conventions', 'road': '03_tire_pacejka_mf96',
};
export function docHref(key) { const f = DOC_MAP[key]; return f === undefined ? null : DOCS + (f ? f + '/' : ''); }
export function helpIcon(href) {
  if (!href) return null;
  const a = document.createElement('a'); a.href = href; a.target = '_blank'; a.rel = 'noopener';
  a.textContent = '?'; a.title = '관련 문서 열기';
  a.style.cssText = 'display:inline-flex;align-items:center;justify-content:center;width:15px;height:15px;margin-left:6px;' +
    'border:1px solid var(--dim);border-radius:50%;font:bold 10px Tahoma;color:var(--dim);text-decoration:none;cursor:help;vertical-align:middle';
  a.onclick = e => e.stopPropagation();
  return a;
}
export function addHelp(node, key) { const a = helpIcon(docHref(key)); if (a && node) node.appendChild(a); }
export function initStaticHelp() {
  document.querySelectorAll('[data-help]').forEach(el => addHelp(el, el.dataset.help));
}
export const f = (v, p = 2) => (v == null ? '—' : (+v).toFixed(p));
export const fmtArr = (a, p = 2) => (!a || !a.length ? '—' : a.map(v => (+v).toFixed(p)).join('  '));
export async function post(url, obj) {
  await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(obj) });
}
export async function postJson(url, obj) {
  const r = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(obj ?? {}),
  });
  const ct = r.headers.get('Content-Type') || '';
  let j = {};
  if (ct.includes('json')) {
    j = await r.json();
  } else {
    const text = await r.text();
    throw new Error(r.ok ? text.slice(0, 120) : (`HTTP ${r.status}: ${text.slice(0, 120)}`));
  }
  if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
  return j;
}
export async function getJson(url) {
  const r = await fetch(url);
  const ct = r.headers.get('Content-Type') || '';
  let j = {};
  if (ct.includes('json')) {
    j = await r.json();
  } else {
    const text = await r.text();
    const hint = r.status === 404 ? ' — restart gui/server.py and hard-refresh' : '';
    throw new Error(`HTTP ${r.status}${hint}: ${text.replace(/<[^>]+>/g, '').slice(0, 100)}`);
  }
  if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
  if (j.ok === false) throw new Error(j.error || 'request failed');
  return j;
}
