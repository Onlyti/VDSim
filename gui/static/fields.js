// Field-row / select-row / plot builders extracted from app.js (B2b #2).
// Pure: every function takes its DOM target + data as args and uses only
// document/canvas APIs (no shared scene/module state, no util helpers).
function arbRow(f, lvl) {
  const applies = f.levels.includes(lvl);
  const axle = f.name.endsWith('front') ? 'front' : 'rear', trackName = 'track_' + axle;
  const row = document.createElement('div'); row.className = 'fld' + (applies ? '' : ' na');
  const lab = document.createElement('label'); lab.textContent = f.label;
  if (!applies) { const b = document.createElement('span'); b.className = 'badge'; b.textContent = f.levels.join('/'); lab.appendChild(b); }
  row.appendChild(lab);
  const wrap = document.createElement('span'); wrap.className = 'arr';
  const roll = document.createElement('input'); roll.type = 'number'; roll.step = 'any';
  roll.value = f.value; roll.dataset.name = f.name; roll.disabled = !applies;
  const wh = document.createElement('input'); wh.type = 'number'; wh.step = 'any'; wh.disabled = !applies;
  const unit = t => { const s = document.createElement('span'); s.style.cssText = 'font:10px Tahoma;color:var(--dim)'; s.textContent = t; return s; };
  const tw = () => { const t = document.querySelector(`[data-name="${trackName}"]`); return t ? +t.value : 1.55; };
  const toWh = () => { const w = tw(); wh.value = w > 1e-6 ? (2 * (+roll.value) / (w * w)).toFixed(1) : 0; };
  const toRoll = () => { const w = tw(); roll.value = ((+wh.value) * w * w / 2).toFixed(1); };
  toWh(); roll.oninput = toWh; wh.oninput = toRoll;
  wrap.appendChild(roll); wrap.appendChild(unit('N·m/rad')); wrap.appendChild(wh); wrap.appendChild(unit('N/m'));
  row.appendChild(wrap); return row;
}
export function fieldRow(f, lvl) {
  if (f.name === 'arb_stiffness_front' || f.name === 'arb_stiffness_rear') return arbRow(f, lvl);
  const applies = f.levels.includes(lvl);
  const row = document.createElement('div'); row.className = 'fld' + (applies ? '' : ' na');
  const lab = document.createElement('label'); lab.textContent = f.label;
  if (!applies) { const b = document.createElement('span'); b.className = 'badge'; b.textContent = f.levels.join('/'); lab.appendChild(b); }
  row.appendChild(lab);
  if (f.kind === 'arr') {
    const wrap = document.createElement('span'); wrap.className = 'arr';
    const corners = f.value.length === 4 ? ['FL', 'FR', 'RL', 'RR'] : null;
    f.value.forEach((v, i) => {
      const cell = document.createElement('span');
      cell.className = 'arr-cell';
      if (corners) {
        const tag = document.createElement('span');
        tag.className = 'arr-corner';
        tag.textContent = corners[i];
        cell.appendChild(tag);
      }
      const inp = document.createElement('input'); inp.type = 'number'; inp.step = 'any';
      inp.value = v; inp.dataset.name = f.name; inp.dataset.idx = i; inp.disabled = !applies;
      cell.appendChild(inp);
      wrap.appendChild(cell);
    });
    row.appendChild(wrap);
  } else if (f.kind === 'bool') {
    const inp = document.createElement('input'); inp.type = 'checkbox';
    inp.checked = !!f.value; inp.dataset.name = f.name; inp.disabled = !applies; row.appendChild(inp);
  } else if (f.kind === 'enum') {
    const sel = document.createElement('select');
    sel.innerHTML = (f.choices || []).map(c => `<option ${c === f.value ? 'selected' : ''}>${c}</option>`).join('');
    sel.dataset.name = f.name; sel.disabled = !applies; row.appendChild(sel);
  } else {
    const inp = document.createElement('input'); inp.type = 'number'; inp.step = 'any';
    inp.value = f.value; inp.dataset.name = f.name; inp.disabled = !applies; row.appendChild(inp);
  }
  return row;
}
export function collectFields(el, names) {
  const out = {}, arrs = {};
  el.querySelectorAll('[data-name]').forEach(inp => {
    const nm = inp.dataset.name; if (names && !names.has(nm)) return;
    if (inp.dataset.idx !== undefined) (arrs[nm] = arrs[nm] || [])[+inp.dataset.idx] = +inp.value;
    else if (inp.type === 'checkbox') out[nm] = inp.checked;
    else if (inp.tagName === 'SELECT') out[nm] = inp.value;
    else out[nm] = +inp.value;
  });
  Object.assign(out, arrs); return out;
}
export function plantEnhance(el) {
  const q = n => el.querySelector(`[data-name="${n}"]`);
  const rowOf = n => n && n.closest('.fld');
  const dim = (n, off) => { const r = rowOf(n); if (r) r.classList.toggle('na', off); if (n) n.disabled = off; };
  const diff = q('differential'), lp = q('lsd_preload'), lr = q('lsd_ramp');
  if (diff) { const upd = () => { const off = diff.value !== 'LSD'; dim(lp, off); dim(lr, off); }; diff.addEventListener('change', upd); upd(); }
  const lug = q('steer.friction.enabled');
  const stau = q('steer.ch.tau_s'), skp = q('steer.servo_kp'), skd = q('steer.servo_kd');
  if (lug) {
    const r = rowOf(lug); let hint = null;
    if (r) { hint = document.createElement('div'); hint.className = 'sub'; hint.style.margin = '-2px 0 4px'; r.after(hint); }
    const upd = () => {
      const on = lug.checked;
      dim(stau, on); dim(skp, !on); dim(skd, !on);
      if (hint) {
        hint.textContent = on ? '▸ mode: torque servo (kp/kd + LuGre friction)' : '▸ mode: first-order lag (τ)';
        hint.style.color = on ? 'var(--acc)' : 'var(--dim)';
      }
    };
    lug.addEventListener('change', upd); upd();
  }
  const ebd = q('brake_ebd_enabled'), bias = q('brake_bias_front');
  if (ebd && bias) {
    const upd = () => {
      const r = rowOf(bias);
      if (r) {
        r.style.opacity = ebd.checked ? 0.6 : 1;
        r.title = ebd.checked ? 'EBD on: dynamic Fz-based bias overrides this (static fallback)' : '';
      }
    };
    ebd.addEventListener('change', upd); upd();
  }
  if (bias) {
    bias.min = 0; bias.max = 1;
    bias.addEventListener('input', () => { const v = +bias.value; if (v < 0) bias.value = 0; else if (v > 1) bias.value = 1; });
  }
  const fzn = q('Fz_nominal');
  if (fzn) {
    const ab = document.createElement('button'); ab.type = 'button'; ab.textContent = 'auto';
    ab.style.cssText = 'width:auto;padding:2px 8px;margin-left:4px';
    ab.title = 'set to mean per-corner static load  m·g/4';
    ab.onclick = () => { const m = +((q('mass') || {}).value || 1500); fzn.value = (m * 9.81 / 4).toFixed(0); };
    rowOf(fzn).appendChild(ab);
  }
  const wi = q('wheel_inertia');
  if (wi) {
    const r = rowOf(wi); const hint = document.createElement('div'); hint.className = 'sub'; hint.style.margin = '-2px 0 4px';
    const upd = () => {
      const R = +((q('wheel_radius_nominal') || {}).value || 0.32);
      const u = [...el.querySelectorAll('[data-name="unsprung_mass"]')].map(i => +i.value || 0);
      hint.textContent = '0 = auto ≈ ' + u.map(m => (0.5 * m * R * R).toFixed(2)).join(' / ') + ' kg·m²';
    };
    upd(); r.after(hint);
    [...el.querySelectorAll('[data-name="unsprung_mass"]'), q('wheel_radius_nominal')]
      .forEach(i => i && i.addEventListener('input', upd));
  }
}
export function linkPlantFields(el) {
  const q = n => el.querySelector(`input[data-name="${n}"]`);
  const setIf = (inp, v) => { if (inp && document.activeElement !== inp) inp.value = (+v).toFixed(3); };
  const mass = q('mass'), sprung = q('mass_sprung');
  const uns = [...el.querySelectorAll('input[data-name="unsprung_mass"]')];
  const sumU = () => uns.reduce((s, i) => s + (+i.value || 0), 0);
  if (mass && sprung) {
    const fromParts = () => setIf(mass, (+sprung.value || 0) + sumU());
    sprung.addEventListener('input', fromParts);
    uns.forEach(i => i.addEventListener('input', fromParts));
    mass.addEventListener('input', () => setIf(sprung, Math.max(0, (+mass.value || 0) - sumU())));
  }
  const wb = q('wheelbase'), a = q('cg_to_front'), b = q('cg_to_rear');
  if (wb && a && b) {
    const fromAB = () => setIf(wb, (+a.value || 0) + (+b.value || 0));
    a.addEventListener('input', fromAB); b.addEventListener('input', fromAB);
    wb.addEventListener('input', () => {
      const L0 = (+a.value || 0) + (+b.value || 0), W = +wb.value || 0;
      if (L0 > 1e-6) { const af = (+a.value) / L0; setIf(a, W * af); setIf(b, W * (1 - af)); }
    });
  }
}
export function selectRow(label, opts, cur, onChange, labelMap) {
  const row = document.createElement('div'); row.className = 'fld';
  const lab = document.createElement('label'); lab.textContent = label; row.appendChild(lab);
  const sel = document.createElement('select');
  sel.innerHTML = opts.map(v => `<option value="${v}" ${v === cur ? 'selected' : ''}>${labelMap ? (labelMap[v] || v) : v}</option>`).join('');
  sel.onchange = e => onChange(e.target.value); row.appendChild(sel); return row;
}
function formatPlotTick(v) {
  const a = Math.abs(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  if (a >= 1) return v.toFixed(2);
  if (a >= 0.01) return v.toFixed(3);
  return v.toExponential(1);
}
function nicePlotTicks(lo, hi, target = 5) {
  if (!isFinite(lo) || !isFinite(hi)) return [0, 1];
  if (lo > hi) { const t = lo; lo = hi; hi = t; }
  const span = hi - lo;
  if (span < 1e-12) return [lo, lo + 1];
  const raw = span / Math.max(2, target - 1);
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const norm = raw / mag;
  let step = mag;
  if (norm > 5) step = 10 * mag;
  else if (norm > 2) step = 5 * mag;
  else if (norm > 1) step = 2 * mag;
  const start = Math.floor(lo / step) * step;
  const ticks = [];
  for (let v = start; v <= hi + step * 1e-6; v += step) ticks.push(+v.toFixed(12));
  if (ticks.length < 2) return [lo, hi];
  return ticks;
}
export function plotChart(cv, p) {
  const dpr = devicePixelRatio || 1, W = cv.clientWidth || 300, H = cv.clientHeight || 148;
  cv.width = W * dpr; cv.height = H * dpr;
  const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const ml = p.ylabel ? 54 : 44, mr = 10, mt = 20, mb = p.xlabel ? 34 : 24;
  const pw = W - ml - mr, ph = H - mt - mb;
  let xmin = 1 / 0, xmax = -1 / 0, ymin = 1 / 0, ymax = -1 / 0;
  p.series.forEach(s => s.x.forEach((x, i) => {
    const y = s.y[i]; if (x < xmin) xmin = x; if (x > xmax) xmax = x; if (y < ymin) ymin = y; if (y > ymax) ymax = y;
  }));
  if (!isFinite(xmin)) return;
  if (ymin === ymax) { ymin -= 1; ymax += 1; }
  if (xmin === xmax) { xmax = xmin + 1; }
  const padY = (ymax - ymin) * 0.06; ymin -= padY; ymax += padY;
  const xTicks = nicePlotTicks(xmin, xmax);
  const yTicks = nicePlotTicks(ymin, ymax);
  xmin = xTicks[0]; xmax = xTicks[xTicks.length - 1];
  ymin = yTicks[0]; ymax = yTicks[yTicks.length - 1];
  const X = x => ml + (x - xmin) / (xmax - xmin) * pw;
  const Y = y => mt + (1 - (y - ymin) / (ymax - ymin)) * ph;
  g.strokeStyle = '#e9edf2'; g.lineWidth = 1;
  xTicks.forEach(v => {
    const px = X(v); g.beginPath(); g.moveTo(px, mt); g.lineTo(px, mt + ph); g.stroke();
  });
  yTicks.forEach(v => {
    const py = Y(v); g.beginPath(); g.moveTo(ml, py); g.lineTo(ml + pw, py); g.stroke();
  });
  g.strokeStyle = '#c5cdd6'; g.lineWidth = 1; g.strokeRect(ml, mt, pw, ph);
  g.fillStyle = '#6b7480'; g.font = '9px Tahoma';
  g.textAlign = 'center';
  xTicks.forEach(v => g.fillText(formatPlotTick(v), X(v), mt + ph + 12));
  g.textAlign = 'right';
  yTicks.forEach(v => g.fillText(formatPlotTick(v), ml - 5, Y(v) + 3));
  if (p.xlabel) {
    g.fillStyle = '#4a5560'; g.textAlign = 'center';
    g.fillText(p.xlabel, ml + pw / 2, H - 6);
  }
  if (p.ylabel) {
    g.save(); g.translate(14, mt + ph / 2); g.rotate(-Math.PI / 2);
    g.fillStyle = '#4a5560'; g.textAlign = 'center'; g.font = '9px Tahoma';
    g.fillText(p.ylabel, 0, 0); g.restore();
  }
  g.fillStyle = '#2a2f37'; g.font = '600 10px Tahoma'; g.textAlign = 'left';
  g.fillText(p.title, ml, 13);
  if (p.series.length > 1 && p.series.some(s => s.label)) {
    let ly = mt + 6;
    g.textAlign = 'right';
    p.series.forEach(s => {
      if (!s.label) return;
      const lx = ml + pw - 4;
      g.strokeStyle = s.color; g.lineWidth = 2; g.setLineDash(s.dash ? [4, 3] : []);
      g.beginPath(); g.moveTo(lx - 48, ly + 4); g.lineTo(lx - 34, ly + 4); g.stroke();
      g.setLineDash([]);
      g.fillStyle = '#5a6270'; g.font = '8px Tahoma';
      g.fillText(s.label, lx, ly + 7);
      ly += 11;
    });
  }
  p.series.forEach(s => {
    g.strokeStyle = s.color; g.lineWidth = 1.6; g.setLineDash(s.dash ? [4, 3] : []);
    g.beginPath();
    s.x.forEach((x, i) => { const px = X(x), py = Y(s.y[i]); i ? g.lineTo(px, py) : g.moveTo(px, py); });
    g.stroke();
  });
  g.setLineDash([]);
}
export function renderPlots(box, plots) {
  box.innerHTML = '';
  plots.forEach(p => {
    const cv = document.createElement('canvas');
    box.appendChild(cv);
    requestAnimationFrame(() => plotChart(cv, p));
  });
}
