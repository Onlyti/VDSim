// Top-view mini-map trail (2D canvas), extracted from app.js (B2b).
// Fully self-contained: owns its canvas + trail buffer; only dependency is $.

import { $ } from './util.js';

const mm = $('minimap');
const mmCtx = mm.getContext('2d');
const mmTrail = [];
const MM_MAX = 400;

export function resetMinimap() { mmTrail.length = 0; }

export function drawMinimap(x, y) {
  if (x == null) return;
  mmTrail.push([x, y]);
  if (mmTrail.length > MM_MAX) mmTrail.shift();
  const dpr = devicePixelRatio || 1;
  const W = mm.clientWidth, H = mm.clientHeight;
  mm.width = W * dpr; mm.height = H * dpr;
  mmCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  mmCtx.clearRect(0, 0, W, H);
  mmCtx.fillStyle = '#f4f7fa'; mmCtx.fillRect(0, 0, W, H);
  let xmin = x, xmax = x, ymin = y, ymax = y;
  for (const [px, py] of mmTrail) {
    if (px < xmin) xmin = px; if (px > xmax) xmax = px;
    if (py < ymin) ymin = py; if (py > ymax) ymax = py;
  }
  const pad = Math.max(20, Math.max(xmax - xmin, ymax - ymin) * 0.15 + 5);
  xmin -= pad; xmax += pad; ymin -= pad; ymax += pad;
  if (xmax - xmin < 10) { xmin -= 5; xmax += 5; }
  if (ymax - ymin < 10) { ymin -= 5; ymax += 5; }
  const X = v => 8 + (v - xmin) / (xmax - xmin) * (W - 16);
  const Y = v => H - 8 - (v - ymin) / (ymax - ymin) * (H - 16);
  mmCtx.strokeStyle = '#d2dae3'; mmCtx.strokeRect(0.5, 0.5, W - 1, H - 1);
  if (mmTrail.length > 1) {
    mmCtx.strokeStyle = '#ffb020'; mmCtx.lineWidth = 1.2; mmCtx.beginPath();
    mmTrail.forEach(([px, py], i) => { const cx = X(px), cy = Y(py); i ? mmCtx.lineTo(cx, cy) : mmCtx.moveTo(cx, cy); });
    mmCtx.stroke();
  }
  mmCtx.fillStyle = '#01A0E9';
  mmCtx.beginPath(); mmCtx.arc(X(x), Y(y), 4, 0, Math.PI * 2); mmCtx.fill();
}
