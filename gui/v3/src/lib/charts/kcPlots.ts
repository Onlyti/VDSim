import type { KcPlot } from '../api/suspension'

export function kcPlotSvg(p: KcPlot, opts?: { width?: number; height?: number }): string {
  const W = opts?.width ?? 280
  const H = opts?.height ?? 120
  const pad = { l: 42, r: 8, t: 18, b: 22 }
  let xmin = Infinity
  let xmax = -Infinity
  let ymin = Infinity
  let ymax = -Infinity
  for (const s of p.series) {
    for (let i = 0; i < s.x.length; i++) {
      xmin = Math.min(xmin, s.x[i])
      xmax = Math.max(xmax, s.x[i])
      ymin = Math.min(ymin, s.y[i])
      ymax = Math.max(ymax, s.y[i])
    }
  }
  if (!Number.isFinite(xmin)) return ''
  if (ymin === ymax) {
    ymin -= 1
    ymax += 1
  }
  if (xmin === xmax) xmax = xmin + 1
  const py = (ymax - ymin) * 0.06
  ymin -= py
  ymax += py
  const X = (x: number) => pad.l + ((x - xmin) / (xmax - xmin)) * (W - pad.l - pad.r)
  const Y = (y: number) => H - pad.b - ((y - ymin) / (ymax - ymin)) * (H - pad.t - pad.b)

  let svg = `<svg viewBox="0 0 ${W} ${H}" width="100%" height="${H}" xmlns="http://www.w3.org/2000/svg">`
  svg += `<rect x="${pad.l}" y="${pad.t}" width="${W - pad.l - pad.r}" height="${H - pad.t - pad.b}" fill="none" stroke="#dde2e8"/>`
  svg += `<text x="${pad.l}" y="12" font-size="9" fill="#6b7480">${p.title}</text>`
  if (p.ylabel) {
    svg += `<text x="4" y="${pad.t + 8}" font-size="8" fill="#9aa3ad" transform="rotate(-90 8 ${pad.t + 8})">${p.ylabel}</text>`
  }
  if (p.xlabel) {
    svg += `<text x="${W - pad.r}" y="${H - 4}" font-size="8" fill="#9aa3ad" text-anchor="end">${p.xlabel}</text>`
  }
  for (const s of p.series) {
    const c = s.color ?? '#01A0E9'
    const pts = s.x.map((x, i) => `${X(x).toFixed(1)},${Y(s.y[i]).toFixed(1)}`).join(' ')
    svg += `<polyline points="${pts}" fill="none" stroke="${c}" stroke-width="1.4"/>`
  }
  svg += '</svg>'
  return svg
}
