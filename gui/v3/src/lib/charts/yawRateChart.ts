import type { CompareTrace } from '../api/compare'

const PALETTE = ['#01A0E9', '#DC291E', '#2a8a4a', '#9b59b6', '#e6a817', '#333333']

export function yawRateOverlaySvg(
  traces: Record<string, CompareTrace>,
  opts?: { width?: number; height?: number },
): string {
  const names = Object.keys(traces)
  if (!names.length) return ''

  const W = opts?.width ?? 720
  const H = opts?.height ?? 220
  const pad = { l: 50, r: 12, t: 22, b: 26 }

  let tmin = Infinity
  let tmax = -Infinity
  let rmin = Infinity
  let rmax = -Infinity
  for (const n of names) {
    const tr = traces[n]
    for (const x of tr.t) {
      tmin = Math.min(tmin, x)
      tmax = Math.max(tmax, x)
    }
    for (const y of tr.r) {
      rmin = Math.min(rmin, y)
      rmax = Math.max(rmax, y)
    }
  }
  rmin = Math.min(rmin, 0)
  rmax = Math.max(rmax, 0)

  const X = (t: number) => pad.l + ((t - tmin) / (tmax - tmin || 1)) * (W - pad.l - pad.r)
  const Y = (r: number) => H - pad.b - ((r - rmin) / (rmax - rmin || 1)) * (H - pad.t - pad.b)

  let svg = `<svg viewBox="0 0 ${W} ${H}" width="100%" xmlns="http://www.w3.org/2000/svg">`
  svg += `<line x1="${pad.l}" y1="${Y(0).toFixed(1)}" x2="${W - pad.r}" y2="${Y(0).toFixed(1)}" stroke="#dde2e8"/>`
  svg += `<line x1="${pad.l}" y1="${pad.t}" x2="${pad.l}" y2="${H - pad.b}" stroke="#dde2e8"/>`
  svg += `<text x="${pad.l}" y="13" font-size="10" fill="#6b7480">yaw rate r [rad/s]</text>`
  svg += `<text x="${W - pad.r}" y="${H - 8}" font-size="9" fill="#6b7480" text-anchor="end">t [s]</text>`

  names.forEach((n, i) => {
    const tr = traces[n]
    const c = PALETTE[i % PALETTE.length]
    const pts = tr.t.map((x, j) => `${X(x).toFixed(1)},${Y(tr.r[j]).toFixed(1)}`).join(' ')
    svg += `<polyline points="${pts}" fill="none" stroke="${c}" stroke-width="1.6"/>`
    const lab = n.replace(/^vehicle\./, '')
    svg += `<text x="${pad.l + 8}" y="${pad.t + 11 + i * 13}" font-size="10" fill="${c}">${lab}</text>`
  })
  svg += '</svg>'
  return svg
}
