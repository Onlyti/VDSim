import type { SuspensionSchematic } from '../api/catalog'

function yzPoints(sch: SuspensionSchematic): [number, number][] {
  const pts: [number, number][] = []
  for (const [a, b] of sch.links) {
    pts.push([a[1], a[2]], [b[1], b[2]])
  }
  for (const p of Object.values(sch.points ?? {})) {
    pts.push([p[1], p[2]])
  }
  return pts
}

export function linkageSvg(
  sch: SuspensionSchematic,
  opts?: { label?: string; stroke?: string },
): string {
  const pts = yzPoints(sch)
  if (!pts.length) return ''
  let ymin = pts[0][1]
  let ymax = pts[0][1]
  let zmin = pts[0][0]
  let zmax = pts[0][0]
  for (const [y, z] of pts) {
    if (y < ymin) ymin = y
    if (y > ymax) ymax = y
    if (z < zmin) zmin = z
    if (z > zmax) zmax = z
  }
  const pad = 0.08
  ymin -= pad
  ymax += pad
  zmin -= pad
  zmax += pad
  const spanY = Math.max(ymax - ymin, 0.2)
  const spanZ = Math.max(zmax - zmin, 0.2)
  const W = 200
  const H = 140
  const margin = 14
  const drawW = W - margin * 2
  const drawH = H - margin * 2
  const scale = Math.min(drawW / spanY, drawH / spanZ)
  const cx = 0.5 * (ymin + ymax)
  const cz = 0.5 * (zmin + zmax)
  const visY = drawW / scale
  const visZ = drawH / scale
  const y0 = cx - visY * 0.5
  const z0 = cz - visZ * 0.5

  const X = (y: number) => margin + (y - y0) * scale
  const Y = (z: number) => H - margin - (z - z0) * scale

  const stroke = opts?.stroke ?? '#2f6d8f'
  let svg = `<svg viewBox="0 0 ${W} ${H}" width="100%" height="100%" xmlns="http://www.w3.org/2000/svg">`
  svg += `<line x1="${margin}" y1="${Y(0).toFixed(1)}" x2="${W - margin}" y2="${Y(0).toFixed(1)}" stroke="#d8dee8" stroke-width="1"/>`
  for (const [a, b] of sch.links) {
    svg += `<line x1="${X(a[1]).toFixed(1)}" y1="${Y(a[2]).toFixed(1)}" x2="${X(b[1]).toFixed(1)}" y2="${Y(b[2]).toFixed(1)}" stroke="${stroke}" stroke-width="2" stroke-linecap="round"/>`
  }
  for (const [lab, p] of Object.entries(sch.points ?? {})) {
    svg += `<circle cx="${X(p[1]).toFixed(1)}" cy="${Y(p[2]).toFixed(1)}" r="3" fill="${stroke}"/>`
    svg += `<text x="${X(p[1]).toFixed(1)}" y="${(Y(p[2]) - 5).toFixed(1)}" font-size="8" fill="#6b7280" text-anchor="middle">${lab}</text>`
  }
  if (opts?.label) {
    svg += `<text x="${margin}" y="11" font-size="9" fill="#6b7280">${opts.label}</text>`
  }
  svg += '</svg>'
  return svg
}
