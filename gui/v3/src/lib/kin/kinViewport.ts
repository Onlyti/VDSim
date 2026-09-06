export interface YzViewport {
  W: number
  H: number
  margin: number
  scale: number
  y0: number
  z0: number
}

export function collectYzPts(
  links: [number, number, number][][],
  points: Record<string, [number, number, number]> | undefined,
  geometry: Record<string, [number, number, number]> | undefined,
): [number, number][] {
  const pts: [number, number][] = []
  for (const [a, b] of links) {
    pts.push([a[1], a[2]], [b[1], b[2]])
  }
  for (const p of Object.values(points ?? {})) {
    pts.push([p[1], p[2]])
  }
  for (const p of Object.values(geometry ?? {})) {
    pts.push([p[1], p[2]])
  }
  return pts
}

export function fitYzViewport(pts: [number, number][], W: number, H: number): YzViewport | null {
  if (!pts.length || W < 20 || H < 20) return null
  let ymin = pts[0][0]
  let ymax = pts[0][0]
  let zmin = pts[0][1]
  let zmax = pts[0][1]
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
  const margin = 14
  const drawW = W - margin * 2
  const drawH = H - margin * 2
  const scale = Math.min(drawW / spanY, drawH / spanZ)
  const cx = 0.5 * (ymin + ymax)
  const cz = 0.5 * (zmin + zmax)
  const visY = drawW / scale
  const visZ = drawH / scale
  return {
    W,
    H,
    margin,
    scale,
    y0: cx - visY * 0.5,
    z0: cz - visZ * 0.5,
  }
}

export function vpX(vp: YzViewport, y: number): number {
  return vp.margin + (y - vp.y0) * vp.scale
}

export function vpY(vp: YzViewport, z: number): number {
  return vp.H - vp.margin - (z - vp.z0) * vp.scale
}

export function vpScreenToYz(vp: YzViewport, sx: number, sy: number): [number, number] {
  const y = vp.y0 + (sx - vp.margin) / vp.scale
  const z = vp.z0 + (vp.H - vp.margin - sy) / vp.scale
  return [y, z]
}
