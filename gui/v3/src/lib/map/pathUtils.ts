export function downsamplePathPts(
  pts: [number, number][],
  maxN = 16,
): [number, number][] {
  if (pts.length <= maxN) return pts.map((p) => [p[0], p[1]])
  const out: [number, number][] = []
  const step = (pts.length - 1) / (maxN - 1)
  for (let i = 0; i < maxN; i++) {
    const p = pts[Math.round(i * step)]
    out.push([p[0], p[1]])
  }
  return out
}
