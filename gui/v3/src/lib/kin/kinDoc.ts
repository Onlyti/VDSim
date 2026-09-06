import type { Vec3 } from '../api/suspension'

export function isDraggablePath(path: string): boolean {
  if (path.includes('spin_axis') || path.endsWith('.static_radius')) return false
  return /^(lca|uca|strut|tie_rod|spring_damper|arm_pivot|links|wheel\.center)/.test(path)
}

export function setKinPath(
  doc: Record<string, unknown>,
  path: string,
  value: Vec3,
): Record<string, unknown> {
  const parts = path.split('.')
  let node = doc
  for (let i = 0; i < parts.length - 1; i++) {
    node = node[parts[i]] as Record<string, unknown>
  }
  node[parts[parts.length - 1]] = [value[0], value[1], value[2]]
  return doc
}

export function shortPath(path: string): string {
  const p = path.split('.')
  return p.length > 2 ? p.slice(-2).join('.') : path
}

export function draggablePaths(geometry: Record<string, Vec3>): string[] {
  return Object.keys(geometry).filter(isDraggablePath).sort()
}
