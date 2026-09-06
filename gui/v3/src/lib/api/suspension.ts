import { fetchJson } from './client'

export type Vec3 = [number, number, number]

export interface KinSchematic {
  links: [Vec3, Vec3][]
  points?: Record<string, Vec3>
}

export interface KcPlot {
  title: string
  xlabel?: string
  ylabel?: string
  series: { x: number[]; y: number[]; label?: string; color?: string }[]
}

export interface SuspensionKin {
  ok?: boolean
  name: string
  part_id: string
  type: string
  doc: Record<string, unknown>
  geometry: Record<string, Vec3>
  schematic: KinSchematic
  path: string
  readonly: boolean
  plots?: KcPlot[]
}

export interface KinPreview {
  ok?: boolean
  type: string
  geometry: Record<string, Vec3>
  schematic: KinSchematic
  plots: KcPlot[]
}

export async function fetchSuspensionKin(partId: string): Promise<SuspensionKin> {
  const qs = new URLSearchParams({ part_id: partId })
  return fetchJson<SuspensionKin>(`/api/suspension/kin?${qs}`)
}

export async function previewSuspensionKin(
  doc: Record<string, unknown>,
): Promise<KinPreview> {
  return fetchJson<KinPreview>('/api/suspension/kin/preview', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ doc }),
  })
}

export async function saveSuspensionKin(body: {
  doc: Record<string, unknown>
  stem: string
  label: string
  base_part_id?: string
}): Promise<{ ok?: boolean; part_id: string; label: string }> {
  return fetchJson('/api/suspension/kin/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}
