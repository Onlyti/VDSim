import { fetchJson } from './client'

export interface PartCard {
  tier?: string
  lines?: string[]
  blurb?: string
}

export interface PartCompat {
  level: string
  msg?: string
}

export interface AssemblyCandidate {
  id: string
  label: string
  tags?: string[]
  card?: PartCard
  compat?: PartCompat[]
  ok?: boolean
}

export interface AssemblySlot {
  slot: string
  label: string
  type: string
  part_id: string
  part_label: string
  part_stem: string
  candidates: AssemblyCandidate[]
}

export interface AssemblySummary {
  mass_kg?: number
  wheelbase_m?: number
  cg_height_m?: number
  track_front_m?: number
  track_rear_m?: number
  wheel_radius_m?: number
  tire_mu?: number
  drive_type?: string
  error?: string
}

export interface AssemblyPreview {
  slot: string
  candidate: string
  summary: AssemblySummary
  delta: Record<string, number>
  candidate_info?: AssemblyCandidate
}

export interface AssemblyView {
  blueprint: { id: string; label: string; default_level?: string }
  blueprints: { id: string; label: string; level?: string }[]
  recommended: { id: string; label: string }[]
  categories: { id: string; label: string; slots: string[] }[]
  level: string
  parts: Record<string, string>
  slots: AssemblySlot[]
  summary: AssemblySummary
  build_complete: boolean
  preview?: AssemblyPreview
}

export interface SuspensionSchematic {
  name: string
  type: string
  links: [[number, number, number], [number, number, number]][]
  points?: Record<string, [number, number, number]>
}

export const FLEET_LEVELS = ['K', 'L1', 'L2', 'L3', 'L4', 'L5'] as const

export async function fetchAssembly(
  vehicleId: number,
  opts?: { slot?: string; candidate?: string },
): Promise<AssemblyView> {
  const qs = new URLSearchParams({ vehicle_id: String(vehicleId) })
  if (opts?.slot) qs.set('slot', opts.slot)
  if (opts?.candidate) qs.set('candidate', opts.candidate)
  const r = await fetchJson<{ ok: boolean; assembly: AssemblyView }>(
    `/api/catalog/assembly?${qs}`,
  )
  return r.assembly
}

export async function fetchSuspensionSchematic(
  name: string,
): Promise<SuspensionSchematic | null> {
  if (!name) return null
  const r = await fetchJson<SuspensionSchematic & { ok?: boolean }>(
    `/api/suspension/schematic?name=${encodeURIComponent(name)}`,
  )
  if (r.ok === false) return null
  return r
}

export function chassisStem(partId: string): string {
  const dot = partId.indexOf('.')
  return dot >= 0 ? partId.slice(dot + 1) : partId
}

export function slotCompatTone(slot: AssemblySlot): '' | 'warn' | 'err' {
  const cur = slot.candidates?.find((c) => c.id === slot.part_id)
  if (!cur?.compat?.length) return ''
  if (cur.compat.some((c) => c.level === 'error')) return 'err'
  if (cur.compat.some((c) => c.level === 'warn')) return 'warn'
  return ''
}

export function fmtDelta(key: string, v: number): string {
  if (!Number.isFinite(v) || Math.abs(v) < 1e-9) return ''
  const sign = v > 0 ? '+' : ''
  if (key === 'mass_kg') return `${sign}${v.toFixed(0)} kg`
  if (key.includes('_m')) return `${sign}${v.toFixed(3)} m`
  if (key === 'tire_mu') return `${sign}${v.toFixed(2)} μ`
  return `${sign}${v.toFixed(3)}`
}

export const PART_LIB_TYPES = [
  ['body', 'Body'],
  ['aero', 'Aero'],
  ['ride', 'Ride'],
  ['chassis', 'Chassis'],
  ['tire', 'Tire'],
  ['brake', 'Brake'],
  ['steering', 'Steering'],
  ['drivetrain', 'Drivetrain'],
] as const

const INSTALL_SLOT: Record<string, string> = {
  body: 'body',
  aero: 'aero',
  ride: 'ride',
  tire: 'tire',
  brake: 'brake',
  steering: 'steering',
  drivetrain: 'drivetrain',
}

export function installSlotForPartType(typeName: string, partId: string): string | null {
  if (typeName === 'chassis') {
    const s = partId.toLowerCase()
    if (s.includes('rear')) return 'rear_chassis'
    if (s.includes('front')) return 'front_chassis'
    return 'front_chassis'
  }
  return INSTALL_SLOT[typeName] ?? null
}

export interface CatalogPart {
  id: string
  label: string
  type?: string
  tags?: string[]
  card?: PartCard
}

export interface PartSchemaField {
  schema?: string
  fields?: [string, string, string][]
  meta_fields?: [string, string, string][]
  array_fields?: [string, string, number][]
  bool_fields?: [string, string][]
}

export interface PartEditor {
  ok?: boolean
  doc?: { id: string; type: string }
  schema?: PartSchemaField
  values?: Record<string, unknown>
  yaml?: string
  editable?: boolean
}

export async function exportBlueprintYaml(blueprintId: string): Promise<{ yaml: string }> {
  const r = await fetchJson<{ ok: boolean; yaml: string; blueprint?: unknown }>(
    `/api/catalog/blueprints/${encodeURIComponent(blueprintId)}/export`,
  )
  return { yaml: r.yaml || '' }
}

export function downloadText(filename: string, text: string, mime = 'text/yaml'): void {
  const blob = new Blob([text], { type: mime })
  const a = document.createElement('a')
  a.href = URL.createObjectURL(blob)
  a.download = filename
  a.click()
  URL.revokeObjectURL(a.href)
}

export async function registerBlueprintFromFleet(
  vehicleId: number,
  stem: string,
  label: string,
): Promise<{ blueprint_id?: string }> {
  return fetchJson('/api/catalog/blueprint/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ vehicle_id: vehicleId, stem, label }),
  })
}

export async function fetchCatalogParts(
  type: string,
  opts?: { q?: string; tag?: string; sort?: string },
): Promise<CatalogPart[]> {
  const qs = new URLSearchParams({ type })
  if (opts?.q) qs.set('q', opts.q)
  if (opts?.tag) qs.set('tag', opts.tag)
  if (opts?.sort) qs.set('sort', opts.sort ?? 'label')
  const r = await fetchJson<{ parts: CatalogPart[] }>(`/api/catalog/parts?${qs}`)
  return r.parts ?? []
}

export async function fetchPartEditor(opts: {
  type: string
  part_id?: string
  stem?: string
  label?: string
  clone?: boolean
}): Promise<PartEditor> {
  const qs = new URLSearchParams({ type: opts.type })
  if (opts.part_id) qs.set('part_id', opts.part_id)
  if (opts.stem) qs.set('stem', opts.stem)
  if (opts.label) qs.set('label', opts.label)
  if (opts.clone) qs.set('clone', '1')
  return fetchJson<PartEditor>(`/api/catalog/parts/editor?${qs}`)
}

export async function savePartFields(body: {
  type: string
  stem: string
  label: string
  fields: Record<string, unknown>
  doc?: unknown
}): Promise<{ part_id: string; package?: string }> {
  return fetchJson('/api/catalog/parts/save-fields', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
}

export async function deleteCatalogPart(partId: string): Promise<void> {
  await fetchJson('/api/catalog/parts/delete', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ part_id: partId }),
  })
}

export async function importPartYaml(yaml: string): Promise<{ part_id: string }> {
  return fetchJson('/api/catalog/parts/import-yaml', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ yaml }),
  })
}
