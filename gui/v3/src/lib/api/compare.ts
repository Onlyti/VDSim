import { fetchJson } from './client'

export const COMPARE_MANEUVERS = [
  { id: 'step_steer', label: 'Step steer' },
  { id: 'skidpad', label: 'Skidpad understeer' },
  { id: 'dlc', label: 'Double lane change' },
] as const

export type CompareManeuverId = (typeof COMPARE_MANEUVERS)[number]['id']

export interface CompareTrace {
  t: number[]
  r: number[]
}

export interface CompareRow {
  vehicle: string
  [key: string]: string | number | undefined
}

export interface CompareResult {
  ok: boolean
  rows: CompareRow[]
  traces: Record<string, CompareTrace>
  columns: string[]
  maneuvers: string[]
  error?: string
}

export interface CompareRequest {
  vehicles: string[]
  maneuvers?: CompareManeuverId[]
  tire?: string | null
  level?: string
}

export async function runCompare(req: CompareRequest): Promise<CompareResult> {
  return fetchJson<CompareResult>('/api/compare', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(req),
  })
}

export function vehicleLabel(id: string): string {
  return id.replace(/^vehicle\./, '')
}

export function cmpFmt(v: unknown): string {
  const n = Number(v)
  return Number.isFinite(n) ? String(parseFloat(n.toPrecision(4))) : String(v ?? '')
}

export function cmpDeltaPct(
  base: unknown,
  v: unknown,
): { text: string; neg: boolean } | null {
  const b = Number(base)
  const x = Number(v)
  if (!Number.isFinite(b) || !Number.isFinite(x) || Math.abs(b) <= 1e-9) return null
  const d = ((x - b) / Math.abs(b)) * 100
  if (Math.abs(d) < 0.05) return null
  return { text: `${d > 0 ? '+' : ''}${d.toFixed(1)}%`, neg: d < 0 }
}

export function rowsToCsv(rows: CompareRow[], columns: string[]): string {
  const cols = ['vehicle', ...columns]
  const esc = (s: string) => (/[",\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s)
  const lines = [cols.join(',')]
  for (const r of rows) {
    lines.push(cols.map((c) => esc(String(r[c] ?? ''))).join(','))
  }
  return lines.join('\n')
}

export function downloadCsv(filename: string, csv: string): void {
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8' })
  const a = document.createElement('a')
  a.href = URL.createObjectURL(blob)
  a.download = filename
  a.click()
  URL.revokeObjectURL(a.href)
}
