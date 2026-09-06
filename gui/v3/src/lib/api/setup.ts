import { fetchJson } from './client'
import type { ApiOk, Setup, SetupPatch } from './types'

export async function getSetup(): Promise<Setup> {
  return fetchJson<Setup>('/api/setup')
}

export async function getPathPoints(): Promise<[number, number][]> {
  const r = await fetchJson<{ pts: [number, number][] }>('/api/path')
  return r.pts ?? []
}

export async function postSetup(patch: SetupPatch): Promise<ApiOk> {
  return fetchJson<ApiOk>('/api/setup', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(patch),
  })
}

export async function applyScenarioTemplate(
  template: string,
): Promise<ApiOk & { setup?: Setup }> {
  return fetchJson('/api/setup/template', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ template }),
  })
}

export async function loadScenario(name: string): Promise<ApiOk> {
  return fetchJson<ApiOk>('/api/fleet', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scenario: name }),
  })
}

export async function saveScenario(name: string, overwrite = false): Promise<ApiOk & { name?: string }> {
  return fetchJson('/api/scenario/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, overwrite }),
  })
}

export async function addFleetVehicle(): Promise<ApiOk> {
  return fetchJson<ApiOk>('/api/setup', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ fleet_add: true }),
  })
}

export async function startRun(): Promise<ApiOk> {
  try {
    return await fetchJson<ApiOk>('/api/run/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({}),
    })
  } catch {
    return fetchJson<ApiOk>('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ action: 'start' }),
    })
  }
}

export const RAD2DEG = 180 / Math.PI
export const DEG2RAD = Math.PI / 180

export function roadGradeDeg(road: { grade?: number }): number {
  return (road.grade ?? 0) * RAD2DEG
}

export function roadBankDeg(road: { bank?: number }): number {
  return (road.bank ?? 0) * RAD2DEG
}
