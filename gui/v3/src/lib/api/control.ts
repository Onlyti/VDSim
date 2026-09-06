import { fetchJson } from './client'

export async function controlAction(action: 'start' | 'stop' | 'pause' | 'resume' | 'reset') {
  return fetchJson<{ ok: boolean; running?: boolean; error?: string }>('/api/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action }),
  })
}

export async function setTimeScale(scale: number) {
  return fetchJson('/api/sim', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ time_scale: scale }),
  })
}

export async function getTimeScale(): Promise<number> {
  try {
    const j = await fetchJson<{ config?: { time_scale?: number } }>('/api/config')
    return j.config?.time_scale ?? 1
  } catch {
    return 1
  }
}

export async function setFleetDriver(vid: number, driver: boolean) {
  return fetchJson<{ ok: boolean; driver?: boolean }>('/api/fleet', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ vehicle_id: vid, driver }),
  })
}

export async function postManual(
  vid: number,
  throttle: number,
  brake: number,
  steer: number,
) {
  return fetchJson('/api/manual', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ vehicle: vid, throttle, brake, steer }),
  })
}
