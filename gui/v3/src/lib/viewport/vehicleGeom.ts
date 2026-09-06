export interface VehicleGeom {
  cg_to_front: number
  cg_to_rear: number
  track_front: number
  track_rear: number
  wheel_radius_nominal: number
}

const DEFAULT_GEOM: VehicleGeom = {
  cg_to_front: 1.25,
  cg_to_rear: 1.55,
  track_front: 1.55,
  track_rear: 1.55,
  wheel_radius_nominal: 0.33,
}

const cache: Record<string, VehicleGeom> = {}

export async function fetchVehicleGeom(vid: number): Promise<VehicleGeom> {
  const k = String(vid)
  if (cache[k]) return cache[k]
  try {
    const r = await fetch(`/api/vehicle?vehicle_id=${vid}`)
    const j = await r.json()
    const g = { ...DEFAULT_GEOM }
    for (const f of j.fields ?? []) {
      if (f.name in g) (g as Record<string, number>)[f.name] = f.value
    }
    cache[k] = g
    return g
  } catch {
    return { ...DEFAULT_GEOM }
  }
}
