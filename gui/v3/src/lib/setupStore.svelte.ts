import {
  addFleetVehicle,
  DEG2RAD,
  applyScenarioTemplate as apiApplyTemplate,
  getPathPoints,
  getSetup,
  loadScenario,
  postSetup,
  roadBankDeg,
  roadGradeDeg,
  saveScenario,
  startRun,
} from './api/setup'
import type { FleetAgent, Setup, SetupPatch } from './api/types'

export const setupStore = $state({
  setup: null as Setup | null,
  pathPts: [] as [number, number][],
  dirty: false,
  busy: false,
  error: null as string | null,
  loadedScenario: '',
  selectedVid: 0,
})

export function scenarios(): string[] {
  return setupStore.setup?.scenarios ?? setupStore.setup?.scenes ?? []
}

export async function refreshSetup(): Promise<void> {
  setupStore.busy = true
  setupStore.error = null
  try {
    const s = await getSetup()
    setupStore.setup = s
    setupStore.pathPts =
      s.path_pts?.length ? s.path_pts.map((p) => [p[0], p[1]] as [number, number]) : await getPathPoints()
    if (s.fleet?.length) {
      const ids = s.fleet.map((f) => f.id)
      if (!ids.includes(setupStore.selectedVid)) {
        setupStore.selectedVid = s.fleet[0].id
      }
    }
    setupStore.dirty = false
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
  } finally {
    setupStore.busy = false
  }
}

export function selectedAgent(): FleetAgent | null {
  return setupStore.setup?.fleet.find((f) => f.id === setupStore.selectedVid) ?? null
}

function defaultRoadPatch(): SetupPatch['road'] {
  const r = setupStore.setup?.road
  return {
    mu: r?.mu ?? 1,
    grade: r?.grade ?? 0,
    bank: r?.bank ?? 0,
    mu_right: r?.mu_right ?? -1,
    mu_boundary: r?.mu_boundary ?? 0,
    rough_amp: r?.rough_amp ?? 0,
    rough_wl: r?.rough_wl ?? 4,
  }
}

export function buildPayload(): SetupPatch {
  const s = setupStore.setup
  return {
    path_preset: s?.path_preset ?? 'figure8',
    fleet: s?.fleet ?? [],
    road: defaultRoadPatch(),
    v_target: s?.v_target ?? 10,
  }
}

export async function syncDraft(): Promise<boolean> {
  setupStore.busy = true
  setupStore.error = null
  try {
    const j = await postSetup(buildPayload())
    if (j.setup) setupStore.setup = { ...setupStore.setup!, ...j.setup, fleet: j.setup.fleet ?? setupStore.setup!.fleet }
    const pts = setupStore.setup?.path_pts
    setupStore.pathPts =
      pts?.length ? pts.map((p) => [p[0], p[1]] as [number, number]) : await getPathPoints()
    setupStore.dirty = false
    return true
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
    return false
  } finally {
    setupStore.busy = false
  }
}

export async function patchSetup(patch: SetupPatch): Promise<boolean> {
  setupStore.busy = true
  setupStore.error = null
  try {
    const j = await postSetup(patch)
    if (j.setup) {
      setupStore.setup = {
        ...setupStore.setup!,
        ...j.setup,
        fleet: j.setup.fleet ?? setupStore.setup!.fleet,
        road: { ...setupStore.setup!.road, ...j.setup.road },
      }
    }
    const pts = setupStore.setup?.path_pts
    setupStore.pathPts =
      pts?.length ? pts.map((p) => [p[0], p[1]] as [number, number]) : await getPathPoints()
    setupStore.dirty = false
    return true
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
    return false
  } finally {
    setupStore.busy = false
  }
}

export function markDirty(): void {
  setupStore.dirty = true
}

export function setMu(mu: number): void {
  if (!setupStore.setup) return
  setupStore.setup.road.mu = mu
  markDirty()
}

export function setGradeDeg(deg: number): void {
  if (!setupStore.setup) return
  setupStore.setup.road.grade = deg * DEG2RAD
  markDirty()
}

export function setBankDeg(deg: number): void {
  if (!setupStore.setup) return
  setupStore.setup.road.bank = deg * DEG2RAD
  markDirty()
}

export function setVTarget(v: number): void {
  if (!setupStore.setup) return
  setupStore.setup.v_target = v
  markDirty()
}

export async function setPathPreset(preset: string): Promise<void> {
  if (!setupStore.setup) return
  setupStore.setup.path_preset = preset
  await patchSetup({ path_preset: preset })
}

let pathCommitTimer: ReturnType<typeof setTimeout> | undefined

export function setLocalPathPts(path_pts: [number, number][]): void {
  setupStore.pathPts = path_pts
  if (setupStore.setup) setupStore.setup.path_preset = 'custom'
  markDirty()
}

export async function commitCustomPath(path_pts: [number, number][]): Promise<void> {
  setLocalPathPts(path_pts)
  clearTimeout(pathCommitTimer)
  pathCommitTimer = setTimeout(async () => {
    await patchSetup({ path_preset: 'custom', path_pts })
  }, 180)
}

export async function moveFleetAgent(vid: number, x0: number, y0: number): Promise<void> {
  if (!setupStore.setup) return
  const fleet = setupStore.setup.fleet.map((f) =>
    f.id === vid ? { ...f, x0, y0 } : f,
  )
  setupStore.setup.fleet = fleet
  await patchSetup({ fleet: [{ id: vid, x0, y0 }] })
}

export async function applyScenarioTemplate(template: string): Promise<boolean> {
  if (
    setupStore.dirty &&
    !confirm('Unsaved draft changes will be lost. Apply template?')
  ) {
    return false
  }
  setupStore.busy = true
  setupStore.error = null
  try {
    const j = await apiApplyTemplate(template)
    if (!j.ok) throw new Error(j.error || 'template failed')
    setupStore.loadedScenario = ''
    if (j.setup) {
      setupStore.setup = { ...setupStore.setup!, ...j.setup, fleet: j.setup.fleet ?? [] }
      setupStore.pathPts =
        j.setup.path_pts?.length
          ? j.setup.path_pts.map((p) => [p[0], p[1]] as [number, number])
          : await getPathPoints()
      if (j.setup.fleet?.length) setupStore.selectedVid = j.setup.fleet[0].id
    } else {
      await refreshSetup()
    }
    setupStore.dirty = false
    return true
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
    return false
  } finally {
    setupStore.busy = false
  }
}

export async function loadScenarioByName(name: string): Promise<boolean> {
  if (!name) return false
  if (setupStore.dirty && !confirm('Unsaved draft changes will be lost. Load scenario?')) {
    return false
  }
  setupStore.busy = true
  try {
    const j = await loadScenario(name)
    if (!j.ok) throw new Error(j.error || 'load failed')
    setupStore.loadedScenario = name
    if (j.live_vid != null) setupStore.selectedVid = j.live_vid
    await refreshSetup()
    return true
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
    return false
  } finally {
    setupStore.busy = false
  }
}

export async function saveScenarioDraft(name: string, overwrite = false): Promise<boolean> {
  if (!name.trim()) return false
  const ok = await syncDraft()
  if (!ok) return false
  try {
    const j = await saveScenario(name.trim(), overwrite)
    if (!j.ok) throw new Error(j.error || 'save failed')
    setupStore.loadedScenario = j.name ?? name
    await refreshSetup()
    return true
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
    return false
  }
}

export async function addVehicle(): Promise<void> {
  const j = await addFleetVehicle()
  if (j.setup?.fleet) {
    setupStore.setup = { ...setupStore.setup!, fleet: j.setup.fleet }
    const last = j.setup.fleet[j.setup.fleet.length - 1]
    if (last) setupStore.selectedVid = last.id
    setupStore.pathPts = await getPathPoints()
  }
}

export async function patchFleet(vid: number, patch: Partial<FleetAgent>): Promise<boolean> {
  return patchSetup({ fleet: [{ id: vid, ...patch }] })
}

export async function setFleetBlueprint(
  vid: number,
  blueprintId: string,
  opts?: { confirm?: boolean },
): Promise<boolean> {
  const agent = setupStore.setup?.fleet.find((f) => f.id === vid)
  if (agent?.blueprint === blueprintId) return true
  if (
    opts?.confirm !== false &&
    agent?.blueprint &&
    !confirm('Changing blueprint replaces catalog parts for this vehicle. Continue?')
  ) {
    return false
  }
  return patchFleet(vid, { blueprint: blueprintId })
}

export async function setFleetPart(vid: number, slot: string, partId: string): Promise<boolean> {
  return patchFleet(vid, { parts: { [slot]: partId } })
}

export async function setFleetLevel(vid: number, level: string): Promise<boolean> {
  return patchFleet(vid, { level })
}

export async function playSimulation(): Promise<boolean> {
  if (!setupStore.setup) return false
  const ok = await syncDraft()
  if (!ok) return false
  try {
    const j = await startRun()
    if (j.ok === false) throw new Error(j.error || 'start failed')
    return true
  } catch (e) {
    setupStore.error = e instanceof Error ? e.message : String(e)
    return false
  }
}

export { roadGradeDeg, roadBankDeg }
