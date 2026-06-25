export interface FleetAgent {
  id: number
  blueprint: string
  level?: string
  label?: string
  x0?: number
  y0?: number
  yaw0?: number
  vx0?: number
  driver?: boolean
  parts?: Record<string, string>
}

export interface SetupRoad {
  mu: number
  mu_right?: number
  mu_boundary?: number
  grade: number
  bank: number
  rough_amp?: number
  rough_wl?: number
}

export interface Setup {
  running: boolean
  paused?: boolean
  path_preset: string
  path_pts: [number, number][]
  fleet: FleetAgent[]
  road: SetupRoad
  v_target: number
  level?: string
  vehicle?: string
  scenarios?: string[]
  scenes?: string[]
  cosim_attach?: boolean
  infra_sensors?: unknown[]
  stunt?: Record<string, unknown>
}

export interface SetupPatch {
  path_preset?: string
  path_pts?: [number, number][]
  fleet?: Partial<FleetAgent>[]
  road?: Partial<SetupRoad>
  v_target?: number
  fleet_add?: boolean
  fleet_remove?: number
  cosim_attach?: boolean
}

export interface ApiOk<T = unknown> {
  ok: boolean
  error?: string
  setup?: Setup
  fleet?: FleetAgent[]
  live_vid?: number
  running?: boolean
}
