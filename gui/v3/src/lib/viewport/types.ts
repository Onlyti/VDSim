export interface SimStreamState {
  t?: number
  running?: boolean
  setup_mode?: boolean
  paused?: boolean
  driver?: boolean
  live_vid?: number
  level?: string
  x?: number
  y?: number
  z?: number
  yaw?: number
  roll?: number
  pitch?: number
  vx?: number
  vy?: number
  r?: number
  ax?: number
  ay?: number
  steer?: number
  throttle_applied?: number
  brake_applied?: number
  wheel_spin?: number[]
  kappa?: number[]
  alpha?: number[]
  Fz?: number[]
  Ft?: number[][]
  rack_torque?: number
  time_scale?: number
  plant_error?: string
  kinematics_warnings?: string[]
  fleet?: Record<string, Partial<SimStreamState>>
  fleet_spec?: { id: number; level?: string }[]
  fleet_cmd?: Record<string, { throttle?: number; brake?: number; steer?: number }>
  cmd_in?: { throttle?: number; brake?: number; steer?: number }
  npath?: number
}

export function pickVehicleState(s: SimStreamState, vid: number): SimStreamState {
  const f = s.fleet?.[String(vid)]
  if (f && f.x != null) return { ...s, ...f }
  if (s.live_vid === vid || vid === 0) return s
  return f ? { ...s, ...f } : s
}
