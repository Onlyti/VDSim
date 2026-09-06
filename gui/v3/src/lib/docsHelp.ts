export const DOCS = 'https://onlyti.github.io/VDSim/main/theory/'

export const DOC_MAP: Record<string, string> = {
  road: '03_tire_pacejka_mf96',
  run: '09_pure_pursuit_path',
  frames: '01_frames_and_conventions',
  Tire: '03_tire_pacejka_mf96',
  Steering: '05_ld2_seven_dof',
  numerics: '11_numerical_integration',
  cosim: '18_runtime_and_cosim',
}

export function docHref(key: string): string | null {
  const f = DOC_MAP[key]
  return f === undefined ? null : DOCS + (f ? `${f}/` : '')
}
