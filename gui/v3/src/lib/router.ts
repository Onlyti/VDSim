export type Mode = 'scenario' | 'run' | 'build' | 'analyze'

const MODES: Mode[] = ['scenario', 'run', 'build', 'analyze']

export function parseMode(hash: string): Mode {
  const seg = hash.replace(/^#\/?/, '').split('/')[0]
  return MODES.includes(seg as Mode) ? (seg as Mode) : 'scenario'
}

export function modeHref(mode: Mode): string {
  return `#/${mode}`
}

export function onModeChange(cb: (mode: Mode) => void): () => void {
  const fn = () => cb(parseMode(location.hash))
  fn()
  window.addEventListener('hashchange', fn)
  return () => window.removeEventListener('hashchange', fn)
}

export function parseVidFromHash(hash: string): number | null {
  const q = hash.split('?')[1]
  if (!q) return null
  const raw = new URLSearchParams(q).get('vid')
  if (raw == null) return null
  const n = parseInt(raw, 10)
  return Number.isFinite(n) ? n : null
}

export function buildHref(vid?: number): string {
  return vid == null ? '#/build' : `#/build?vid=${vid}`
}
