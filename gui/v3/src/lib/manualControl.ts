import { postManual } from './api/control'

const ARROW_KEYS = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'])

export type ManualCmd = { throttle: number; brake: number; steer: number }

export function createManualControl(opts: {
  isManual: () => boolean
  isRunning: () => boolean
  getVid: () => number
}) {
  const keys: Record<string, boolean> = {}
  let touchThr = 0
  let touchBrk = 0
  let touchSteer = 0
  let sendBusy = false
  let sendLatest: ManualCmd | null = null

  function sample(): ManualCmd {
    const throttle = Math.max(keys.ArrowUp ? 0.7 : 0, touchThr)
    const brake = Math.max(keys.ArrowDown ? 0.6 : 0, touchBrk)
    const steer = Math.max(
      -0.4,
      Math.min(0.4, (keys.ArrowLeft ? 0.25 : 0) - (keys.ArrowRight ? 0.25 : 0) + touchSteer),
    )
    return { throttle, brake, steer }
  }

  async function drain() {
    if (sendBusy) return
    sendBusy = true
    while (sendLatest) {
      const cmd = sendLatest
      sendLatest = null
      try {
        await postManual(opts.getVid(), cmd.throttle, cmd.brake, cmd.steer)
      } catch {
        /* optional */
      }
    }
    sendBusy = false
  }

  function push() {
    if (!opts.isRunning() || !opts.isManual()) {
      sendLatest = null
      return
    }
    sendLatest = sample()
    void drain()
  }

  function onKeyDown(e: KeyboardEvent) {
    if (opts.isManual() && opts.isRunning() && ARROW_KEYS.has(e.key)) e.preventDefault()
    keys[e.key] = true
    push()
  }

  function onKeyUp(e: KeyboardEvent) {
    keys[e.key] = false
    push()
  }

  const timer = setInterval(push, 40)

  return {
    setTouch(throttle: number, brake: number, steer: number) {
      touchThr = throttle
      touchBrk = brake
      touchSteer = steer
      push()
    },
    dispose() {
      clearInterval(timer)
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
    },
    bind() {
      window.addEventListener('keydown', onKeyDown)
      window.addEventListener('keyup', onKeyUp)
    },
  }
}
