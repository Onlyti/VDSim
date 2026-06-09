// Manual driving input (keyboard + touch) extracted from app.js (B2b #3).
// The shared driver-mode state (manualMode) and the manbar toggle stay in app.js
// because they are woven through the driver-mode UI; this module only *reads*
// manualMode / simRunning / selectedVid, injected as closures, so there is no
// circular import. initManualControl() owns the input buffers, listeners and the
// 40 ms send loop; call it once where the inline block used to run.

import { $ } from './util.js';

export function initManualControl({ isManualMode, isRunning, getSelectedVid }) {
  const man = { throttle: 0, brake: 0, steer: 0 };
  const keys = {}; let touchThr = 0, touchBrk = 0, touchSteer = 0;
  const ARROW_KEYS = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);
  addEventListener('keydown', e => {
    if (isManualMode() && isRunning() && ARROW_KEYS.has(e.key)) e.preventDefault();
    keys[e.key] = true;
  });
  addEventListener('keyup', e => { keys[e.key] = false; });
  function driverManualActive() {
    return $('drv_manual')?.classList.contains('on');
  }
  function sampleManualCmd() {
    man.throttle = Math.max(keys['ArrowUp'] ? 0.7 : 0, touchThr);
    man.brake = Math.max(keys['ArrowDown'] ? 0.6 : 0, touchBrk);
    man.steer = Math.max(-0.4, Math.min(0.4,
      (keys['ArrowLeft'] ? 0.25 : 0) - (keys['ArrowRight'] ? 0.25 : 0) + touchSteer));
    return { throttle: man.throttle, brake: man.brake, steer: man.steer, vehicle: getSelectedVid() };
  }
  let manualSendBusy = false;
  let manualSendLatest = null;
  async function drainManualSend() {
    if (manualSendBusy) return;
    manualSendBusy = true;
    while (manualSendLatest !== null) {
      const body = manualSendLatest;
      manualSendLatest = null;
      try {
        await fetch('/api/manual', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        });
      } catch (_) {}
    }
    manualSendBusy = false;
  }
  function pushManualSend() {
    if (!isRunning() || !driverManualActive()) {
      manualSendLatest = null;
      return;
    }
    manualSendLatest = sampleManualCmd();
    void drainManualSend();
  }
  setInterval(pushManualSend, 40);
  function holdBtn(el, on, off) {
    ['pointerdown', 'touchstart'].forEach(ev => el.addEventListener(ev, e => { e.preventDefault(); on(); }, { passive: false }));
    ['pointerup', 'pointerleave', 'pointercancel', 'touchend'].forEach(ev => el.addEventListener(ev, off));
  }
  holdBtn($('m_thr'), () => touchThr = 0.7, () => touchThr = 0);
  holdBtn($('m_brk'), () => touchBrk = 0.6, () => touchBrk = 0);
  $('m_steer').addEventListener('input', e => touchSteer = +e.target.value);
  const recenterSteer = () => { $('m_steer').value = 0; touchSteer = 0; };
  ['pointerup', 'touchend', 'pointercancel'].forEach(ev => $('m_steer').addEventListener(ev, recenterSteer));
}
