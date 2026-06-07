(function () {
  const paint = (t, col) => {
    const c = document.getElementById('conn');
    if (!c) return;
    c.textContent = t;
    if (col) c.style.color = col;
  };
  const fromState = s => {
    if (!s) return ['● no state', 'var(--dim)'];
    if (s.kinematics_warnings?.length) return ['● kinematics warning', '#ff9500'];
    if (s.plant_error) return ['● ' + s.plant_error, '#ff9500'];
    if (s.setup_mode || !s.running) return ['● setup — press ▶ Play', 'var(--dim)'];
    if (s.running && s.source === 'waiting') return ['● waiting for plant… (▶ Play)', '#ff9500'];
    if (s.running) return ['● running', '#34c759'];
    return ['● setup — press ▶ Play', 'var(--dim)'];
  };
  const paintState = s => {
    window.__bootState = s;
    window._lastState = s;
    const p = fromState(s);
    paint(p[0], p[1]);
    const run = !!(s && s.running && !s.setup_mode);
    document.getElementById('setup_panel')?.classList.toggle('hidden', run);
    document.getElementById('telemetry_aside')?.classList.toggle('hidden', !run);
    document.getElementById('sim_stop')?.classList.toggle('on', run);
  };
  async function apiPost(url, body) {
    const r = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body ?? {}),
    });
    let j = {};
    try { j = await r.json(); } catch (_e) {}
    if (!r.ok || j.ok === false) throw new Error(j.error || ('HTTP ' + r.status));
    return j;
  }
  async function refreshState() {
    const r = await fetch('/api/state');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const s = await r.json();
    paintState(s);
    if (typeof window.__vdsimApplyState === 'function') window.__vdsimApplyState(s);
    return s;
  }
  async function playSim() {
    paint('● starting…', '#ff9500');
    try {
      let s = await refreshState();
      const live = !!(s && s.running && !s.setup_mode);
      if (live) {
        const action = s.paused ? 'resume' : 'pause';
        await apiPost('/api/control', { action });
        await refreshState();
        return;
      }
      if (typeof window.__vdsimSyncTimeScale === 'function') {
        await window.__vdsimSyncTimeScale();
      }
      if (typeof window.__vdsimApplySetup === 'function') {
        const ok = await window.__vdsimApplySetup(true);
        if (!ok) { await refreshState(); return; }
      }
      if (typeof window.__vdsimOnPlayStart === 'function') {
        await window.__vdsimOnPlayStart();
      }
      const j = await apiPost('/api/run/start', {});
      if (!j.running) throw new Error(j.error || 'Simulation failed to start');
      if (j.error) alert(j.error);
      if (typeof window.__vdsimOnPlayDone === 'function') {
        await window.__vdsimOnPlayDone(j);
      }
      await refreshState();
    } catch (e) {
      alert('Play failed: ' + (e.message || e));
      try { await refreshState(); } catch (_e) {
        paint('● start failed — reload page', '#ff9500');
      }
    }
  }
  async function stopSim() {
    paint('● stopping…', '#ff9500');
    try {
      await apiPost('/api/control', { action: 'stop' });
      if (typeof window.__vdsimOnStop === 'function') await window.__vdsimOnStop();
      await refreshState();
    } catch (e) {
      alert('Stop failed: ' + (e.message || e));
      try { await refreshState(); } catch (_e) {}
    }
  }
  async function resetSim() {
    try {
      await apiPost('/api/control', { action: 'reset' });
      if (typeof window.__vdsimOnStop === 'function') await window.__vdsimOnStop();
      await refreshState();
    } catch (e) { alert('Reset failed: ' + (e.message || e)); }
  }
  function bind(id, fn) {
    const el = document.getElementById(id);
    if (el) el.addEventListener('click', fn);
  }
  function init() {
    bind('sim_play', () => { void playSim(); });
    bind('sim_stop', () => { void stopSim(); });
    bind('sim_reset', () => { void resetSim(); });
    const ac = new AbortController();
    setTimeout(() => ac.abort(), 12000);
    fetch('/api/state', { signal: ac.signal })
      .then(r => r.ok ? r.json() : Promise.reject(new Error('HTTP ' + r.status)))
      .then(paintState)
      .catch(e => paint(
        e.name === 'AbortError' ? '● server timeout — restart gui/server.py'
          : '● server unreachable — start gui/server.py',
        '#ff9500'));
  }
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
  window.addEventListener('error', ev => {
    const c = document.getElementById('conn');
    if (!c || !/loading|connecting/i.test(c.textContent)) return;
    paint('● JS error — open F12 console', '#ff9500');
  });
  window.addEventListener('unhandledrejection', ev => {
    console.error('VDSim:', ev.reason);
    paint('● error — see F12 console', '#ff9500');
  });
  window.__vdsimCore = { paintState, refreshState, playSim, stopSim, resetSim };
})();
