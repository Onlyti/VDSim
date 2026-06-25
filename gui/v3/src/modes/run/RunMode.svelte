<script lang="ts">
  import { onMount } from 'svelte'
  import ViewportPane from '../../components/ViewportPane.svelte'
  import HelpLink from '../../components/HelpLink.svelte'
  import {
    controlAction,
    getTimeScale,
    setFleetDriver,
    setTimeScale,
  } from '../../lib/api/control'
  import { createManualControl } from '../../lib/manualControl'
  import { modeHref } from '../../lib/router'
  import { pickVehicleState, type SimStreamState } from '../../lib/viewport/types'
  import { setupStore } from '../../lib/setupStore.svelte'
  import type { CamMode } from '../../lib/viewport/RunViewport'

  const WHEELS = ['FL', 'FR', 'RL', 'RR']

  let camMode = $state<CamMode>('chase')
  let showForces = $state(true)
  let tel = $state<SimStreamState | null>(null)
  let timeScale = $state(1)
  let running = $state(false)
  let driver = $state(true)
  let touchSteer = $state(0)
  let status = $state('')

  const vid = $derived(setupStore.selectedVid)
  let manualCtl: ReturnType<typeof createManualControl> | null = null

  onMount(() => {
    void getTimeScale().then((v) => { timeScale = v })
    manualCtl = createManualControl({
      isManual: () => !driver,
      isRunning: () => running,
      getVid: () => vid,
    })
    manualCtl.bind()
    return () => manualCtl?.dispose()
  })

  function onStream(s: SimStreamState) {
    running = !!(s.running && !s.setup_mode)
    if (s.driver != null) driver = !!s.driver
    status = s.plant_error ?? (s.kinematics_warnings?.[0] ?? '')
    tel = pickVehicleState(s, vid)
  }

  async function setDriverMode(drv: boolean) {
    driver = drv
    await setFleetDriver(vid, drv)
  }

  function onTouchSteer(v: number) {
    touchSteer = v
    manualCtl?.setTouch(0, 0, v)
  }

  function holdThr(on: boolean) {
    manualCtl?.setTouch(on ? 0.7 : 0, 0, touchSteer)
  }

  function holdBrk(on: boolean) {
    manualCtl?.setTouch(0, on ? 0.6 : 0, touchSteer)
  }

  function fmt(n: number | undefined, d = 2) {
    return n == null || Number.isNaN(n) ? '—' : n.toFixed(d)
  }

  function wheelFt(i: number, axis: 0 | 1): string {
    const row = tel?.Ft?.[i]
    return row ? fmt(row[axis], 0) : '—'
  }

  function wheelFz(i: number): string {
    const v = tel?.Fz?.[i]
    return v == null ? '—' : String(Math.round(v))
  }

  function wheelKappa(i: number): string {
    const v = tel?.kappa?.[i]
    return v == null ? '—' : v.toFixed(3)
  }

  function wheelAlphaDeg(i: number): string {
    const v = tel?.alpha?.[i]
    return v == null ? '—' : (v * 57.2958).toFixed(1)
  }

  async function stop() {
    await controlAction('stop')
  }

  async function reset() {
    await controlAction('reset')
  }

  async function onTimeScale() {
    await setTimeScale(timeScale)
  }

  function toScenario() {
    location.hash = modeHref('scenario')
  }
</script>

<div class="run">
  <div class="main">
    <div class="cam-bar">
      <button class:on={camMode === 'orbit'} onclick={() => { camMode = 'orbit' }}>Orbit</button>
      <button class:on={camMode === 'chase'} onclick={() => { camMode = 'chase' }}>Chase</button>
      <button class:on={camMode === 'top'} onclick={() => { camMode = 'top' }}>Top</button>
      <span class="sep">|</span>
      <button class:on={showForces} onclick={() => { showForces = !showForces }}>Forces</button>
      <span class="sep">|</span>
      <button class:on={driver} onclick={() => setDriverMode(true)}>Autopilot</button>
      <button class:on={!driver} onclick={() => setDriverMode(false)}>Manual</button>
      <span class="sep">|</span>
      <button onclick={stop}>⏹ Stop</button>
      <button onclick={reset}>⟲ Reset</button>
      <label class="ts">
        {timeScale.toFixed(1)}×
        <input
          type="range"
          min="0.1"
          max="3"
          step="0.1"
          bind:value={timeScale}
          onchange={onTimeScale}
        />
      </label>
      {#if running}
        <span class="live">● running</span>
        {#if tel?.t != null}
          <span class="simt">t = {tel.t.toFixed(2)} s</span>
        {/if}
      {:else}
        <span class="idle">setup</span>
      {/if}
    </div>
    <div class="viewport">
      <ViewportPane {camMode} {showForces} onState={onStream} />
      {#if !driver && running}
        <div class="manbar">
          <button type="button" onpointerdown={() => holdThr(true)} onpointerup={() => holdThr(false)}
            onpointerleave={() => holdThr(false)}>▲ throttle</button>
          <button type="button" onpointerdown={() => holdBrk(true)} onpointerup={() => holdBrk(false)}
            onpointerleave={() => holdBrk(false)}>▼ brake</button>
          <input
            type="range"
            min="-0.4"
            max="0.4"
            step="0.02"
            bind:value={touchSteer}
            oninput={() => onTouchSteer(touchSteer)}
          />
          <span class="hint-kb">↑↓←→ keys</span>
        </div>
      {/if}
    </div>
    {#if status}
      <div class="warn">{status}</div>
    {/if}
  </div>
  <aside class="tel">
    <h2>Vehicle {vid}</h2>
    {#if tel?.x != null}
      <div class="grp">Pose <HelpLink topic="frames" /></div>
      <div class="kv"><span>x</span><b>{fmt(tel.x)}</b></div>
      <div class="kv"><span>y</span><b>{fmt(tel.y)}</b></div>
      <div class="kv"><span>z</span><b>{fmt(tel.z)}</b></div>
      <div class="kv"><span>yaw</span><b>{fmt(tel.yaw, 3)}</b></div>
      <div class="grp">Velocity</div>
      <div class="kv"><span>vx</span><b>{fmt(tel.vx)} m/s</b></div>
      <div class="kv"><span>vy</span><b>{fmt(tel.vy)}</b></div>
      <div class="kv"><span>yaw rate</span><b>{fmt(tel.r, 3)}</b></div>
      <div class="grp">Per wheel <HelpLink topic="Tire" /></div>
      <table class="wh">
        <thead>
          <tr><th></th>{#each WHEELS as w}<th>{w}</th>{/each}</tr>
        </thead>
        <tbody>
          <tr><td>Fz</td>{#each WHEELS as _, i}<td>{wheelFz(i)}</td>{/each}</tr>
          <tr><td>Fx</td>{#each WHEELS as _, i}<td>{wheelFt(i, 0)}</td>{/each}</tr>
          <tr><td>Fy</td>{#each WHEELS as _, i}<td>{wheelFt(i, 1)}</td>{/each}</tr>
          <tr><td>κ</td>{#each WHEELS as _, i}<td>{wheelKappa(i)}</td>{/each}</tr>
          <tr><td>α°</td>{#each WHEELS as _, i}<td>{wheelAlphaDeg(i)}</td>{/each}</tr>
        </tbody>
      </table>
      <div class="legend">Fx red · Fy green · Fz blue (contact frame)</div>
    {:else}
      <p class="hint">Press ▶ Play from Scenario or wait for stream…</p>
    {/if}
    <button type="button" class="back" onclick={toScenario}>← Scenario</button>
  </aside>
</div>

<style>
  .run {
    display: grid;
    grid-template-columns: 1fr 260px;
    height: 100%;
  }
  .main {
    display: flex;
    flex-direction: column;
    min-width: 0;
    min-height: 0;
  }
  .cam-bar {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 6px 10px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    font-size: var(--fs-xs);
    flex-shrink: 0;
    flex-wrap: wrap;
  }
  .cam-bar button {
    border: 1px solid var(--border);
    background: var(--surface);
    border-radius: 4px;
    padding: 4px 8px;
    font-size: var(--fs-xs);
  }
  .cam-bar button.on {
    border-color: var(--acc);
    color: var(--acc);
    font-weight: 600;
  }
  .sep { color: var(--border); }
  .ts {
    display: flex;
    align-items: center;
    gap: 4px;
    margin-left: auto;
    color: var(--dim);
  }
  .ts input { width: 72px; }
  .live { color: var(--ok); font-weight: 600; }
  .simt { color: var(--dim); font-variant-numeric: tabular-nums; }
  .idle { color: var(--dim); }
  .viewport {
    flex: 1;
    min-height: 0;
    position: relative;
  }
  .manbar {
    position: absolute;
    bottom: 12px;
    left: 50%;
    transform: translateX(-50%);
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 8px 12px;
    background: rgba(255, 255, 255, 0.92);
    border: 1px solid var(--border);
    border-radius: 8px;
    font-size: var(--fs-xs);
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
  }
  .manbar button {
    border: 1px solid var(--border);
    background: var(--surface);
    border-radius: 4px;
    padding: 6px 10px;
  }
  .manbar input { width: min(38vw, 200px); }
  .hint-kb { color: var(--dim); }
  .warn {
    background: #fff8e6;
    color: #92400e;
    font-size: var(--fs-xs);
    padding: 4px 10px;
    border-top: 1px solid #fde68a;
  }
  .tel {
    padding: 12px;
    background: var(--surface);
    border-left: 1px solid var(--border);
    overflow: auto;
    font-size: var(--fs-sm);
  }
  h2 { margin: 0 0 10px; font-size: var(--fs-md); }
  .grp {
    font-size: var(--fs-xs);
    text-transform: uppercase;
    color: var(--dim);
    margin: 10px 0 4px;
  }
  .kv {
    display: flex;
    justify-content: space-between;
    margin-bottom: 2px;
    color: var(--dim);
  }
  .kv b { color: var(--ink); }
  .wh {
    width: 100%;
    border-collapse: collapse;
    font-size: var(--fs-xs);
    font-variant-numeric: tabular-nums;
  }
  .wh th, .wh td {
    border: 1px solid var(--border);
    padding: 2px 4px;
    text-align: right;
  }
  .wh th:first-child, .wh td:first-child {
    text-align: left;
    color: var(--dim);
  }
  .legend {
    margin-top: 4px;
    font-size: 10px;
    color: var(--dim);
  }
  .hint { color: var(--dim); font-size: var(--fs-sm); }
  .back {
    margin-top: 16px;
    width: 100%;
    padding: 6px;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--bg);
    font-size: var(--fs-sm);
  }
</style>
