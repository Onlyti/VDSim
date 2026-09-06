<script lang="ts">
  import { onMount } from 'svelte'
  import { modeHref, onModeChange, type Mode } from './lib/router'
  import { pingApi } from './lib/api/client'
  import { playSimulation, refreshSetup, setupStore } from './lib/setupStore.svelte'
  import ScenarioMode from './modes/scenario/ScenarioMode.svelte'
  import RunMode from './modes/run/RunMode.svelte'
  import BuildMode from './modes/build/BuildMode.svelte'
  import AnalyzeMode from './modes/analyze/AnalyzeMode.svelte'

  let mode = $state<Mode>('scenario')
  let connected = $state(false)

  onMount(() => {
    if (!location.hash || location.hash === '#') location.hash = '#/scenario'
    const off = onModeChange((m) => { mode = m })
    pingApi().then((ok) => { connected = ok })
    refreshSetup()
    const t = setInterval(() => pingApi().then((ok) => { connected = ok }), 8000)
    return () => { off(); clearInterval(t) }
  })

  function go(m: Mode) {
    mode = m
    location.hash = modeHref(m)
  }

  async function play() {
    if (!await playSimulation()) return
    mode = 'run'
    location.hash = modeHref('run')
  }

  const draftLabel = $derived(
    setupStore.dirty ? '● draft modified' : '● draft synced',
  )
</script>

<div class="shell">
  <header class="top">
    <div class="brand">VDSim</div>
    <nav class="nav">
      <button class:primary={mode === 'scenario'} onclick={() => go('scenario')}>
        Scenario {#if mode === 'scenario'}★{/if}
      </button>
      <button class:primary={mode === 'run'} onclick={() => go('run')}>Run</button>
      <span class="sep">·</span>
      <button class:secondary={mode === 'build'} onclick={() => go('build')}>Build</button>
      <button class:secondary={mode === 'analyze'} onclick={() => go('analyze')}>Analyze</button>
    </nav>
    <div class="top-right">
      {#if setupStore.loadedScenario}
        <span class="scenario-name">{setupStore.loadedScenario}.yaml</span>
      {/if}
      <span class="badge" class:dirty={setupStore.dirty}>{draftLabel}</span>
      <button class="play" onclick={play} disabled={setupStore.busy || !setupStore.setup}>▶ Play</button>
      <span class="conn" class:ok={connected} title="API /api/state">{connected ? '●' : '○'} api</span>
    </div>
  </header>

  <main class="workspace">
    {#if mode === 'scenario'}
      <ScenarioMode />
    {:else if mode === 'run'}
      <RunMode />
    {:else if mode === 'build'}
      <BuildMode />
    {:else}
      <AnalyzeMode />
    {/if}
  </main>

  <footer class="status">
    <span>Mode: {mode}</span>
    {#if setupStore.setup?.running}
      <span class="run">sim running</span>
    {/if}
    <span class="hint">Map: drag fleet · Path from preset · Sync before Play</span>
  </footer>
</div>

<style>
  .shell {
    display: flex;
    flex-direction: column;
    height: 100%;
  }
  .top {
    display: flex;
    align-items: center;
    gap: 16px;
    height: var(--nav-h);
    padding: 0 12px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    flex-shrink: 0;
  }
  .brand {
    font-weight: 700;
    font-size: var(--fs-lg);
    color: var(--acc);
  }
  .nav {
    display: flex;
    align-items: center;
    gap: 4px;
  }
  .nav button {
    border: none;
    background: transparent;
    padding: 6px 10px;
    font-size: var(--fs-sm);
    color: var(--dim);
    border-radius: var(--radius);
  }
  .nav button.primary {
    color: var(--ink);
    font-weight: 600;
    box-shadow: inset 0 -2px 0 var(--acc);
  }
  .nav button.secondary {
    font-size: var(--fs-xs);
  }
  .sep { color: var(--border); font-size: var(--fs-xs); }
  .top-right {
    margin-left: auto;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .scenario-name {
    font-size: var(--fs-xs);
    color: var(--dim);
  }
  .badge {
    font-size: var(--fs-xs);
    color: var(--ok);
  }
  .badge.dirty { color: var(--warn); }
  .play {
    background: var(--acc);
    color: #001018;
    border: none;
    border-radius: var(--radius);
    padding: 5px 12px;
    font-weight: 600;
    font-size: var(--fs-sm);
  }
  .play:disabled { opacity: 0.5; }
  .conn { font-size: var(--fs-xs); color: var(--dim); }
  .conn.ok { color: var(--ok); }
  .workspace {
    flex: 1;
    min-height: 0;
    overflow: hidden;
  }
  .status {
    display: flex;
    gap: 16px;
    height: var(--status-h);
    align-items: center;
    padding: 0 12px;
    font-size: var(--fs-xs);
    color: var(--dim);
    border-top: 1px solid var(--border);
    background: var(--surface);
  }
  .run { color: var(--ok); }
</style>
