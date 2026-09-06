<script lang="ts">
  import { onMount } from 'svelte'
  import MapCanvas from '../../components/MapCanvas.svelte'
  import { buildHref } from '../../lib/router'
  import {
    addVehicle,
    applyScenarioTemplate,
    commitCustomPath,
    loadScenarioByName,
    markDirty,
    moveFleetAgent,
    refreshSetup,
    roadBankDeg,
    roadGradeDeg,
    saveScenarioDraft,
    setBankDeg,
    setGradeDeg,
    setMu,
    setPathPreset,
    setVTarget,
    setupStore,
    syncDraft,
  } from '../../lib/setupStore.svelte'
  import { downsamplePathPts } from '../../lib/map/pathUtils'
  import HelpLink from '../../components/HelpLink.svelte'
  import { SCENARIO_TEMPLATES } from '../../lib/scenarioTemplates'

  const wpTable = $derived(
    downsamplePathPts(setupStore.pathPts.length ? setupStore.pathPts : [], 12),
  )
  let saveName = $state('')
  let gradeDeg = $state(0)
  let bankDeg = $state(0)
  let pathEditActive = $state(false)
  let activeTemplate = $state('')

  onMount(() => {
    refreshSetup().then(() => {
      if (setupStore.setup) {
        gradeDeg = roadGradeDeg(setupStore.setup.road)
        bankDeg = roadBankDeg(setupStore.setup.road)
      }
    })
  })

  function editParts() {
    location.hash = buildHref(setupStore.selectedVid)
  }

  async function onPathChange(e: Event) {
    const v = (e.target as HTMLSelectElement).value
    pathEditActive = false
    await setPathPreset(v)
  }

  async function togglePathEdit() {
    if (pathEditActive) {
      pathEditActive = false
      return
    }
    const pts =
      setupStore.pathPts.length > 1
        ? downsamplePathPts(setupStore.pathPts)
        : ([
            [0, 0],
            [30, 0],
          ] as [number, number][])
    pathEditActive = true
    await commitCustomPath(pts)
  }

  function onPathPtsChange(pts: [number, number][]) {
    void commitCustomPath(pts)
  }

  async function onTemplate(id: string) {
    pathEditActive = false
    const ok = await applyScenarioTemplate(id)
    if (!ok) return
    activeTemplate = id
    saveName = ''
    if (setupStore.setup) {
      gradeDeg = roadGradeDeg(setupStore.setup.road)
      bankDeg = roadBankDeg(setupStore.setup.road)
    }
  }

  async function onScenarioLoad(e: Event) {
    const name = (e.target as HTMLSelectElement).value
    if (name) await loadScenarioByName(name)
  }

  async function onSave() {
    const name = saveName.trim() || setupStore.loadedScenario || 'draft'
    const names = setupStore.setup?.scenarios ?? setupStore.setup?.scenes ?? []
    const exists = names.includes(name)
    const overwrite = exists && confirm(`Overwrite scenario "${name}"?`)
    if (exists && !overwrite) return
    await saveScenarioDraft(name, overwrite)
    saveName = name
  }
</script>

{#if setupStore.error}
  <div class="err">{setupStore.error}</div>
{/if}

{#if setupStore.setup}
  {@const s = setupStore.setup}
  <div class="layout">
    <section class="map">
      <div class="map-toolbar">
        <label>μ
          <HelpLink topic="road" />
          <input
            type="number"
            value={s.road.mu}
            step="0.05"
            min="0"
            oninput={(e) => setMu(+(e.target as HTMLInputElement).value)}
          />
        </label>
        <label>grade°
          <input
            type="number"
            bind:value={gradeDeg}
            step="0.5"
            onchange={() => setGradeDeg(gradeDeg)}
          />
        </label>
        <label>bank°
          <input
            type="number"
            bind:value={bankDeg}
            step="0.5"
            onchange={() => setBankDeg(bankDeg)}
          />
        </label>
        <label>v
          <input
            type="number"
            value={s.v_target}
            step="0.5"
            oninput={(e) => setVTarget(+(e.target as HTMLInputElement).value)}
          />
        </label>
        <button
          type="button"
          class="edit-path"
          class:on={pathEditActive}
          onclick={togglePathEdit}
        >
          {pathEditActive ? 'Done path' : 'Edit path'}
        </button>
      </div>
      {#if pathEditActive}
        <div class="edit-hint">Drag ● waypoints · click to add · Del remove · Esc done</div>
      {/if}
      <div class="map-body">
        <MapCanvas
          pathPts={setupStore.pathPts}
          fleet={s.fleet}
          selectedVid={setupStore.selectedVid}
          pathEdit={pathEditActive}
          onSelect={(vid) => { setupStore.selectedVid = vid }}
          onMove={(vid, x, y) => moveFleetAgent(vid, x, y)}
          onPathChange={onPathPtsChange}
          onPathEditEnd={() => { pathEditActive = false }}
        />
      </div>
    </section>
    <aside class="inspector">
      <h2>Scenario</h2>
      <div class="grp">New from template</div>
      <div class="templates">
        {#each SCENARIO_TEMPLATES as t}
          <button
            type="button"
            class="tpl"
            class:on={activeTemplate === t.id}
            title={t.hint}
            disabled={setupStore.busy}
            onclick={() => onTemplate(t.id)}
          >
            <b>{t.label}</b>
            <span>{t.hint}</span>
          </button>
        {/each}
      </div>

      <select class="wide" onchange={onScenarioLoad} value={setupStore.loadedScenario}>
        <option value="">Load scenario…</option>
        {#each s.scenarios ?? s.scenes ?? [] as name}
          <option value={name}>{name}</option>
        {/each}
      </select>

      <div class="grp">Road</div>
      <div class="row"><span>μ</span><b>{s.road.mu.toFixed(2)}</b></div>
      <div class="row"><span>grade</span><b>{gradeDeg.toFixed(1)}°</b></div>

      <div class="grp">Fleet agents</div>
      {#each s.fleet as agent (agent.id)}
        <button
          type="button"
          class="agent"
          class:sel={agent.id === setupStore.selectedVid}
          onclick={() => { setupStore.selectedVid = agent.id }}
        >
          <div><b>Vehicle {agent.id}</b> · {agent.blueprint}</div>
          <div class="sub">x0={agent.x0?.toFixed(1) ?? '0'} y0={agent.y0?.toFixed(1) ?? '0'}</div>
        </button>
      {/each}
      <button type="button" class="add" onclick={() => addVehicle()} disabled={setupStore.busy}>
        + add vehicle
      </button>
      {#if setupStore.selectedVid != null}
        <button type="button" class="link" onclick={editParts}>Edit parts →</button>
      {/if}

      <div class="grp">Path <HelpLink topic="run" /></div>
      <select class="wide" value={s.path_preset} onchange={onPathChange}>
        <option value="figure8">Figure-8</option>
        <option value="straight">Straight</option>
        <option value="custom">Custom waypoints</option>
      </select>
      <div class="sub">{(setupStore.pathPts.length || s.path_pts?.length || 0)} points · {s.path_preset}</div>
      {#if wpTable.length > 0}
        <table class="wp">
          <thead><tr><th>#</th><th>x [m]</th><th>y [m]</th></tr></thead>
          <tbody>
            {#each wpTable as [x, y], i}
              <tr><td>{i + 1}</td><td>{x.toFixed(1)}</td><td>{y.toFixed(1)}</td></tr>
            {/each}
          </tbody>
        </table>
        {#if setupStore.pathPts.length > wpTable.length}
          <div class="sub">…{setupStore.pathPts.length} total · use Edit path for full edit</div>
        {/if}
      {/if}

      <div class="grp">Save</div>
      <input
        class="wide"
        placeholder="scenario name"
        bind:value={saveName}
        oninput={() => markDirty()}
      />

      <div class="actions">
        <button type="button" onclick={() => syncDraft()} disabled={setupStore.busy}>
          Sync draft
        </button>
        <button type="button" class="primary" onclick={onSave} disabled={setupStore.busy}>
          Save scenario
        </button>
      </div>
    </aside>
  </div>
{:else if setupStore.busy}
  <p class="loading">Loading setup…</p>
{:else}
  <p class="loading">No API — start <code>python3 gui/server.py --port 8095</code></p>
{/if}

<style>
  .layout {
    display: grid;
    grid-template-columns: 1fr min(360px, 32%);
    height: 100%;
  }
  .map {
    display: flex;
    flex-direction: column;
    min-width: 0;
    border-right: 1px solid var(--border);
  }
  .map-toolbar {
    display: flex;
    flex-wrap: wrap;
    gap: 12px;
    padding: 8px 12px;
    font-size: var(--fs-sm);
    background: var(--surface);
    border-bottom: 1px solid var(--border);
  }
  .map-toolbar label {
    display: flex;
    align-items: center;
    gap: 4px;
  }
  .map-toolbar input {
    width: 56px;
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 2px 4px;
  }
  .edit-path {
    margin-left: auto;
    border: 1px solid var(--border);
    background: var(--surface);
    border-radius: var(--radius);
    padding: 4px 10px;
    font-size: var(--fs-xs);
    font-weight: 600;
  }
  .edit-path.on {
    border-color: var(--acc);
    color: var(--acc);
    background: #e8f7fd;
  }
  .edit-hint {
    padding: 4px 12px;
    font-size: var(--fs-xs);
    color: var(--dim);
    background: #fff8e8;
    border-bottom: 1px solid var(--border);
  }
  .map-body {
    flex: 1;
    min-height: 0;
  }
  .inspector {
    padding: 12px;
    overflow: auto;
    background: var(--surface);
  }
  h2 { margin: 0 0 8px; font-size: var(--fs-md); }
  .grp {
    font-size: var(--fs-xs);
    text-transform: uppercase;
    letter-spacing: 0.04em;
    color: var(--dim);
    margin: 12px 0 6px;
  }
  .templates {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
    margin-bottom: 8px;
  }
  .tpl {
    display: flex;
    flex-direction: column;
    align-items: flex-start;
    text-align: left;
    padding: 8px;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    font-size: var(--fs-xs);
    cursor: pointer;
  }
  .tpl b { font-size: var(--fs-sm); color: var(--ink); }
  .tpl span { color: var(--dim); margin-top: 2px; line-height: 1.3; }
  .tpl.on {
    border-color: var(--acc);
    box-shadow: inset 0 0 0 1px var(--acc);
  }
  .tpl:disabled { opacity: 0.5; cursor: default; }
  .row {
    display: flex;
    justify-content: space-between;
    font-size: var(--fs-sm);
  }
  .wide {
    width: 100%;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 6px;
    font-size: var(--fs-sm);
  }
  .agent {
    display: block;
    width: 100%;
    text-align: left;
    padding: 8px;
    margin-bottom: 4px;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    font-size: var(--fs-sm);
    cursor: pointer;
  }
  .agent.sel {
    border-left: 3px solid var(--acc);
    background: #f4f7fa;
  }
  .sub { font-size: var(--fs-xs); color: var(--dim); margin-top: 2px; }
  .wp {
    width: 100%;
    border-collapse: collapse;
    font-size: var(--fs-xs);
    margin-top: 6px;
    font-variant-numeric: tabular-nums;
  }
  .wp th, .wp td {
    border: 1px solid var(--border);
    padding: 2px 5px;
    text-align: right;
  }
  .wp th:first-child, .wp td:first-child { text-align: center; color: var(--dim); }
  .wp th:nth-child(2), .wp th:nth-child(3) { text-align: right; }
  .add, .link {
    width: 100%;
    margin-top: 6px;
    border: none;
    background: none;
    color: var(--acc);
    font-size: var(--fs-xs);
    cursor: pointer;
    text-align: left;
    padding: 4px 0;
  }
  .add { color: var(--ink); border: 1px dashed var(--border); border-radius: var(--radius); padding: 6px; text-align: center; }
  .actions {
    display: flex;
    gap: 8px;
    margin-top: 16px;
  }
  .actions button {
    flex: 1;
    padding: 6px;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    font-size: var(--fs-sm);
  }
  .actions .primary {
    background: var(--acc);
    border-color: var(--acc);
    color: #001018;
    font-weight: 600;
  }
  .err {
    background: #fef2f2;
    color: var(--err);
    padding: 6px 12px;
    font-size: var(--fs-sm);
    border-bottom: 1px solid #fecaca;
  }
  .loading {
    padding: 24px;
    color: var(--dim);
    text-align: center;
  }
</style>
