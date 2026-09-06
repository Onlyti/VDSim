<script lang="ts">
  import { onMount } from 'svelte'
  import {
    COMPARE_MANEUVERS,
    cmpDeltaPct,
    cmpFmt,
    downloadCsv,
    runCompare,
    rowsToCsv,
    type CompareManeuverId,
    type CompareResult,
    vehicleLabel,
  } from '../../lib/api/compare'
  import { fetchAssembly } from '../../lib/api/catalog'
  import { yawRateOverlaySvg } from '../../lib/charts/yawRateChart'
  import { modeHref } from '../../lib/router'
  import { refreshSetup, setupStore } from '../../lib/setupStore.svelte'

  interface Pick {
    id: string
    label: string
    level: string
    fromFleet: boolean
  }

  let picks = $state<Pick[]>([])
  let selected = $state<Set<string>>(new Set())
  let maneuvers = $state<CompareManeuverId[]>(['step_steer', 'skidpad', 'dlc'])
  let level = $state('L2')
  let tire = $state('')
  let busy = $state(false)
  let err = $state<string | null>(null)
  let result = $state<CompareResult | null>(null)
  let chartSvg = $state('')

  const fleet = $derived(setupStore.setup?.fleet ?? [])

  function toggleManeuver(id: CompareManeuverId) {
    if (maneuvers.includes(id)) {
      maneuvers = maneuvers.filter((m) => m !== id)
    } else {
      maneuvers = [...maneuvers, id]
    }
  }

  function toggleVehicle(id: string) {
    const next = new Set(selected)
    if (next.has(id)) next.delete(id)
    else next.add(id)
    selected = next
  }

  async function loadPicks() {
    const map = new Map<string, Pick>()
    for (const agent of fleet) {
      const id = agent.blueprint
      if (!id || map.has(id)) continue
      map.set(id, {
        id,
        label: vehicleLabel(id),
        level: agent.level ?? 'L2',
        fromFleet: true,
      })
    }
    try {
      const asm = await fetchAssembly(fleet[0]?.id ?? 0)
      for (const bp of asm.blueprints) {
        if (!map.has(bp.id)) {
          map.set(bp.id, {
            id: bp.id,
            label: bp.label,
            level: bp.level ?? 'L2',
            fromFleet: false,
          })
        }
      }
    } catch {
      /* catalog list optional */
    }
    picks = [...map.values()]
    if (!selected.size) {
      const fleetIds = fleet.map((a) => a.blueprint).filter(Boolean)
      const uniq = [...new Set(fleetIds)]
      selected = new Set(uniq.slice(0, Math.min(3, uniq.length)))
    }
  }

  async function onRun() {
    const vehicles = [...selected]
    if (vehicles.length < 2) {
      err = 'Pick at least 2 vehicles.'
      return
    }
    if (!maneuvers.length) {
      err = 'Pick at least one maneuver.'
      return
    }
    busy = true
    err = null
    result = null
    chartSvg = ''
    try {
      const r = await runCompare({
        vehicles,
        maneuvers,
        level,
        tire: tire.trim() || null,
      })
      if (!r.ok) throw new Error(r.error || 'compare failed')
      result = r
      if (r.traces && Object.keys(r.traces).length) {
        chartSvg = yawRateOverlaySvg(r.traces)
      }
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      busy = false
    }
  }

  function onExport() {
    if (!result) return
    downloadCsv('vdsim_compare.csv', rowsToCsv(result.rows, result.columns))
  }

  onMount(() => {
    void refreshSetup().then(() => loadPicks())
  })

  $effect(() => {
    fleet
    void loadPicks()
  })
</script>

<div class="analyze">
  <header class="bar">
    <span class="title">ISO compare</span>
    <div class="maneuvers">
      {#each COMPARE_MANEUVERS as m (m.id)}
        <label class="chk">
          <input
            type="checkbox"
            checked={maneuvers.includes(m.id)}
            onchange={() => toggleManeuver(m.id)}
          />
          {m.label}
        </label>
      {/each}
    </div>
    <label class="fld">
      Level
      <select bind:value={level}>
        {#each ['K', 'L1', 'L2', 'L3', 'L4', 'L5'] as lv}
          <option value={lv}>{lv}</option>
        {/each}
      </select>
    </label>
    <label class="fld">
      Tire (optional)
      <input type="text" bind:value={tire} placeholder="common tire stem" />
    </label>
    <button type="button" class="run" disabled={busy} onclick={onRun}>
      {busy ? 'Running…' : 'Run compare'}
    </button>
    {#if result}
      <button type="button" class="export" onclick={onExport}>Export CSV</button>
    {/if}
    <a class="back" href={modeHref('scenario')}>← Scenario</a>
  </header>

  {#if err}
    <div class="err">{err}</div>
  {/if}

  <section class="picks">
    <span class="lbl">Vehicles</span>
    {#if !picks.length}
      <span class="sub">Add fleet agents in Scenario first.</span>
    {:else}
      {#each picks as p (p.id)}
        <label class="pick" class:fleet={p.fromFleet}>
          <input
            type="checkbox"
            checked={selected.has(p.id)}
            onchange={() => toggleVehicle(p.id)}
          />
          <span class="name">{p.label}</span>
          <span class="meta">{vehicleLabel(p.id)} · {p.level}{p.fromFleet ? ' · fleet' : ''}</span>
        </label>
      {/each}
    {/if}
  </section>

  <div class="body">
    <section class="chart">
      {#if chartSvg}
        <h3>Step-steer yaw-rate overlay</h3>
        <div class="svg-wrap">{@html chartSvg}</div>
      {:else if result && !Object.keys(result.traces ?? {}).length}
        <p class="hint">Include <b>Step steer</b> maneuver for yaw-rate chart.</p>
      {:else if busy}
        <p class="hint">Running ISO maneuvers…</p>
      {:else}
        <p class="hint">Select ≥2 vehicles and run compare.</p>
      {/if}
    </section>

    <section class="table-wrap">
      {#if result?.rows?.length}
        <h3>Metrics · Δ% vs first column (baseline)</h3>
        <div class="scroll">
          <table class="cmp">
            <thead>
              <tr>
                <th>metric</th>
                {#each result.rows as row, i}
                  <th>
                    {vehicleLabel(row.vehicle)}
                    {#if i === 0}<span class="base">base</span>{/if}
                  </th>
                {/each}
              </tr>
            </thead>
            <tbody>
              {#each result.columns as col}
                {@const base = result.rows[0]?.[col]}
                <tr>
                  <th class="metric">{col}</th>
                  {#each result.rows as row, i}
                    {@const v = row[col]}
                    <td>
                      {#if typeof v === 'number'}
                        <span class="num">{cmpFmt(v)}</span>
                        {#if i > 0}
                          {@const d = cmpDeltaPct(base, v)}
                          {#if d}
                            <span class="delta" class:neg={d.neg}>{d.text}</span>
                          {/if}
                        {/if}
                      {:else}
                        <span class="qual">{v ?? ''}</span>
                      {/if}
                    </td>
                  {/each}
                </tr>
              {/each}
            </tbody>
          </table>
        </div>
      {:else}
        <p class="hint">Δ% table appears after compare run.</p>
      {/if}
    </section>
  </div>
</div>

<style>
  .analyze {
    display: flex;
    flex-direction: column;
    height: 100%;
    min-height: 0;
  }
  .bar {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 10px;
    padding: 8px 12px;
    border-bottom: 1px solid var(--border);
    background: var(--surface);
    flex-shrink: 0;
  }
  .title { font-weight: 600; font-size: var(--fs-sm); }
  .maneuvers { display: flex; gap: 10px; flex-wrap: wrap; }
  .chk {
    font-size: var(--fs-xs);
    display: inline-flex;
    align-items: center;
    gap: 4px;
  }
  .fld {
    font-size: var(--fs-xs);
    color: var(--dim);
    display: inline-flex;
    align-items: center;
    gap: 4px;
  }
  .fld select, .fld input {
    padding: 3px 6px;
    border: 1px solid var(--border);
    border-radius: 4px;
    font-size: var(--fs-xs);
  }
  .run {
    background: var(--acc);
    border: none;
    color: #001018;
    padding: 6px 12px;
    border-radius: var(--radius);
    font-weight: 600;
    font-size: var(--fs-sm);
  }
  .run:disabled { opacity: 0.5; }
  .export {
    border: 1px solid var(--border);
    background: var(--surface);
    padding: 5px 10px;
    border-radius: var(--radius);
    font-size: var(--fs-xs);
  }
  .back {
    margin-left: auto;
    font-size: var(--fs-xs);
    color: var(--acc);
    text-decoration: none;
  }
  .err {
    margin: 8px 12px 0;
    padding: 8px;
    background: #fef2f2;
    color: var(--err);
    font-size: var(--fs-sm);
    border-radius: var(--radius);
  }
  .picks {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    align-items: center;
    padding: 8px 12px;
    border-bottom: 1px solid var(--border);
    background: var(--bg);
  }
  .lbl { font-size: var(--fs-xs); color: var(--dim); font-weight: 600; }
  .sub { font-size: var(--fs-xs); color: var(--dim); }
  .pick {
    display: inline-flex;
    flex-direction: column;
    padding: 4px 8px;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    font-size: var(--fs-xs);
    cursor: pointer;
  }
  .pick.fleet { border-color: var(--acc); }
  .name { font-weight: 600; }
  .meta { color: var(--dim); font-size: 9px; }
  .body {
    flex: 1;
    display: grid;
    grid-template-rows: minmax(180px, 42%) 1fr;
    gap: 8px;
    padding: 8px 12px 12px;
    min-height: 0;
  }
  .chart, .table-wrap {
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    padding: 10px;
    min-height: 0;
    overflow: auto;
  }
  h3 {
    margin: 0 0 8px;
    font-size: var(--fs-sm);
    color: var(--dim);
    font-weight: 600;
  }
  .svg-wrap { max-width: 720px; }
  .hint {
    color: var(--dim);
    font-size: var(--fs-sm);
    margin: 0;
    text-align: center;
    padding: 24px;
  }
  .scroll { overflow: auto; }
  .cmp {
    width: 100%;
    border-collapse: collapse;
    font-size: var(--fs-sm);
  }
  .cmp th, .cmp td {
    border-bottom: 1px solid var(--border);
    padding: 4px 8px;
    text-align: right;
  }
  .cmp tr th:first-child, .cmp .metric {
    text-align: left;
    color: var(--dim);
    font-weight: 500;
  }
  .cmp thead tr th {
    color: var(--ink);
    font-weight: 600;
  }
  .cmp .num { font-variant-numeric: tabular-nums; }
  .cmp .delta { font-size: 9px; color: #2a8a4a; margin-left: 4px; }
  .cmp .delta.neg { color: #c55; }
  .cmp .qual { color: var(--dim); font-style: italic; }
  .base {
    font-size: 8px;
    color: var(--dim);
    font-weight: 400;
    text-transform: uppercase;
    margin-left: 4px;
  }
</style>
