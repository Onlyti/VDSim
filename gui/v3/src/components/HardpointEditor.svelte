<script lang="ts">
  import { onMount } from 'svelte'
  import {
    fetchSuspensionKin,
    previewSuspensionKin,
    saveSuspensionKin,
    type KinSchematic,
    type KcPlot,
    type SuspensionKin,
    type Vec3,
  } from '../lib/api/suspension'
  import { kcPlotSvg } from '../lib/charts/kcPlots'
  import { draggablePaths, setKinPath, shortPath } from '../lib/kin/kinDoc'
  import {
    collectYzPts,
    fitYzViewport,
    vpScreenToYz,
    vpX,
    vpY,
    type YzViewport,
  } from '../lib/kin/kinViewport'
  import { setFleetPart } from '../lib/setupStore.svelte'

  interface Props {
    vid: number
    slot: string
    partId: string
    sideLabel?: string
    onInstalled?: () => void
  }

  let { vid, slot, partId, sideLabel = 'Suspension', onInstalled }: Props = $props()

  let kin = $state<SuspensionKin | null>(null)
  let doc = $state<Record<string, unknown> | null>(null)
  let geometry = $state<Record<string, Vec3>>({})
  let schematic = $state<KinSchematic>({ links: [], points: {} })
  let plots = $state<KcPlot[]>([])
  let selected = $state('')
  let busy = $state(false)
  let previewBusy = $state(false)
  let err = $state<string | null>(null)
  let status = $state('')
  let dirty = $state(false)
  let canvasW = $state(360)
  let canvasH = $state(260)
  let dragPath = $state('')
  let dragPtr = $state<number | null>(null)
  let previewTimer = 0

  const paths = $derived(draggablePaths(geometry))
  const stroke = $derived(slot === 'rear_chassis' ? '#1d8a6e' : '#e85d04')
  const vp = $derived.by(() => {
    const pts = collectYzPts(schematic.links, schematic.points, geometry)
    return fitYzViewport(pts, canvasW, canvasH)
  })
  const selPt = $derived(selected ? geometry[selected] : null)
  const showPlots = $derived(plots.slice(0, 2))

  function nearly(a: Vec3, b: Vec3, eps = 1e-4): boolean {
    return Math.abs(a[0] - b[0]) < eps && Math.abs(a[1] - b[1]) < eps && Math.abs(a[2] - b[2]) < eps
  }

  function patchLinks(oldPt: Vec3, newPt: Vec3) {
    schematic = {
      ...schematic,
      links: schematic.links.map(([a, b]) => [
        nearly(a, oldPt) ? [...newPt] : a,
        nearly(b, oldPt) ? [...newPt] : b,
      ] as [Vec3, Vec3]),
    }
  }

  function schedulePreview() {
    dirty = true
    clearTimeout(previewTimer)
    previewTimer = window.setTimeout(() => void runPreview(), 220)
  }

  async function runPreview() {
    if (!doc) return
    previewBusy = true
    try {
      const r = await previewSuspensionKin(doc)
      geometry = r.geometry
      schematic = r.schematic
      plots = r.plots
      dirty = false
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      previewBusy = false
    }
  }

  function applyPoint(path: string, value: Vec3) {
    if (!doc) return
    const old = geometry[path]
    if (!old) return
    setKinPath(doc, path, value)
    geometry = { ...geometry, [path]: [...value] }
    patchLinks(old, value)
    schedulePreview()
  }

  function onCoord(axis: 0 | 1 | 2, raw: string) {
    if (!selected || !selPt) return
    const v = Number(raw)
    if (!Number.isFinite(v)) return
    const next: Vec3 = [...selPt]
    next[axis] = v
    applyPoint(selected, next)
  }

  function onPick(path: string) {
    selected = path
  }

  function onPtrDown(path: string, e: PointerEvent) {
    selected = path
    dragPath = path
    dragPtr = e.pointerId
    ;(e.currentTarget as Element).setPointerCapture(e.pointerId)
  }

  function onPtrMove(e: PointerEvent) {
    if (!dragPath || dragPtr !== e.pointerId || !vp || !doc) return
    const pt = geometry[dragPath]
    if (!pt) return
    const rect = (e.currentTarget as SVGSVGElement).getBoundingClientRect()
    const sx = ((e.clientX - rect.left) / rect.width) * canvasW
    const sy = ((e.clientY - rect.top) / rect.height) * canvasH
    const [y, z] = vpScreenToYz(vp, sx, sy)
    applyPoint(dragPath, [pt[0], y, z])
  }

  function onPtrUp(e: PointerEvent) {
    if (dragPtr === e.pointerId) {
      dragPath = ''
      dragPtr = null
    }
  }

  async function load() {
    busy = true
    err = null
    status = ''
    kin = null
    doc = null
    selected = ''
    try {
      const r = await fetchSuspensionKin(partId)
      kin = r
      doc = structuredClone(r.doc)
      geometry = { ...r.geometry }
      schematic = r.schematic
      plots = r.plots ?? []
      if (!plots.length) await runPreview()
      const first = draggablePaths(r.geometry)[0]
      if (first) selected = first
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      busy = false
    }
  }

  async function onSave() {
    if (!doc || !kin) return
    let stem = kin.name.replace(/[^a-z0-9_]+/gi, '_')
    let label = `${sideLabel} edit`
    let baseId: string | undefined
    if (kin.readonly) {
      const s = prompt('Custom part stem (a-z, digits, _)', `${stem}_edit`)
      if (!s) return
      stem = s
      label = prompt('Display label', label) || label
    } else {
      baseId = kin.part_id
      stem = kin.part_id.split('.').pop() ?? stem
      label = prompt('Display label', kin.part_id.split('.').pop() ?? label) || label
    }
    busy = true
    err = null
    try {
      const r = await saveSuspensionKin({
        doc,
        stem,
        label,
        base_part_id: baseId,
      })
      status = `Saved ${r.part_id}`
      await setFleetPart(vid, slot, r.part_id)
      onInstalled?.()
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      busy = false
    }
  }

  $effect(() => {
    partId
    slot
    void load()
  })

  onMount(() => () => clearTimeout(previewTimer))
</script>

<div class="hp">
  <header class="hp-bar">
    <span class="hp-title">{sideLabel} hardpoints</span>
    {#if kin}
      <span class="hp-meta">{kin.type} · {kin.name}</span>
      {#if kin.readonly}<span class="hp-ro">read-only stock</span>{/if}
      {#if previewBusy || dirty}<span class="hp-busy">updating…</span>{/if}
    {/if}
    <button type="button" class="hp-save" disabled={busy || !doc} onclick={onSave}>Save & install</button>
  </header>

  {#if err}
    <div class="hp-err">{err}</div>
  {/if}
  {#if status}
    <div class="hp-ok">{status}</div>
  {/if}

  {#if busy && !kin}
    <p class="hp-load">Loading kinematics…</p>
  {:else if kin && doc}
    <div class="hp-body">
      <aside class="hp-list">
        <div class="hp-list-head">Pick point</div>
        {#each paths as path (path)}
          <button
            type="button"
            class="hp-pt"
            class:on={path === selected}
            onclick={() => onPick(path)}
          >
            {shortPath(path)}
          </button>
        {/each}
      </aside>

      <div class="hp-canvas" bind:clientWidth={canvasW} bind:clientHeight={canvasH}>
        {#if vp}
          <svg
            class="hp-svg"
            role="img"
            aria-label="Suspension hardpoint side view"
            viewBox="0 0 {canvasW} {canvasH}"
            onpointermove={onPtrMove}
            onpointerup={onPtrUp}
            onpointercancel={onPtrUp}
          >
            <line
              x1={vp.margin}
              y1={vpY(vp, 0)}
              x2={canvasW - vp.margin}
              y2={vpY(vp, 0)}
              stroke="#d8dee8"
              stroke-width="1"
            />
            {#each schematic.links as [a, b], i (i)}
              <line
                x1={vpX(vp, a[1])}
                y1={vpY(vp, a[2])}
                x2={vpX(vp, b[1])}
                y2={vpY(vp, b[2])}
                stroke={stroke}
                stroke-width="2"
                stroke-linecap="round"
              />
            {/each}
            {#each paths as path (path)}
              {@const p = geometry[path]}
              {#if p}
                <circle
                  cx={vpX(vp, p[1])}
                  cy={vpY(vp, p[2])}
                  r={path === selected ? 6 : 4}
                  fill={path === selected ? '#01A0E9' : stroke}
                  stroke="#fff"
                  stroke-width="1.5"
                  style="cursor: grab"
                  onpointerdown={(e) => onPtrDown(path, e)}
                />
              {/if}
            {/each}
          </svg>
        {/if}
        <div class="hp-hint">Side view (y–z) · drag ● or edit coordinates</div>
      </div>

      <aside class="hp-side">
        {#if selected && selPt}
          <div class="hp-coords">
            <div class="hp-coords-head">{shortPath(selected)}</div>
            <label>x <input type="number" step="0.001" value={selPt[0]} oninput={(e) => onCoord(0, (e.target as HTMLInputElement).value)} /></label>
            <label>y <input type="number" step="0.001" value={selPt[1]} oninput={(e) => onCoord(1, (e.target as HTMLInputElement).value)} /></label>
            <label>z <input type="number" step="0.001" value={selPt[2]} oninput={(e) => onCoord(2, (e.target as HTMLInputElement).value)} /></label>
          </div>
        {/if}
        <div class="hp-kc">
          {#each showPlots as plot (plot.title)}
            <div class="hp-plot">{@html kcPlotSvg(plot)}</div>
          {/each}
        </div>
      </aside>
    </div>
  {/if}
</div>

<style>
  .hp {
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    overflow: hidden;
  }
  .hp-bar {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 6px 10px;
    border-bottom: 1px solid var(--border);
    font-size: var(--fs-xs);
    flex-wrap: wrap;
  }
  .hp-title { font-weight: 600; }
  .hp-meta { color: var(--dim); }
  .hp-ro { color: var(--warn); font-style: italic; }
  .hp-busy { color: var(--acc); }
  .hp-save {
    margin-left: auto;
    padding: 4px 10px;
    border: 1px solid var(--acc);
    border-radius: 4px;
    background: #e8f7fd;
    color: var(--acc);
    font-size: var(--fs-xs);
  }
  .hp-save:disabled { opacity: 0.5; }
  .hp-err {
    margin: 6px 10px;
    padding: 6px 8px;
    background: #fef2f2;
    color: var(--err);
    font-size: var(--fs-xs);
    border-radius: 4px;
  }
  .hp-ok {
    margin: 6px 10px;
    padding: 6px 8px;
    background: #f0fdf4;
    color: var(--ok);
    font-size: var(--fs-xs);
    border-radius: 4px;
  }
  .hp-load {
    padding: 12px;
    color: var(--dim);
    font-size: var(--fs-sm);
    margin: 0;
  }
  .hp-body {
    display: grid;
    grid-template-columns: minmax(110px, 16%) 1fr minmax(180px, 28%);
    min-height: 280px;
  }
  .hp-list {
    border-right: 1px solid var(--border);
    padding: 6px;
    overflow: auto;
    max-height: 320px;
  }
  .hp-list-head {
    font-size: 9px;
    text-transform: uppercase;
    color: var(--dim);
    margin-bottom: 4px;
  }
  .hp-pt {
    display: block;
    width: 100%;
    text-align: left;
    padding: 4px 6px;
    margin-bottom: 2px;
    border: none;
    border-radius: 4px;
    background: transparent;
    font-size: var(--fs-xs);
    cursor: pointer;
  }
  .hp-pt.on {
    background: #e8f7fd;
    color: var(--acc);
    font-weight: 600;
  }
  .hp-canvas {
    position: relative;
    min-height: 260px;
    padding: 4px;
  }
  .hp-svg {
    width: 100%;
    height: 100%;
    min-height: 240px;
    touch-action: none;
  }
  .hp-hint {
    position: absolute;
    left: 8px;
    bottom: 4px;
    font-size: 9px;
    color: var(--dim);
    pointer-events: none;
  }
  .hp-side {
    border-left: 1px solid var(--border);
    padding: 8px;
    overflow: auto;
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .hp-coords-head {
    font-size: var(--fs-xs);
    font-weight: 600;
    margin-bottom: 4px;
  }
  .hp-coords label {
    display: grid;
    grid-template-columns: 14px 1fr;
    align-items: center;
    gap: 4px;
    font-size: var(--fs-xs);
    margin-bottom: 4px;
  }
  .hp-coords input {
    width: 100%;
    padding: 2px 4px;
    border: 1px solid var(--border);
    border-radius: 4px;
    font-size: var(--fs-xs);
  }
  .hp-kc { display: flex; flex-direction: column; gap: 6px; }
  .hp-plot {
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 4px;
    padding: 2px;
  }
</style>
