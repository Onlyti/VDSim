<script lang="ts">
  import { onMount } from 'svelte'
  import type { FleetAgent } from '../lib/api/types'
  import {
    drawMapScene,
    fitViewport,
    hitFleetAgent,
    hitPathWaypoint,
    screenToWorld,
    type Viewport,
  } from '../lib/map/viewport'

  interface Props {
    pathPts: [number, number][]
    fleet: FleetAgent[]
    selectedVid: number
    pathEdit?: boolean
    onSelect?: (vid: number) => void
    onMove?: (vid: number, x: number, y: number) => void
    onPathChange?: (pts: [number, number][]) => void
    onPathEditEnd?: () => void
  }

  let {
    pathPts,
    fleet,
    selectedVid,
    pathEdit = false,
    onSelect,
    onMove,
    onPathChange,
    onPathEditEnd,
  }: Props = $props()

  let canvas: HTMLCanvasElement | undefined = $state()
  let vp = $state<Viewport | null>(null)
  let dragVid: number | null = $state(null)
  let dragPos: [number, number] | null = $state(null)
  let dragWp = $state(-1)
  let selectedWp = $state(-1)
  let draftPts = $state<[number, number][]>([])

  const drawPath = $derived(pathEdit && draftPts.length ? draftPts : pathPts)

  function fleetDraw(): FleetAgent[] {
    if (dragVid == null || !dragPos) return fleet
    return fleet.map((f) =>
      f.id === dragVid ? { ...f, x0: dragPos![0], y0: dragPos![1] } : f,
    )
  }

  function paint() {
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const dpr = window.devicePixelRatio || 1
    const w = canvas.clientWidth
    const h = canvas.clientHeight
    if (w < 8 || h < 8) return
    canvas.width = w * dpr
    canvas.height = h * dpr
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

    const drawn = fleetDraw()
    const points: [number, number][] = [...drawPath]
    for (const f of drawn) points.push([f.x0 ?? 0, f.y0 ?? 0])
    vp = fitViewport(points, w, h)
    drawMapScene(ctx, vp, drawPath, drawn, selectedVid, pathEdit ? { selectedWp } : undefined)
  }

  function canvasCoords(e: PointerEvent): [number, number] {
    const r = canvas!.getBoundingClientRect()
    return [e.clientX - r.left, e.clientY - r.top]
  }

  function commitPath(pts: [number, number][]) {
    draftPts = pts.map((p) => [p[0], p[1]])
    onPathChange?.(draftPts)
  }

  function onPointerDown(e: PointerEvent) {
    if (!vp || !canvas) return
    const [sx, sy] = canvasCoords(e)

    if (pathEdit) {
      const wi = hitPathWaypoint(vp, drawPath, sx, sy, 12)
      if (wi >= 0) {
        dragWp = wi
        selectedWp = wi
        canvas.setPointerCapture(e.pointerId)
        paint()
        return
      }
      const [x, y] = screenToWorld(vp, sx, sy)
      const next: [number, number][] = [...drawPath, [x, y]]
      selectedWp = next.length - 1
      commitPath(next)
      canvas.setPointerCapture(e.pointerId)
      paint()
      return
    }

    const hit = hitFleetAgent(vp, fleet, sx, sy)
    if (!hit) return
    dragVid = hit.id
    dragPos = [hit.x0 ?? 0, hit.y0 ?? 0]
    onSelect?.(hit.id)
    canvas.setPointerCapture(e.pointerId)
    paint()
  }

  function onPointerMove(e: PointerEvent) {
    if (!vp) return
    const [sx, sy] = canvasCoords(e)

    if (pathEdit && dragWp >= 0) {
      const [x, y] = screenToWorld(vp, sx, sy)
      const next = drawPath.map((p, i) => (i === dragWp ? [x, y] as [number, number] : p))
      draftPts = next
      paint()
      return
    }

    if (dragVid == null) return
    dragPos = screenToWorld(vp, sx, sy)
    paint()
  }

  function onPointerUp(e: PointerEvent) {
    if (pathEdit && dragWp >= 0) {
      dragWp = -1
      canvas?.releasePointerCapture(e.pointerId)
      commitPath(draftPts.length ? draftPts : drawPath)
      paint()
      return
    }

    if (dragVid == null || !dragPos) return
    const vid = dragVid
    const [x, y] = dragPos
    dragVid = null
    dragPos = null
    canvas?.releasePointerCapture(e.pointerId)
    onMove?.(vid, x, y)
    paint()
  }

  function onKeyDown(e: KeyboardEvent) {
    if (!pathEdit) return
    if (e.key === 'Escape') {
      onPathEditEnd?.()
      e.preventDefault()
      return
    }
    if ((e.key === 'Delete' || e.key === 'Backspace') && drawPath.length > 2) {
      const idx = selectedWp >= 0 ? selectedWp : drawPath.length - 1
      const next = drawPath.filter((_, i) => i !== idx)
      selectedWp = Math.min(selectedWp, next.length - 1)
      commitPath(next)
      e.preventDefault()
    }
  }

  $effect(() => {
    pathPts
    fleet
    selectedVid
    pathEdit
    paint()
  })

  $effect(() => {
    if (pathEdit) draftPts = pathPts.map((p) => [p[0], p[1]])
    else {
      dragWp = -1
      selectedWp = -1
    }
  })

  onMount(() => {
    const ro = new ResizeObserver(() => paint())
    if (canvas) ro.observe(canvas)
    window.addEventListener('keydown', onKeyDown)
    paint()
    return () => {
      ro.disconnect()
      window.removeEventListener('keydown', onKeyDown)
    }
  })
</script>

<canvas
  bind:this={canvas}
  class="map-canvas"
  class:editing={pathEdit}
  aria-label={pathEdit ? 'Edit path waypoints' : 'Scenario map — drag vehicles to set spawn'}
  onpointerdown={onPointerDown}
  onpointermove={onPointerMove}
  onpointerup={onPointerUp}
  onpointercancel={onPointerUp}
></canvas>

<style>
  .map-canvas {
    width: 100%;
    height: 100%;
    display: block;
    cursor: crosshair;
    touch-action: none;
  }
  .map-canvas.editing {
    cursor: cell;
  }
</style>
