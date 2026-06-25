<script lang="ts">
  import { onMount } from 'svelte'
  import { RunViewport, type CamMode } from '../lib/viewport/RunViewport'
  import type { SimStreamState } from '../lib/viewport/types'

  interface Props {
    onState?: (s: SimStreamState) => void
    camMode?: CamMode
    showForces?: boolean
  }

  let { onState, camMode = 'chase', showForces = true }: Props = $props()

  let host: HTMLDivElement | undefined = $state()
  let vp: RunViewport | null = null
  let es: EventSource | null = null

  $effect(() => {
    if (vp && camMode) vp.setCamMode(camMode)
  })

  $effect(() => {
    if (vp) vp.setShowForces(showForces)
  })

  onMount(() => {
    if (!host) return
    vp = new RunViewport(host)
    if (onState) vp.setOnState(onState)
    vp.setCamMode(camMode)

    es = new EventSource('/api/stream')
    es.onmessage = (e) => {
      try {
        const s = JSON.parse(e.data) as SimStreamState
        void vp?.applyState(s)
      } catch {
        /* ignore */
      }
    }

    return () => {
      es?.close()
      vp?.dispose()
      vp = null
    }
  })
</script>

<div class="host" bind:this={host}></div>

<style>
  .host {
    width: 100%;
    height: 100%;
    min-height: 200px;
    background: #1a1f28;
  }
</style>
