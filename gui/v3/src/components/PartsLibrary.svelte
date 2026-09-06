<script lang="ts">
  import {
    deleteCatalogPart,
    downloadText,
    fetchCatalogParts,
    fetchPartEditor,
    importPartYaml,
    installSlotForPartType,
    PART_LIB_TYPES,
    savePartFields,
    type CatalogPart,
    type PartEditor,
  } from '../lib/api/catalog'
  import { setFleetPart } from '../lib/setupStore.svelte'

  interface Props {
    vid: number
    initialType?: string
    onInstalled?: () => void
    onBack?: () => void
  }

  let { vid, initialType = 'body', onInstalled, onBack }: Props = $props()

  let partType = $state('body')
  let query = $state('')
  let tag = $state('')
  let sort = $state('label')
  let parts = $state<CatalogPart[]>([])
  let selId = $state('')
  let editor = $state<PartEditor | null>(null)
  let busy = $state(false)
  let status = $state('')
  let err = $state<string | null>(null)
  let fieldVals = $state<Record<string, unknown>>({})
  let stem = $state('custom')
  let label = $state('Custom part')
  let searchTimer = 0

  const editable = $derived(editor?.editable !== false)
  const schema = $derived(editor?.schema)

  async function loadParts() {
    parts = await fetchCatalogParts(partType, { q: query, tag, sort })
  }

  interface LoadEditorOpts {
    part_id: string
    stem: string
    label: string
    clone: boolean
  }

  type LoadEditorArg = Partial<LoadEditorOpts>

  async function loadEditor(opts: LoadEditorArg = {}) {
    busy = true
    err = null
    try {
      editor = await fetchPartEditor({ type: partType, ...opts })
      selId = editor.doc?.id ?? ''
      stem = String(editor.values?.stem ?? opts.stem ?? 'custom')
      label = String(editor.values?.label ?? opts.label ?? `New ${partType}`)
      fieldVals = { ...(editor.values ?? {}) }
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      busy = false
    }
  }

  async function refresh() {
    await loadParts()
    if (!editor) await loadEditor(selId ? { part_id: selId } : {})
  }

  function onSearchInput() {
    clearTimeout(searchTimer)
    searchTimer = window.setTimeout(() => void loadParts(), 220)
  }

  function setField(name: string, value: unknown, idx: number | undefined = undefined) {
    if (idx !== undefined) {
      const arr = Array.isArray(fieldVals[name]) ? [...(fieldVals[name] as unknown[])] : []
      arr[idx] = value
      fieldVals = { ...fieldVals, [name]: arr }
    } else {
      fieldVals = { ...fieldVals, [name]: value }
    }
  }

  function collectFields(): Record<string, unknown> {
    const out: Record<string, unknown> = {}
    for (const [k, v] of Object.entries(fieldVals)) {
      if (k === 'stem' || k === 'label') continue
      out[k] = v
    }
    return out
  }

  async function onSave(install: boolean) {
    if (!editor) return
    busy = true
    err = null
    try {
      const r = await savePartFields({
        type: partType,
        stem,
        label,
        fields: collectFields(),
        doc: editor.doc,
      })
      status = `Saved ${r.part_id}`
      await loadEditor({ part_id: r.part_id })
      await loadParts()
      if (install) {
        const slot = installSlotForPartType(partType, r.part_id)
        if (slot) {
          await setFleetPart(vid, slot, r.part_id)
          onInstalled?.()
        }
      }
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      busy = false
    }
  }

  async function onDelete() {
    if (!selId || !editor?.editable) return
    if (!confirm(`Delete part ${selId}?`)) return
    busy = true
    try {
      await deleteCatalogPart(selId)
      selId = ''
      editor = null
      status = 'Part deleted'
      await refresh()
    } catch (e) {
      err = e instanceof Error ? e.message : String(e)
    } finally {
      busy = false
    }
  }

  async function onClone() {
    if (!selId) return
    const newStem = prompt('New stem', `${stem}_copy`)
    if (!newStem) return
    const newLabel = prompt('Label', `${label} (copy)`) || `${label} (copy)`
    await loadEditor({ part_id: selId, stem: newStem, label: newLabel, clone: true })
  }

  async function onImportYaml() {
    const input = document.createElement('input')
    input.type = 'file'
    input.accept = '.yaml,.yml'
    input.onchange = async () => {
      const f = input.files?.[0]
      if (!f) return
      try {
        const r = await importPartYaml(await f.text())
        status = `Imported ${r.part_id}`
        partType = r.part_id.split('.')[0] || partType
        await loadEditor({ part_id: r.part_id })
        await loadParts()
      } catch (e) {
        err = e instanceof Error ? e.message : String(e)
      }
    }
    input.click()
  }

  function onExportYaml() {
    if (!editor?.yaml) return
    downloadText(`${selId || 'part'}.yaml`, editor.yaml)
  }

  $effect(() => {
    partType = initialType
  })

  $effect(() => {
    partType
    void refresh()
  })

  $effect(() => {
    return () => clearTimeout(searchTimer)
  })
</script>

<div class="plib">
  <header class="head">
    <button type="button" class="back" onclick={() => onBack?.()}>← Assembly</button>
    <span class="title">Parts library</span>
    {#if status}<span class="status">{status}</span>{/if}
  </header>

  {#if err}<div class="err">{err}</div>{/if}

  <div class="types">
    {#each PART_LIB_TYPES as [t, lab] (t)}
      <button
        type="button"
        class:on={partType === t}
        onclick={() => { partType = t; selId = ''; editor = null }}
      >
        {lab}
      </button>
    {/each}
  </div>

  <div class="toolbar">
    <input
      type="search"
      placeholder="Search…"
      bind:value={query}
      oninput={onSearchInput}
    />
    <select bind:value={tag} onchange={() => loadParts()}>
      <option value="">All tags</option>
      <option value="sport">sport</option>
      <option value="race">race</option>
      <option value="gui_custom">custom</option>
    </select>
    <select bind:value={sort} onchange={() => loadParts()}>
      <option value="label">Name</option>
      <option value="tier">Tier</option>
      <option value="id">ID</option>
    </select>
  </div>

  <div class="split">
    <div class="cards">
      {#each parts as p (p.id)}
        <button
          type="button"
          class="pcard"
          class:on={p.id === selId}
          onclick={() => loadEditor({ part_id: p.id })}
        >
          <span class="tier">{p.card?.tier ?? p.type}</span>
          <span class="name">{p.label || p.id}</span>
          <span class="id">{p.id}</span>
        </button>
      {/each}
      <button type="button" class="pcard new" onclick={() => loadEditor({ stem: 'custom', label: `New ${partType}` })}>
        + New {partType}
      </button>
    </div>

    <div class="editor">
      {#if editor}
        {#if !editable}
          <p class="ro">Builtin part — clone to edit.</p>
        {/if}

        <div class="actions">
          <button type="button" disabled={busy} onclick={() => onSave(false)}>Save</button>
          <button type="button" class="primary" disabled={busy} onclick={() => onSave(true)}>
            Save &amp; install
          </button>
          {#if selId}
            <button type="button" disabled={busy} onclick={onClone}>Clone</button>
          {/if}
          {#if editor.editable && selId}
            <button type="button" class="danger" disabled={busy} onclick={onDelete}>Delete</button>
          {/if}
          <button type="button" onclick={onImportYaml}>Import YAML</button>
          <button type="button" onclick={onExportYaml} disabled={!editor.yaml}>Export</button>
        </div>

        <div class="form">
          <label class="fld">
            <span>Stem</span>
            <input bind:value={stem} disabled={!editable && !!selId} />
          </label>
          <label class="fld">
            <span>Label</span>
            <input bind:value={label} />
          </label>

          {#if schema?.meta_fields}
            <div class="grp">Metadata</div>
            {#each schema.meta_fields as [name, lab, kind] (name)}
              <label class="fld">
                <span>{lab}</span>
                {#if kind === 'bool'}
                  <input
                    type="checkbox"
                    checked={!!fieldVals[name]}
                    disabled={!editable}
                    onchange={(e) => setField(name, (e.target as HTMLInputElement).checked)}
                  />
                {:else}
                  <input
                    type={kind === 'num' ? 'number' : 'text'}
                    step="any"
                    value={String(fieldVals[name] ?? '')}
                    disabled={!editable}
                    oninput={(e) =>
                      setField(
                        name,
                        kind === 'num'
                          ? +(e.target as HTMLInputElement).value
                          : (e.target as HTMLInputElement).value,
                      )}
                  />
                {/if}
              </label>
            {/each}
          {/if}

          {#if schema?.fields}
            <div class="grp">Physics</div>
            {#each schema.fields as [name, lab, kind] (name)}
              <label class="fld">
                <span>{lab}</span>
                <input
                  type={kind === 'num' ? 'number' : 'text'}
                  step="any"
                  value={String(fieldVals[name] ?? '')}
                  disabled={!editable}
                  oninput={(e) =>
                    setField(
                      name,
                      kind === 'num'
                        ? +(e.target as HTMLInputElement).value
                        : (e.target as HTMLInputElement).value,
                    )}
                />
              </label>
            {/each}
          {/if}

          {#if schema?.array_fields}
            {#each schema.array_fields as [name, lab, n] (name)}
              {#each Array(n) as _, i}
                <label class="fld">
                  <span>{lab} [{i}]</span>
                  <input
                    type="number"
                    step="any"
                    value={String((fieldVals[name] as number[])?.[i] ?? 0)}
                    disabled={!editable}
                    oninput={(e) => setField(name, +(e.target as HTMLInputElement).value, i)}
                  />
                </label>
              {/each}
            {/each}
          {/if}

          {#if schema?.bool_fields}
            {#each schema.bool_fields as [name, lab] (name)}
              <label class="fld chk">
                <input
                  type="checkbox"
                  checked={!!fieldVals[name]}
                  disabled={!editable}
                  onchange={(e) => setField(name, (e.target as HTMLInputElement).checked)}
                />
                <span>{lab}</span>
              </label>
            {/each}
          {/if}
        </div>

        {#if editor.yaml}
          <details class="yaml">
            <summary>YAML preview</summary>
            <pre>{editor.yaml}</pre>
          </details>
        {/if}
      {:else if busy}
        <p class="hint">Loading…</p>
      {:else}
        <p class="hint">Select or create a part.</p>
      {/if}
    </div>
  </div>
</div>

<style>
  .plib {
    display: flex;
    flex-direction: column;
    height: 100%;
    min-height: 0;
    font-size: var(--fs-sm);
  }
  .head {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 8px;
  }
  .back {
    border: none;
    background: none;
    color: var(--acc);
    font-size: var(--fs-xs);
    padding: 0;
  }
  .title { font-weight: 600; }
  .status { font-size: var(--fs-xs); color: var(--ok); margin-left: auto; }
  .err {
    padding: 6px 8px;
    background: #fef2f2;
    color: var(--err);
    border-radius: var(--radius);
    margin-bottom: 6px;
    font-size: var(--fs-xs);
  }
  .types {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
    margin-bottom: 6px;
  }
  .types button {
    font-size: var(--fs-xs);
    padding: 2px 6px;
    border: 1px solid var(--border);
    border-radius: 4px;
    background: var(--bg);
  }
  .types button.on {
    border-color: var(--acc);
    color: var(--acc);
    background: #e8f7fd;
  }
  .toolbar {
    display: flex;
    gap: 4px;
    margin-bottom: 8px;
  }
  .toolbar input, .toolbar select {
    flex: 1;
    font-size: var(--fs-xs);
    padding: 3px 6px;
    border: 1px solid var(--border);
    border-radius: 4px;
  }
  .split {
    display: grid;
    grid-template-columns: 120px 1fr;
    gap: 8px;
    flex: 1;
    min-height: 0;
    overflow: hidden;
  }
  .cards, .editor {
    overflow: auto;
    min-height: 0;
  }
  .pcard {
    display: flex;
    flex-direction: column;
    width: 100%;
    text-align: left;
    padding: 6px;
    margin-bottom: 4px;
    border: 1px solid var(--border);
    border-radius: var(--radius);
    background: var(--surface);
    cursor: pointer;
  }
  .pcard.on { border-color: var(--acc); background: #e8f7fd; }
  .pcard.new { border-style: dashed; color: var(--dim); }
  .tier { font-size: 8px; text-transform: uppercase; color: var(--dim); }
  .name { font-weight: 600; font-size: var(--fs-xs); }
  .id { font-size: 8px; color: var(--dim); word-break: break-all; }
  .ro { font-size: var(--fs-xs); color: var(--dim); margin: 0 0 6px; }
  .actions {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
    margin-bottom: 8px;
  }
  .actions button {
    font-size: var(--fs-xs);
    padding: 3px 8px;
    border: 1px solid var(--border);
    border-radius: 4px;
    background: var(--surface);
  }
  .actions .primary {
    background: var(--acc);
    border-color: var(--acc);
    color: #001018;
    font-weight: 600;
  }
  .actions .danger { color: var(--err); }
  .grp {
    font-size: var(--fs-xs);
    text-transform: uppercase;
    color: var(--dim);
    margin: 8px 0 4px;
  }
  .fld {
    display: flex;
    flex-direction: column;
    margin-bottom: 4px;
    font-size: var(--fs-xs);
  }
  .fld span { color: var(--dim); margin-bottom: 1px; }
  .fld input[type='text'],
  .fld input[type='number'] {
    padding: 3px 6px;
    border: 1px solid var(--border);
    border-radius: 4px;
  }
  .fld.chk {
    flex-direction: row;
    align-items: center;
    gap: 6px;
  }
  .yaml {
    margin-top: 8px;
    font-size: var(--fs-xs);
  }
  .yaml pre {
    max-height: 120px;
    overflow: auto;
    background: var(--bg);
    padding: 6px;
    border-radius: 4px;
    font-size: 9px;
  }
  .hint { color: var(--dim); text-align: center; padding: 16px; }
</style>
