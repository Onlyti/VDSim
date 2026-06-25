<script lang="ts">
  import { docHref } from '../lib/docsHelp'

  interface Props {
    topic: string
    label?: string
  }

  let { topic, label = '?' }: Props = $props()
  const href = $derived(docHref(topic))
</script>

{#if href}
  <a class="help" {href} target="_blank" rel="noopener noreferrer" title="Open docs" aria-label="Help: {topic}"
    >{label}</a
  >
{/if}

<style>
  .help {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: 14px;
    height: 14px;
    margin-left: 4px;
    border: 1px solid var(--dim);
    border-radius: 50%;
    font: bold 9px/1 system-ui;
    color: var(--dim);
    text-decoration: none;
    vertical-align: middle;
  }
  .help:hover {
    color: var(--acc);
    border-color: var(--acc);
  }
</style>
