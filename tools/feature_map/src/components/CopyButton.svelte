<script lang="ts">
  let { text, label = "copy" }: { text: string; label?: string } = $props();
  let copied = $state(false);

  async function copy(): Promise<void> {
    await navigator.clipboard.writeText(text);
    copied = true;
    setTimeout(() => (copied = false), 1200);
  }
</script>

<button class="copy" title={text} onclick={copy}>{copied ? "copied" : label}</button>

<style>
  .copy {
    padding: 1px 8px;
    font-size: 12px;
  }
</style>
