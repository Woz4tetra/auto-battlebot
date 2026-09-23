<script lang="ts">
  let {
    on,
    label,
    disabled = false,
    onchange,
  }: {
    on: boolean;
    label: string;
    disabled?: boolean;
    onchange: (next: boolean) => void;
  } = $props();
</script>

<button
  class="toggle"
  class:on
  aria-pressed={on}
  aria-label={label}
  {disabled}
  onclick={() => onchange(!on)}
>
  <span class="knob"></span>
</button>

<style>
  /* 64x36 visible pill inside a 44 px tall hit area. */
  .toggle {
    position: relative;
    flex-shrink: 0;
    width: 64px;
    height: 44px;
    padding: 0;
    border: none;
    background: transparent;
  }
  .toggle::before {
    content: "";
    position: absolute;
    left: 0;
    top: 4px;
    width: 64px;
    height: 36px;
    border: 2px solid var(--ink);
    border-radius: 18px;
    background: transparent;
  }
  .toggle.on::before {
    background: var(--ink);
  }
  .knob {
    position: absolute;
    left: 5px;
    top: 9px;
    width: 26px;
    height: 26px;
    border-radius: 13px;
    background: var(--ink);
  }
  .toggle.on .knob {
    left: 33px;
    background: var(--bg);
  }
  .toggle:disabled {
    opacity: 0.4;
  }
</style>
