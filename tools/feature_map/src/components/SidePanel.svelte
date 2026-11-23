<script lang="ts">
  import CopyButton from "./CopyButton.svelte";
  import { allProofs } from "../lib/graph";
  import type { Feature, ProofEntry } from "../lib/types";

  let { feature, onclose }: { feature: Feature; onclose: () => void } = $props();

  const runnable = $derived(
    allProofs(feature).filter((p) => p.runner !== "hil" && p.runner !== "sim"),
  );
  const rerun = $derived(
    runnable.filter((p) => p.state === "stale" || p.state === "failed" || p.state === "unrun"),
  );
  const onlyCommand = (proofs: ProofEntry[]) =>
    `scripts/verify --only ${proofs.map((p) => p.id).join(" ")}`;

  function seconds(value: number | null): string {
    if (value === null) return "";
    return value < 1 ? `${Math.round(value * 1000)} ms` : `${value.toFixed(1)} s`;
  }
</script>

{#snippet proofRows(title: string, proofs: ProofEntry[])}
  {#if proofs.length}
    <h4>{title}</h4>
    <table>
      <tbody>
        {#each proofs as proof (proof.id)}
          <tr>
            <td><i class="dot {proof.state}" title={proof.state}></i></td>
            <td class="mono id">
              {proof.id}
              {#if proof.changed.length}
                <div class="changed">changed: {proof.changed.slice(0, 3).join(", ")}</div>
              {/if}
            </td>
            <td class="num">{seconds(proof.last_seconds)}</td>
            <td class="mono">{proof.last_pass_commit ?? ""}</td>
            <td class="num">{proof.flakes ? `${proof.flakes} flaky` : ""}</td>
            <td>
              {#if proof.runner !== "hil" && proof.runner !== "sim"}
                <CopyButton text={proof.command} label="run" />
              {/if}
            </td>
          </tr>
        {/each}
      </tbody>
    </table>
  {/if}
{/snippet}

<aside>
  <header>
    <div>
      <h3><i class="dot {feature.state}"></i>{feature.name}</h3>
      <span class="muted">{feature.key} · {feature.stage} · {feature.status}</span>
    </div>
    <button onclick={onclose}>close</button>
  </header>

  <div class="actions">
    {#if runnable.length}<CopyButton text={onlyCommand(runnable)} label="copy: run all" />{/if}
    {#if rerun.length}<CopyButton
        text={onlyCommand(rerun)}
        label="copy: rerun stale and failed"
      />{/if}
  </div>

  {#if !allProofs(feature).length}
    <p class="gap-note">No proof exercises this feature yet.</p>
  {/if}
  {@render proofRows("Entry-point proofs", feature.proofs)}
  {#each feature.variants as variant (variant.type)}
    <h4><i class="dot {variant.state}"></i>{variant.type}</h4>
    {#if variant.config.profiles?.length}
      <p class="muted">selected by {variant.config.profiles.join(", ")}</p>
    {/if}
    {@render proofRows("proofs", variant.proofs)}
    {#if variant.config.fields?.length}
      <details>
        <summary>[{variant.config.section}] {variant.config.fields.length} keys</summary>
        <ul class="mono keys">
          {#each variant.config.fields as field (field.section + field.key)}
            <li>{field.key} = {field.required ? "required" : field.default}</li>
          {/each}
        </ul>
      </details>
    {/if}
  {/each}
  {@render proofRows("Supporting unit tests", feature.supporting)}

  {#if feature.links.length}
    <h4>Links</h4>
    <ul>
      {#each feature.links as link (link.name)}
        <li class="mono">{link.name}: {link.grade ?? "never graded"}</li>
      {/each}
    </ul>
  {/if}

  <h4>Code</h4>
  <ul class="mono">
    {#each feature.code as path (path)}<li>{path}</li>{/each}
  </ul>
  {#if feature.touches.length}
    <details>
      <summary>{feature.touches.length} files from coverage</summary>
      <ul class="mono">
        {#each feature.touches as path (path)}<li>{path}</li>{/each}
      </ul>
    </details>
  {/if}

  {#if feature.commits.length}
    <h4>Recent commits{feature.churn !== null ? ` (${feature.churn} since last green)` : ""}</h4>
    <ul>
      {#each feature.commits as commit (commit.sha)}
        <li><code>{commit.sha}</code> {commit.date} {commit.subject}</li>
      {/each}
    </ul>
  {/if}

  {#if feature.docs.length}
    <h4>Docs</h4>
    <ul class="mono">
      {#each feature.docs as doc (doc)}<li>{doc}</li>{/each}
    </ul>
  {/if}

  <h4>Reach</h4>
  <ul class="mono">
    {#each feature.reach as reach (reach)}<li>{reach}</li>{/each}
  </ul>
</aside>

<style>
  aside {
    width: 440px;
    max-width: 100vw;
    height: 100%;
    overflow: auto;
    padding: 16px 18px 40px;
    background: var(--panel);
    border-left: 1px solid var(--line);
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    gap: 12px;
  }
  h3 {
    margin: 0 0 4px;
  }
  h4 {
    margin: 18px 0 6px;
  }
  .muted,
  .changed {
    color: var(--muted);
    font-size: 12px;
  }
  .actions {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 12px;
  }
  .gap-note {
    color: var(--gap);
  }
  table {
    width: 100%;
    border-collapse: collapse;
  }
  td {
    padding: 3px 4px;
    border-bottom: 1px solid var(--line);
    vertical-align: top;
  }
  .id {
    word-break: break-all;
  }
  .num {
    text-align: right;
    white-space: nowrap;
    font-variant-numeric: tabular-nums;
  }
  ul {
    margin: 0;
    padding-left: 18px;
  }
  .keys {
    max-height: 240px;
    overflow: auto;
  }
  @media (max-width: 700px) {
    aside {
      width: 100vw;
      position: fixed;
      inset: 0;
      z-index: 5;
    }
  }
</style>
