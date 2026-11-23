<script lang="ts">
  import cytoscape from "cytoscape";
  import { onDestroy, onMount } from "svelte";
  import type { Elements, NodeData } from "../lib/graph";
  import { COLUMN_WIDTH, columnLayout, sCurve } from "../lib/layout";

  let {
    elements,
    selected,
    onselect,
    ontoggle,
  }: {
    elements: Elements;
    selected: string | null;
    onselect: (feature: string | null) => void;
    ontoggle: (group: string) => void;
  } = $props();

  let container: HTMLDivElement;
  let cy: cytoscape.Core | null = null;

  function css(name: string): string {
    return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  }

  function style(): cytoscape.StylesheetJson {
    const color = (state: string) => css(`--${state}`) || css("--unrun");
    return [
      {
        selector: "node",
        style: {
          label: "data(label)",
          "font-size": 11,
          color: css("--ink"),
          "text-valign": "center",
          "text-halign": "center",
          "text-wrap": "ellipsis",
          shape: "round-rectangle",
          height: 22,
          "background-color": css("--panel"),
          "border-width": 1,
          "border-color": css("--line"),
        },
      },
      ...Object.entries(COLUMN_WIDTH).map(([kind, width]) => ({
        selector: `node[kind = "${kind}"]`,
        style: { width, "text-max-width": `${width - 16}px` },
      })),
      {
        selector: 'node[kind = "reach"]',
        style: { "font-family": "monospace", "font-size": 10, color: css("--muted") },
      },
      { selector: 'node[kind = "feature"]', style: { "font-weight": "bold", "border-width": 3 } },
      {
        selector: 'node[kind = "module"]',
        style: { shape: "rectangle", "font-family": "monospace", "font-size": 10 },
      },
      ...["passing", "stale", "failed", "unrun", "gap"].map((state) => ({
        selector: `node[state = "${state}"]`,
        style: { "border-color": color(state) },
      })),
      { selector: "node[?inDiff]", style: { "background-color": css("--diff"), color: "#1d1d1b" } },
      { selector: "node.faded", style: { opacity: 0.18 } },
      { selector: "edge.faded", style: { opacity: 0.08 } },
      {
        selector: "edge",
        style: {
          width: 1,
          "line-color": css("--muted"),
          "line-opacity": 0.45,
          "curve-style": "unbundled-bezier",
          "edge-distances": "node-position",
          "source-endpoint": "90deg",
          "target-endpoint": "270deg",
        },
      },
    ];
  }

  function layout(positions: Map<string, { x: number; y: number }>): void {
    if (!cy) return;
    cy.layout({ name: "preset", positions: Object.fromEntries(positions), fit: false }).run();
    cy.edges().forEach((edge) => {
      const curve = sCurve(edge.source().position(), edge.target().position());
      edge.style({
        "control-point-distances": curve.distances,
        "control-point-weights": curve.weights,
      });
    });
    fitWidth();
  }

  function fitWidth(): void {
    if (!cy || cy.elements().empty()) return;
    const box = cy.elements().boundingBox();
    const pad = 30;
    cy.zoom(Math.max(0.15, Math.min(1.2, (cy.width() - 2 * pad) / box.w)));
    const z = cy.zoom();
    const x = box.w * z < cy.width() ? (cy.width() - box.w * z) / 2 : pad;
    const y = box.h * z < cy.height() ? (cy.height() - box.h * z) / 2 : pad;
    cy.pan({ x: x - box.x1 * z, y: y - box.y1 * z });
  }

  function wheel(event: WheelEvent): void {
    if (!cy) return;
    event.preventDefault();
    if (event.ctrlKey || event.metaKey) {
      const rect = container.getBoundingClientRect();
      const level = cy.zoom() * Math.exp(-event.deltaY * 0.002);
      cy.zoom({
        level: Math.min(3, Math.max(0.1, level)),
        renderedPosition: { x: event.clientX - rect.left, y: event.clientY - rect.top },
      });
    } else {
      cy.panBy({ x: -event.deltaX, y: -event.deltaY });
    }
  }

  function highlight(): void {
    if (!cy) return;
    cy.elements().removeClass("faded");
    if (!selected) return;
    const node = cy.getElementById(`feature:${selected}`);
    if (node.empty()) return;
    const own = cy
      .nodes()
      .filter((n) => n.data("feature") === selected && n.data("kind") !== "reach");
    const nodes = node.incomers("node").union(own).union(node.outgoers("node"));
    const keep = nodes.union(nodes.edgesWith(nodes));
    cy.elements().not(keep).addClass("faded");
  }

  onMount(() => {
    cy = cytoscape({
      container,
      style: style(),
      userZoomingEnabled: false,
      minZoom: 0.1,
      maxZoom: 3,
    });
    container.addEventListener("wheel", wheel, { passive: false });
    if (new URLSearchParams(location.search).has("debug")) {
      (window as unknown as { featureMapCy: cytoscape.Core }).featureMapCy = cy;
    }
    cy.on("tap", "node", (event) => {
      const data = event.target.data() as NodeData;
      if (data.kind === "proofs") ontoggle(data.id);
      else if (data.kind === "proof" && data.feature)
        ontoggle(`proofs:${data.feature}:${data.label.split(":")[0]}`);
      if (data.feature) onselect(data.feature);
    });
    cy.on("tap", (event) => {
      if (event.target === cy) onselect(null);
    });
  });

  $effect(() => {
    const current = elements;
    if (!cy) return;
    const positions = columnLayout(current);
    cy.batch(() => {
      cy!.elements().remove();
      cy!.add([...current.nodes, ...current.edges]);
    });
    layout(positions);
    highlight();
  });

  $effect(() => {
    void selected;
    highlight();
  });

  onDestroy(() => cy?.destroy());
</script>

<div class="graph" bind:this={container}></div>

<style>
  .graph {
    width: 100%;
    height: 100%;
  }
</style>
