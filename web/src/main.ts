import "@fontsource-variable/archivo/wdth.css";
import "@fontsource/ibm-plex-mono/400.css";
import "@fontsource/ibm-plex-mono/500.css";
import "@fontsource/ibm-plex-mono/600.css";
import "./app.css";
import { mount } from "svelte";
import App from "./App.svelte";
import { diagnostics } from "./lib/diagnostics.svelte";
import { status } from "./lib/status.svelte";
import { ui } from "./lib/ui.svelte";

ui.start();
status.start();
diagnostics.start();

export default mount(App, { target: document.getElementById("app")! });
