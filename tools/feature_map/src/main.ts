import "./app.css";
import { mount } from "svelte";
import App from "./App.svelte";
import { live } from "./lib/live.svelte";

live.start();

export default mount(App, { target: document.getElementById("app")! });
