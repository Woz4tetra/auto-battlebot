import { defineConfig, loadEnv } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";

// The WebSocket URL defaults to ws://<page host>:8765 at runtime (src/lib/connection.ts).
// For `npm run dev`, VITE_WS_URL points the page at a relay on another host or port, and
// VITE_HTTP_URL is where /healthz is proxied (the relay's Crow server, default port 8080).
export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), "VITE_");
  return {
    plugins: [svelte()],
    build: {
      target: "safari16",
      assetsInlineLimit: 0,
    },
    server: {
      proxy: {
        "/healthz": env.VITE_HTTP_URL || "http://localhost:8080",
      },
    },
  };
});
