import { defineConfig, loadEnv } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";

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
