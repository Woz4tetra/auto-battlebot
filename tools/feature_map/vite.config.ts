import { defineConfig } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";

export default defineConfig({
  plugins: [svelte()],
  build: {
    outDir: "../../auto_battlebot/features/static",
    emptyOutDir: true,
  },
  server: {
    proxy: {
      "/api": process.env.FEATURE_MAP_API || "http://127.0.0.1:8780",
    },
  },
});
