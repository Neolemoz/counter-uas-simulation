import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import cesium from "vite-plugin-cesium";
import path from "path";

export default defineConfig({
  plugins: [react(), cesium()],
  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
  server: {
    port: 5174,
    strictPort: true,
    proxy: {
      "/v1": {
        target: "http://127.0.0.1:18765",
        changeOrigin: true,
      },
    },
  },
  preview: {
    port: 5174,
    strictPort: true,
    proxy: {
      "/v1": {
        target: "http://127.0.0.1:18765",
        changeOrigin: true,
      },
    },
  },
});
