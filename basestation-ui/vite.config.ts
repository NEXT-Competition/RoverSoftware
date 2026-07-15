import { defineConfig } from "vite";
import preact from "@preact/preset-vite";

// The client is a single-page app. In dev, Vite serves it with HMR and proxies
// the two backend endpoints to the Python bridge (run it on :8001, e.g.
// `python run_basestation.py --sim --web-port 8001`). In prod the same two
// paths are proxied by the Deno front door (server/main.ts) instead, so the
// client always talks to a same-origin /ws and /tiles — identical whether it's
// running under Vite dev, `deno serve`, or a packaged `deno desktop` binary.
const UPSTREAM = process.env.RS_UPSTREAM ?? "127.0.0.1:8001";

export default defineConfig({
  plugins: [preact()],
  build: {
    target: "es2022",
    outDir: "dist",
    // Leaflet's marker images resolve fine when inlined/emitted by Vite; keep
    // assets self-contained so the offline native binary has no CDN dependency.
    assetsInlineLimit: 4096,
  },
  server: {
    host: true, // expose on the LAN so an iPad can hit the dev server too
    port: 5173,
    proxy: {
      "/ws": { target: `ws://${UPSTREAM}`, ws: true, changeOrigin: true },
      "/tiles": { target: `http://${UPSTREAM}`, changeOrigin: true },
    },
  },
});
