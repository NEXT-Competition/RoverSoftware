// Deno front door for the base-station UI.
//
// One entrypoint, three run modes:
//   deno task serve      -> LAN server (iPad Safari, kiosk Chromium)
//   deno task desktop     -> native window with HMR   (needs Deno >= 2.9)
//   deno task bundle      -> native cross-platform binary (needs Deno >= 2.9)
//
// It serves the Vite-built client from ./dist and reverse-proxies the two
// backend endpoints to the Python bridge, so the SPA is always same-origin.
//
// Config (env) — all optional, RS_-prefixed to match the rest of the project;
// PORT/HOST are also honored for convenience:
//   RS_UI_PORT / PORT    port to bind                (default 8000)
//   RS_UI_HOST / HOST    hostname to bind            (default 0.0.0.0 — LAN-reachable)
//   RS_UPSTREAM          host:port of the Python bridge
//                        (default 127.0.0.1:<RS_WEB_PORT || 8001>)

import { serveDir } from "@std/http/file-server";
import { proxyTiles, proxyWs } from "./proxy.ts";

const env = (k: string) => Deno.env.get(k);
const UPSTREAM = env("RS_UPSTREAM") ??
  `127.0.0.1:${env("RS_WEB_PORT") ?? "8001"}`;
const PORT = Number(env("RS_UI_PORT") ?? env("PORT") ?? 8000);
const HOST = env("RS_UI_HOST") ?? env("HOST") ?? "0.0.0.0";

// dist/ sits next to this file's parent (basestation-ui/dist).
const DIST = new URL("../dist", import.meta.url).pathname;

async function handler(req: Request): Promise<Response> {
  const url = new URL(req.url);

  if (url.pathname === "/ws") return proxyWs(req, UPSTREAM);
  if (url.pathname.startsWith("/tiles/")) {
    return proxyTiles(url.pathname, UPSTREAM);
  }

  // Static assets from the built client.
  const res = await serveDir(req, { fsRoot: DIST, quiet: true });
  // SPA fallback: unknown non-asset routes serve index.html so deep links work.
  if (res.status === 404 && !hasFileExtension(url.pathname)) {
    return serveDir(new Request(new URL("/index.html", url), req), {
      fsRoot: DIST,
      quiet: true,
    });
  }
  return res;
}

function hasFileExtension(pathname: string): boolean {
  const last = pathname.split("/").pop() ?? "";
  return last.includes(".");
}

Deno.serve({ port: PORT, hostname: HOST }, handler);
console.log(
  `[base-ui] serving ./dist on http://${HOST}:${PORT}  ->  bridge ws://${UPSTREAM}/ws`,
);
