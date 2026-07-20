// Reverse-proxy helpers that keep the browser talking to a same-origin /ws and
// /tiles, while the real work happens on the Python base-station bridge
// (run_basestation.py). This is what lets one built SPA run identically as a
// native `deno desktop` window, as a `deno serve` LAN server for an iPad, or
// behind Vite in dev — the client never needs to know where the bridge lives.

/**
 * Splice the browser <-> Python WebSocket. The browser connects to our /ws; we
 * open a second socket to the upstream bridge and pump frames both ways.
 *
 * Frames the client sends before the upstream handshake completes (e.g. an
 * E-STOP fired the instant the page loads, or during a reconnect) are queued
 * and flushed on upstream open, so a safety-critical command is never dropped
 * into a half-open socket.
 */
export function proxyWs(req: Request, upstream: string): Response {
  const { socket: client, response } = Deno.upgradeWebSocket(req);
  const up = new WebSocket(`ws://${upstream}/ws`);

  // Buffer client->upstream frames until the upstream socket is open.
  const pending: (string | ArrayBufferLike | Blob | ArrayBufferView)[] = [];
  let upOpen = false;

  up.onopen = () => {
    upOpen = true;
    for (const m of pending) up.send(m as string);
    pending.length = 0;
  };
  // Fleet snapshots (upstream -> browser).
  up.onmessage = (e) => {
    if (client.readyState === WebSocket.OPEN) client.send(e.data);
  };
  // Actions (browser -> upstream), queued until the bridge is ready.
  client.onmessage = (e) => {
    if (upOpen && up.readyState === WebSocket.OPEN) up.send(e.data);
    else pending.push(e.data);
  };

  const teardown = () => {
    try {
      if (client.readyState === WebSocket.OPEN) client.close();
    } catch { /* ignore */ }
    try {
      if (
        up.readyState === WebSocket.OPEN ||
        up.readyState === WebSocket.CONNECTING
      ) up.close();
    } catch { /* ignore */ }
  };
  client.onclose = teardown;
  client.onerror = teardown;
  up.onclose = teardown;
  up.onerror = teardown;

  return response;
}

/**
 * Proxy the robot's live MJPEG feed from the Python bridge's
 * /video/{robot_id}.mjpg. This is an endless multipart/x-mixed-replace stream,
 * so we hand the upstream body straight through (no buffering) — the browser's
 * <img> consumes it frame by frame. The response body is a stream, so back-
 * pressure and client disconnects propagate to the upstream fetch naturally.
 */
export async function proxyVideo(
  pathname: string,
  upstream: string,
): Promise<Response> {
  try {
    const r = await fetch(`http://${upstream}${pathname}`);
    if (!r.body) return new Response(null, { status: r.status });
    return new Response(r.body, {
      status: r.status,
      headers: {
        "content-type": r.headers.get("content-type") ??
          "multipart/x-mixed-replace; boundary=frame",
        "cache-control": "no-store",
      },
    });
  } catch {
    return new Response(null, { status: 502 }); // bridge unreachable
  }
}

/**
 * Proxy a raster map tile from the Python bridge's /tiles/{z}/{x}/{y}.png.
 * Status is preserved so the bridge's 204 (uncached + offline -> blank tile)
 * passes through and Leaflet renders it as a transparent tile rather than a
 * broken-image icon.
 */
export async function proxyTiles(
  pathname: string,
  upstream: string,
): Promise<Response> {
  try {
    const r = await fetch(`http://${upstream}${pathname}`);
    if (r.status === 204 || !r.body) {
      return new Response(null, { status: r.status });
    }
    return new Response(r.body, {
      status: r.status,
      headers: {
        "content-type": "image/png",
        "cache-control": "public, max-age=86400",
      },
    });
  } catch {
    // Upstream unreachable: behave like the offline "blank tile" path.
    return new Response(null, { status: 204 });
  }
}
