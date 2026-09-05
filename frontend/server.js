// Minimal static file server + proxy to vroom-express. No dependencies.
//
//   GET  /            -> public/index.html
//   GET  /<file>      -> public/<file>
//   ANY  /api/<path>  -> ${VROOM_URL}/<path>   (POST /api -> vroom-express solve endpoint)
//
// Env: PORT (default 8080), VROOM_URL (default http://localhost:3000)

const http = require("http");
const fs = require("fs");
const path = require("path");
const { URL } = require("url");

const PORT = Number(process.env.PORT || 8080);
const VROOM_URL = new URL(process.env.VROOM_URL || "http://localhost:3000");
const PUBLIC_DIR = path.join(__dirname, "public");

const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".png": "image/png",
  ".svg": "image/svg+xml",
  ".ico": "image/x-icon",
};

function sendJson(res, status, body) {
  res.writeHead(status, { "Content-Type": "application/json; charset=utf-8" });
  res.end(JSON.stringify(body));
}

function serveStatic(req, res) {
  let pathname = decodeURIComponent(new URL(req.url, "http://x").pathname);
  if (pathname === "/") pathname = "/index.html";
  const filePath = path.normalize(path.join(PUBLIC_DIR, pathname));
  if (!filePath.startsWith(PUBLIC_DIR)) {
    return sendJson(res, 403, { error: "forbidden" });
  }
  fs.readFile(filePath, (err, data) => {
    if (err) return sendJson(res, 404, { error: "not found" });
    res.writeHead(200, {
      "Content-Type": MIME[path.extname(filePath)] || "application/octet-stream",
    });
    res.end(data);
  });
}

function proxy(req, res) {
  const targetPath = req.url.replace(/^\/api/, "") || "/";
  const options = {
    hostname: VROOM_URL.hostname,
    port: VROOM_URL.port || 80,
    path: targetPath,
    method: req.method,
    headers: { ...req.headers, host: VROOM_URL.host },
  };
  const upstream = http.request(options, (up) => {
    res.writeHead(up.statusCode, up.headers);
    up.pipe(res);
  });
  upstream.on("error", (err) => {
    sendJson(res, 502, {
      error: `Cannot reach vroom-express at ${VROOM_URL.origin} (${err.code || err.message}). ` +
        "Is the docker compose stack running? (frontend/setup.sh starts it)",
    });
  });
  req.pipe(upstream);
}

http
  .createServer((req, res) => {
    if (req.url.startsWith("/api")) return proxy(req, res);
    if (req.method !== "GET") return sendJson(res, 405, { error: "method not allowed" });
    serveStatic(req, res);
  })
  .listen(PORT, () => {
    console.log(`Frontend:      http://localhost:${PORT}`);
    console.log(`Proxy /api ->  ${VROOM_URL.origin}`);
  });
