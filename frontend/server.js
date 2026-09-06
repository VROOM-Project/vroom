// Minimal static file server + proxy to vroom-express. No dependencies.
//
//   GET  /               -> public/index.html
//   GET  /rules.js       -> docs/waste_rules.json wrapped as `window.WASTE_RULES = ...`
//   GET  /rules.json     -> docs/waste_rules.json as is
//   GET  /defaults.js    -> docs/waste_defaults.json as `window.WASTE_DEFAULTS = ...`
//   GET  /defaults.json  -> docs/waste_defaults.json as is
//   GET  /<file>         -> public/<file>
//   ANY  /api/<path>     -> ${VROOM_URL}/<path>   (POST /api -> vroom-express solve endpoint)
//
// Env: PORT (default 8080), VROOM_URL (default http://localhost:3000)

const http = require("http");
const fs = require("fs");
const path = require("path");
const { URL } = require("url");

const PORT = Number(process.env.PORT || 8080);
const VROOM_URL = new URL(process.env.VROOM_URL || "http://localhost:3000");
const PUBLIC_DIR = path.join(__dirname, "public");
const DOCS_DIR = path.join(__dirname, "..", "docs");
// Single source of truth for truck types and their loading rules.
const RULES_FILE = path.join(DOCS_DIR, "waste_rules.json");
// Starting values the planner may change (company site, fleet, times).
const DEFAULTS_FILE = path.join(DOCS_DIR, "waste_defaults.json");

function serveConfig(res, file, globalName, asScript) {
  fs.readFile(file, "utf8", (err, data) => {
    if (err) return sendJson(res, 500, { error: `cannot read ${file}: ${err.message}` });
    try {
      JSON.parse(data);
    } catch (e) {
      return sendJson(res, 500, { error: `${file} is not valid JSON: ${e.message}` });
    }
    if (asScript) {
      res.writeHead(200, { "Content-Type": MIME[".js"], "Cache-Control": "no-cache" });
      res.end(`window.${globalName} = ${data.trim()};\n`);
    } else {
      res.writeHead(200, { "Content-Type": MIME[".json"], "Cache-Control": "no-cache" });
      res.end(data);
    }
  });
}

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
    if (req.url === "/rules.js") return serveConfig(res, RULES_FILE, "WASTE_RULES", true);
    if (req.url === "/rules.json") return serveConfig(res, RULES_FILE, "WASTE_RULES", false);
    if (req.url === "/defaults.js") return serveConfig(res, DEFAULTS_FILE, "WASTE_DEFAULTS", true);
    if (req.url === "/defaults.json") return serveConfig(res, DEFAULTS_FILE, "WASTE_DEFAULTS", false);
    serveStatic(req, res);
  })
  .listen(PORT, () => {
    console.log(`Frontend:      http://localhost:${PORT}`);
    console.log(`Proxy /api ->  ${VROOM_URL.origin}`);
  });
