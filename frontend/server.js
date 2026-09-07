// Minimal static file server + proxy to vroom-express. No dependencies.
//
//   GET  /               -> public/index.html
//   GET  /rules.js       -> docs/waste_rules.json wrapped as `window.WASTE_RULES = ...`
//   GET  /rules.json     -> docs/waste_rules.json as is
//   GET  /defaults.js    -> docs/waste_defaults.json as `window.WASTE_DEFAULTS = ...`
//   GET  /defaults.json  -> docs/waste_defaults.json as is
//   GET  /zones.js       -> docs/no_go_zones.json as `window.WASTE_ZONES = ...`
//   GET  /zones.json     -> docs/no_go_zones.json as is
//   PUT  /zones.json     -> replace the "zones" array of that file (the Zones tab)
//   GET  /zones/status   -> per restricted profile: is its routing data built, does
//                           it match the zones as they are now, is its OSRM up
//   POST /zones/build    -> run scripts/build_zone_graphs.sh --apply in the background
//                           (the Zones tab's Apply button); 409 while one is running
//   GET  /zones/build    -> state and log of that run
//   GET  /<file>         -> public/<file>
//   ANY  /api/<path>     -> ${VROOM_URL}/<path>   (POST /api -> vroom-express solve endpoint)
//
// Env: PORT (default 8080), VROOM_URL (default http://localhost:3000),
//      ZONES_BASH (the bash that runs the build script; found on its own
//      otherwise, see findBash)

const http = require("http");
const net = require("net");
const fs = require("fs");
const path = require("path");
const crypto = require("crypto");
const { spawn } = require("child_process");
const { URL } = require("url");

const PORT = Number(process.env.PORT || 8080);
const VROOM_URL = new URL(process.env.VROOM_URL || "http://localhost:3000");
const ROOT_DIR = path.join(__dirname, "..");
const PUBLIC_DIR = path.join(__dirname, "public");
const DOCS_DIR = path.join(ROOT_DIR, "docs");
const OSRM_DIR = path.join(ROOT_DIR, "osrm-data");
// Builds the restricted routing data and (re)starts its containers.
const BUILD_SCRIPT = path.join(ROOT_DIR, "scripts", "build_zone_graphs.sh");
// Single source of truth for truck types and their loading rules.
const RULES_FILE = path.join(DOCS_DIR, "waste_rules.json");
// Starting values the planner may change (company site, fleet, times).
const DEFAULTS_FILE = path.join(DOCS_DIR, "waste_defaults.json");
// Areas the trucks may not drive through, and the routing profiles that
// enforce them. The only config file the UI writes back to: zones are
// drawn on the map and have to reach the build script, which is
// server-side (scripts/build_zone_graphs.sh).
const ZONES_FILE = path.join(DOCS_DIR, "no_go_zones.json");

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

// ------------------------------------------------------------------ zones

// Fingerprint of what a profile's routing data was built from, so a
// dataset built before the last zone edit can be spotted. The very same
// text is built by scripts/build_zone_graphs.sh, so the two must stay in
// step: one "penalty" line, then one line per zone sorted by id, with
// every number written to six decimals (about 10 cm, well beyond what
// anyone draws by hand). A fixed text format rather than a hash of the
// JSON, because Node and Python do not serialise numbers the same way.
function zonesHash(config, profile) {
  const n = (v) => Number(v).toFixed(6);
  const penalty = config.penalty || {};
  const speed = penalty.speed_kmh === undefined ? 1 : penalty.speed_kmh;
  const rate = penalty.rate === undefined || penalty.rate === null ? "none" : n(penalty.rate);
  const lines = [`penalty speed_kmh=${n(speed)} rate=${rate}`];
  const zones = (config.zones || [])
    .filter((z) => (z.blocked_profiles || []).includes(profile))
    .slice()
    .sort((a, b) => Number(a.id) - Number(b.id));
  for (const z of zones) {
    const ring = (z.polygon || []).map((p) => `${n(p[0])},${n(p[1])}`).join(" ");
    lines.push(`zone ${z.id} ${ring}`);
  }
  return crypto.createHash("sha256").update(`${lines.join("\n")}\n`).digest("hex").slice(0, 16);
}

function readZones(cb) {
  fs.readFile(ZONES_FILE, "utf8", (err, data) => {
    if (err) return cb(new Error(`cannot read ${ZONES_FILE}: ${err.message}`));
    try {
      cb(null, JSON.parse(data));
    } catch (e) {
      cb(new Error(`${ZONES_FILE} is not valid JSON: ${e.message}`));
    }
  });
}

// Is something listening on that host port? A profile whose routing data
// is built but whose container is down would otherwise only show up as a
// puzzling solver error at the end of a plan.
function probePort(port, cb) {
  const socket = net.connect({ host: "127.0.0.1", port });
  let done = false;
  const finish = (up) => {
    if (done) return;
    done = true;
    socket.destroy();
    cb(up);
  };
  socket.setTimeout(600);
  socket.on("connect", () => finish(true));
  socket.on("timeout", () => finish(false));
  socket.on("error", () => finish(false));
}

function zonesStatus(res) {
  readZones((err, config) => {
    if (err) return sendJson(res, 500, { error: err.message });

    const profiles = {};
    for (const [name, def] of Object.entries(config.profiles || {})) {
      if (name.startsWith("_")) continue;
      const zones = (config.zones || []).filter((z) => (z.blocked_profiles || []).includes(name));
      if (!zones.length) continue;

      let meta = null;
      try {
        meta = JSON.parse(fs.readFileSync(path.join(OSRM_DIR, name, "zones.meta.json"), "utf8"));
      } catch (e) {
        meta = null;
      }
      profiles[name] = {
        zones: zones.length,
        host_port: def.host_port || null,
        built: meta !== null,
        up_to_date: meta !== null && meta.zones_hash === zonesHash(config, name),
        built_at: meta ? meta.built_at : null,
        reachable: null,
      };
    }

    const names = Object.keys(profiles).filter((name) => profiles[name].host_port);
    let pending = names.length;
    if (!pending) return sendJson(res, 200, { profiles });
    for (const name of names) {
      probePort(profiles[name].host_port, (up) => {
        profiles[name].reachable = up;
        if (--pending === 0) sendJson(res, 200, { profiles });
      });
    }
  });
}

// Replace the "zones" array, leaving everything else in the file (the
// profiles, the penalty, the comments) exactly as it is.
function saveZones(req, res) {
  let body = "";
  req.setEncoding("utf8");
  req.on("data", (chunk) => {
    body += chunk;
    if (body.length > 2 * 1024 * 1024) {
      sendJson(res, 413, { error: "zones payload too large" });
      req.destroy();
    }
  });
  req.on("end", () => {
    let incoming;
    try {
      incoming = JSON.parse(body);
    } catch (e) {
      return sendJson(res, 400, { error: `invalid JSON: ${e.message}` });
    }
    readZones((err, config) => {
      if (err) return sendJson(res, 500, { error: err.message });

      const known = Object.keys(config.profiles || {}).filter((k) => !k.startsWith("_"));
      // The default profile is served by the untouched routing data, so
      // closing an area to it would change nothing: the vehicles that
      // must avoid an area need a profile of their own.
      const defaultProfile = (config.vehicle_profiles || {}).default || "car";
      const list = Array.isArray(incoming) ? incoming : incoming.zones;
      if (!Array.isArray(list)) {
        return sendJson(res, 400, { error: 'expected a JSON body of the form {"zones": [...]}' });
      }

      const zones = [];
      const seen = new Set();
      for (const z of list) {
        const id = Number(z.id);
        if (!Number.isInteger(id) || id <= 0) {
          return sendJson(res, 400, { error: `zone id must be a positive integer, got ${z.id}` });
        }
        if (seen.has(id)) return sendJson(res, 400, { error: `duplicate zone id ${id}` });
        seen.add(id);

        const ring = Array.isArray(z.polygon) ? z.polygon : [];
        if (ring.length < 3) {
          return sendJson(res, 400, { error: `zone ${id}: a polygon needs at least three points` });
        }
        for (const p of ring) {
          if (!Array.isArray(p) || p.length !== 2 || !isFinite(p[0]) || !isFinite(p[1])) {
            return sendJson(res, 400, {
              error: `zone ${id}: polygon points are [longitude, latitude] pairs`,
            });
          }
        }

        const blocked = Array.isArray(z.blocked_profiles) ? z.blocked_profiles : [];
        for (const p of blocked) {
          if (!known.includes(p)) {
            return sendJson(res, 400, { error: `zone ${id}: unknown routing profile ${p}` });
          }
          if (p === defaultProfile) {
            return sendJson(res, 400, {
              error: `zone ${id}: ${p} is the default routing profile, which is served by the ` +
                "unrestricted routing data; give the vehicles that must avoid the area a " +
                "profile of their own in no_go_zones.json",
            });
          }
        }

        zones.push({
          id,
          name: String(z.name || `zone ${id}`),
          blocked_profiles: blocked,
          polygon: ring.map((p) => [Number(p[0]), Number(p[1])]),
        });
      }

      config.zones = zones;
      const tmp = `${ZONES_FILE}.tmp`;
      try {
        // Write then rename, so an interrupted write cannot leave the
        // single source of truth truncated.
        fs.writeFileSync(tmp, `${JSON.stringify(config, null, 2)}\n`, "utf8");
        fs.renameSync(tmp, ZONES_FILE);
      } catch (e) {
        return sendJson(res, 500, { error: `cannot write ${ZONES_FILE}: ${e.message}` });
      }
      sendJson(res, 200, { ok: true, zones: zones.length });
    });
  });
}

// ------------------------------------------------------- applying zones

// The bash that runs the build script. On Windows the one on PATH may be
// WSL's (C:\Windows\System32\bash.exe), which lives in another machine
// as far as docker and the repository paths are concerned; Git's is the
// one that works, wherever it was installed.
function findBash() {
  if (process.env.ZONES_BASH) return process.env.ZONES_BASH;
  if (process.platform !== "win32") return "bash";
  const candidates = [
    process.env.ProgramFiles,
    process.env["ProgramFiles(x86)"],
    process.env.LOCALAPPDATA && path.join(process.env.LOCALAPPDATA, "Programs"),
  ]
    .filter(Boolean)
    .map((dir) => path.join(dir, "Git", "bin", "bash.exe"));
  for (const bash of candidates) {
    if (fs.existsSync(bash)) return bash;
  }
  return "bash";
}

// One build at a time, its output kept in memory for the Zones tab to
// show. Only the last part of a long log is kept.
const LOG_LIMIT = 256 * 1024;
let build = null; // { running, started_at, finished_at, exit_code, log }

function appendLog(text) {
  build.log += text;
  if (build.log.length > LOG_LIMIT) {
    build.log = `[...]\n${build.log.slice(build.log.length - LOG_LIMIT)}`;
  }
}

function startBuild(res) {
  if (build && build.running) {
    return sendJson(res, 409, { error: "a build is already running" });
  }
  build = {
    running: true,
    started_at: new Date().toISOString(),
    finished_at: null,
    exit_code: null,
    log: "",
  };

  const bash = findBash();
  let child;
  try {
    child = spawn(bash, [BUILD_SCRIPT, "--apply"], {
      cwd: ROOT_DIR,
      // The script mounts repository paths into docker; without this,
      // Git Bash rewrites the container-side ones into Windows paths.
      env: { ...process.env, MSYS_NO_PATHCONV: "1" },
      stdio: ["ignore", "pipe", "pipe"],
      windowsHide: true,
    });
  } catch (e) {
    finishBuild(-1, `cannot start ${bash}: ${e.message}\n`);
    return sendJson(res, 500, { error: build.log.trim() });
  }
  const job = build;
  appendLog(`$ ${path.relative(ROOT_DIR, BUILD_SCRIPT).split(path.sep).join("/")} --apply\n`);
  child.stdout.on("data", (chunk) => job === build && appendLog(chunk.toString()));
  child.stderr.on("data", (chunk) => job === build && appendLog(chunk.toString()));
  child.on("error", (err) => {
    if (job === build) finishBuild(-1, `cannot run ${bash}: ${err.message}\n`);
  });
  child.on("close", (code) => {
    if (job === build) finishBuild(code === null ? -1 : code, "");
  });
  sendJson(res, 202, { ok: true, started_at: build.started_at });
}

function finishBuild(code, text) {
  if (text) appendLog(text);
  build.running = false;
  build.exit_code = code;
  build.finished_at = new Date().toISOString();
}

function buildStatus(res) {
  sendJson(res, 200, build || { running: false, started_at: null, finished_at: null, exit_code: null, log: "" });
}

// ---------------------------------------------------------------- serving

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
    if (req.method === "PUT" && req.url === "/zones.json") return saveZones(req, res);
    if (req.method === "POST" && req.url === "/zones/build") return startBuild(res);
    if (req.method !== "GET") return sendJson(res, 405, { error: "method not allowed" });
    if (req.url === "/zones/build") return buildStatus(res);
    if (req.url === "/rules.js") return serveConfig(res, RULES_FILE, "WASTE_RULES", true);
    if (req.url === "/rules.json") return serveConfig(res, RULES_FILE, "WASTE_RULES", false);
    if (req.url === "/defaults.js") return serveConfig(res, DEFAULTS_FILE, "WASTE_DEFAULTS", true);
    if (req.url === "/defaults.json") return serveConfig(res, DEFAULTS_FILE, "WASTE_DEFAULTS", false);
    if (req.url === "/zones.js") return serveConfig(res, ZONES_FILE, "WASTE_ZONES", true);
    if (req.url === "/zones.json") return serveConfig(res, ZONES_FILE, "WASTE_ZONES", false);
    if (req.url === "/zones/status") return zonesStatus(res);
    serveStatic(req, res);
  })
  .listen(PORT, () => {
    console.log(`Frontend:      http://localhost:${PORT}`);
    console.log(`Proxy /api ->  ${VROOM_URL.origin}`);
  });
