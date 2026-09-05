#!/usr/bin/env bash
# Pre-run setup for the frontend:
#   1. (optional) builds the OSRM routing data from osrm-data/*.osm.pbf if the
#      derived *.osrm files are missing, or if you pass --rebuild-osrm
#   2. starts the OSRM + vroom-express stack from the repo root docker-compose.yml
#      with docker-compose.local.yml, i.e. VROOM built from this repository's
#      sources (the planner needs the `capacities` extension)
#   3. waits until vroom-express answers on http://localhost:3000/health
#   4. installs npm deps (none today, but keeps the workflow uniform)
#
# Afterwards run:  npm run dev   (from this folder), then open http://localhost:8080
#
# Notes on OSRM data:
#   * extract -> partition -> customize must run in that order, from a clean set
#     of files. Re-running osrm-partition alone on already-partitioned data
#     leaves stale cell metrics and osrm-routed then segfaults on every route/table
#     request (exit code 139, silent restart loop). Use --rebuild-osrm to redo it.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DATA_DIR="$ROOT/osrm-data"
PBF="portugal-latest.osm.pbf"
BASE="${PBF%.osm.pbf}"
OSRM_IMAGE="osrm/osrm-backend"
REBUILD=0
[ "${1:-}" = "--rebuild-osrm" ] && REBUILD=1

if ! command -v docker >/dev/null 2>&1; then
  echo "docker not found; install Docker Desktop first." >&2
  exit 1
fi

if [ ! -f "$DATA_DIR/$PBF" ]; then
  echo "Missing $DATA_DIR/$PBF. Download it, e.g.:" >&2
  echo "  curl -L -o $DATA_DIR/$PBF https://download.geofabrik.de/europe/portugal-latest.osm.pbf" >&2
  exit 1
fi

if [ "$REBUILD" = 1 ] || [ ! -f "$DATA_DIR/$BASE.osrm.mldgr" ] || [ ! -f "$DATA_DIR/$BASE.osrm.cell_metrics" ]; then
  echo "==> Building OSRM data (extract / partition / customize); this takes a while"
  (cd "$ROOT" && docker compose stop osrm >/dev/null 2>&1 || true)
  if [ "$REBUILD" = 1 ]; then
    BK="$DATA_DIR/_old_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$BK"
    find "$DATA_DIR" -maxdepth 1 -name "$BASE.osrm*" -exec mv {} "$BK/" \;
    echo "    previous derived files moved to $BK (delete when happy)"
  fi
  # Unix-style path for Docker Desktop on Windows (Git Bash).
  MOUNT="$DATA_DIR"
  command -v cygpath >/dev/null 2>&1 && MOUNT="$(cygpath -w "$DATA_DIR")"
  docker run --rm -v "$MOUNT:/data" "$OSRM_IMAGE" osrm-extract -p /opt/car.lua "/data/$PBF"
  docker run --rm -v "$MOUNT:/data" "$OSRM_IMAGE" osrm-partition "/data/$BASE.osrm"
  docker run --rm -v "$MOUNT:/data" "$OSRM_IMAGE" osrm-customize "/data/$BASE.osrm"
  echo "    OSRM data ready."
fi

echo "==> Starting OSRM + vroom-express built from local sources (docker compose up -d --build)"
(cd "$ROOT" && docker compose -f docker-compose.yml -f docker-compose.local.yml up -d --build)

echo "==> Waiting for vroom-express on http://localhost:3000/health"
for i in $(seq 1 60); do
  if curl -fs http://localhost:3000/health >/dev/null 2>&1; then
    echo "    vroom-express is up."
    break
  fi
  if [ "$i" -eq 60 ]; then
    echo "    vroom-express did not come up in time; check 'docker compose logs vroom'." >&2
    exit 1
  fi
  sleep 2
done

echo "==> Smoke test: one OSRM route request (catches the silent-segfault case)"
if curl -fs -m 20 "http://localhost:5000/route/v1/car/-9.15,38.72;-9.16,38.73?overview=false" >/dev/null; then
  echo "    OSRM routing OK."
else
  echo "    OSRM route request failed. If 'docker compose ps' shows osrm restarting," >&2
  echo "    the data files are inconsistent: rerun  $0 --rebuild-osrm" >&2
  exit 1
fi

echo "==> npm install"
(cd "$HERE" && npm install --no-audit --no-fund)

echo
echo "Done. Start the UI with:"
echo "    cd $HERE && npm run dev"
echo "then open http://localhost:8080"
