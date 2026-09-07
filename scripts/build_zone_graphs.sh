#!/usr/bin/env bash
# Build the restricted OSRM datasets that enforce the no-go zones of
# docs/no_go_zones.json, and (with --apply) put them in service.
#
# VROOM never sees a map: it asks OSRM for travel times and for the
# drawn route, so "this truck may not drive through here" cannot be a
# solver constraint. It is enforced in the road graph instead: every
# profile that some zone blocks gets its own copy of the routing data
# in which the roads inside its zones are made prohibitively slow, and
# the vehicles bound to that profile query the OSRM instance serving
# that copy. See docs/no_go_zones.md.
#
#   scripts/build_zone_graphs.sh --apply        # what the planner's Apply button runs:
#                                               # build what is missing or out of date,
#                                               # then (re)start the containers serving
#                                               # it and wait until they answer
#   scripts/build_zone_graphs.sh --apply chico   # only that profile
#   scripts/build_zone_graphs.sh                # build every restricted profile
#   scripts/build_zone_graphs.sh chico      # only that one
#   scripts/build_zone_graphs.sh --status       # what is built and whether it is stale
#   scripts/build_zone_graphs.sh --image        # (re)build the tooling image only
#
# The base dataset (osrm-data/portugal-latest.osrm*) must exist first;
# frontend/setup.sh builds it. A build is three steps, none of them a
# re-extraction (which would take hours):
#
#   1. the country extract is clipped to the region around the zones
#      with osmium, once, and the clip is kept under osrm-data/zones/
#      for as long as the zones stay inside it. Finding the road
#      segments inside the zones then scans a few MB instead of the
#      whole country;
#   2. the base routing data is hard-linked into osrm-data/<profile>/,
#      except for the files osrm-customize rewrites, which are copied;
#   3. osrm-customize --segment-speed-file applies the penalty. Only the
#      cell metrics depend on the segment speeds, the partition does
#      not, so this takes a minute or two and not hours.
#
# Environment: ZONES_IMAGE (default vroom-zones), OSRM_IMAGE (default
# osrm/osrm-backend), PBF (default portugal-latest.osm.pbf), CLIP_MARGIN
# (degrees of extract kept around the zones, default 0.05, about 5 km).
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$PWD"
DATA_DIR="$ROOT/osrm-data"
CLIP_DIR="$DATA_DIR/zones"
ZONES_FILE="$ROOT/docs/no_go_zones.json"
ZONES_IMAGE="${ZONES_IMAGE:-vroom-zones}"
OSRM_IMAGE="${OSRM_IMAGE:-osrm/osrm-backend}"
PBF="${PBF:-portugal-latest.osm.pbf}"
BASE="${PBF%.osm.pbf}"
CLIP_MARGIN="${CLIP_MARGIN:-0.05}"

# Files osrm-customize rewrites in place when given a segment speed
# file (the updated edge weights and durations, and the cell metrics
# computed from them). These get a real copy; every other file of the
# dataset is identical to the base one and is hard-linked instead, which
# is why a profile costs about 500 MB and not the full 1.3 GB.
CUSTOMIZE_WRITES=".osrm.geometry .osrm.datasource_names .osrm.turn_weight_penalties
.osrm.turn_duration_penalties .osrm.enw .osrm.cell_metrics .osrm.mldgr"

# Docker Desktop on Windows needs a Windows-style host path and no
# MSYS mangling of the container-side paths.
MOUNT="$ROOT"
if command -v cygpath > /dev/null 2>&1; then
  MOUNT="$(cygpath -w "$ROOT")"
  export MSYS_NO_PATHCONV=1
fi

# Same compose files as frontend/setup.sh, so that this script and the
# stack it started agree on the project.
COMPOSE=(docker compose -f docker-compose.yml)
if [ -f "$ROOT/docker-compose.local.yml" ]; then
  COMPOSE+=(-f docker-compose.local.yml)
fi

if ! command -v docker > /dev/null 2>&1; then
  echo "docker not found; install Docker Desktop first." >&2
  exit 1
fi

if [ ! -f "$ZONES_FILE" ]; then
  echo "Missing $ZONES_FILE." >&2
  exit 1
fi

# Small python helper reading the zone file. Plain stdlib, so the host
# interpreter does when there is one (asking what is built should not
# need a docker build); the tooling image is the fallback, and the only
# way to run osmium and zone_segments.py, which need their packages.
zones_query() {
  if command -v python3 > /dev/null 2>&1; then
    python3 - "$@"
  else
    build_image
    docker run --rm -i -v "${MOUNT}:/work" -w /work "$ZONES_IMAGE" - "$@"
  fi
}

build_image() {
  if [ -z "$(docker images -q "$ZONES_IMAGE" 2> /dev/null)" ] || [ "${1:-}" = "force" ]; then
    echo "==> Building the $ZONES_IMAGE tooling image"
    # The host-side path, like every path handed to docker below.
    docker build -t "$ZONES_IMAGE" "$MOUNT/scripts/zones"
  fi
}

# Profiles that at least one zone blocks, one per line.
restricted_profiles() {
  zones_query <<'PY'
import json, sys
config = json.load(open("docs/no_go_zones.json", encoding="utf-8"))
profiles = []
for zone in config.get("zones", []):
    for p in zone.get("blocked_profiles", []):
        if p not in profiles:
            profiles.append(p)
known = [k for k in config.get("profiles", {}) if not k.startswith("_")]
default = config.get("vehicle_profiles", {}).get("default", "car")
for p in profiles:
    if p not in known:
        sys.exit(f"zone blocks unknown profile {p}; declare it in no_go_zones.json")
    if not p.isalpha() or not p.isascii():
        # VROOM puts the profile name in the OSRM request URL
        # (/table/v1/<profile>/...), and OSRM's URL grammar accepts
        # letters only there: "car_chico" fails with "URL string
        # malformed" at solve time, long after the build.
        sys.exit(f"profile name {p} is not letters only; OSRM rejects any other "
                 "character in its request URL, so rename it (e.g. chico)")
    if p == default:
        # The default profile is served by the untouched dataset, so
        # there is nothing to penalise: a vehicle that must avoid an
        # area needs a profile of its own.
        sys.exit(f"zone blocks {p}, which is the default profile and is served by the "
                 "unrestricted routing data; give the vehicles that must avoid the area "
                 "a profile of their own in no_go_zones.json")
print("\n".join(profiles))
PY
}

# "<compose service> <host port>" of a profile, "- -" when not declared.
profile_service() {
  zones_query "$1" <<'PY'
import json, sys
config = json.load(open("docs/no_go_zones.json", encoding="utf-8"))
p = config.get("profiles", {}).get(sys.argv[1], {})
print(p.get("service") or "-", p.get("host_port") or "-")
PY
}

# Fingerprint of what a profile's data is built from, stored next to it
# so that a dataset older than the last zone edit can be spotted. The
# very same text is built by zonesHash() in frontend/server.js, which is
# how the planner knows the routing data is stale, so the two must stay
# in step: one "penalty" line, then one line per zone sorted by id, with
# every number written to six decimals. A fixed text format rather than
# a hash of the JSON, because Node and Python do not serialise numbers
# the same way. (A rounding disagreement on the last decimal would only
# ever show a spurious "out of date" warning, never a wrong plan.)
zones_hash() {
  zones_query "$1" <<'PY'
import hashlib, json, sys
profile = sys.argv[1]
config = json.load(open("docs/no_go_zones.json", encoding="utf-8"))
penalty = config.get("penalty", {})
rate = penalty.get("rate")
lines = ["penalty speed_kmh=%.6f rate=%s"
         % (float(penalty.get("speed_kmh", 1)),
            "none" if rate is None else "%.6f" % float(rate))]
zones = [z for z in config.get("zones", []) if profile in z.get("blocked_profiles", [])]
for z in sorted(zones, key=lambda z: int(z["id"])):
    ring = " ".join("%.6f,%.6f" % (float(p[0]), float(p[1])) for p in z["polygon"])
    lines.append("zone %s %s" % (z["id"], ring))
text = "\n".join(lines) + "\n"
print(hashlib.sha256(text.encode()).hexdigest()[:16])
PY
}

# Note: assign, then use. Reading the profiles inside a `for` list would
# hide a bad zone file behind an empty list and report "nothing to do".
PROFILES=""
read_profiles() {
  PROFILES="$(restricted_profiles)"
}

# "up to date", "out of date" or "not built".
profile_state() {
  local meta="$DATA_DIR/$1/zones.meta.json"
  if [ ! -f "$meta" ]; then
    echo "not built"
  elif grep -q "\"zones_hash\": \"$(zones_hash "$1")\"" "$meta"; then
    echo "up to date"
  else
    echo "out of date"
  fi
}

status() {
  local any=0
  for profile in $PROFILES; do
    any=1
    case "$(profile_state "$profile")" in
      "not built") echo "  $profile: not built" ;;
      "up to date") echo "  $profile: up to date" ;;
      *) echo "  $profile: out of date (zones changed since the last build)" ;;
    esac
  done
  [ "$any" = 0 ] && echo "  no zone blocks any profile: every vehicle uses the default one"
  return 0
}

# ------------------------------------------------------------ the clip
#
# The segment scan only needs the roads near the zones, so the country
# extract is clipped once to the box around every zone plus a margin,
# and the clip is reused while the zones stay inside it and the extract
# itself is unchanged. osrm-data/zones/region.json remembers what the
# clip covers. OSM node ids are preserved by osmium, so a segment file
# built from the clip applies to the country dataset unchanged.

# Prints "reuse" when the cached clip covers the zones, otherwise
# "clip <left> <bottom> <right> <top>" (with the margin applied).
clip_check() {
  zones_query "$1" "$2" <<'PY'
import json, os, sys
margin = float(sys.argv[1])
pbf = sys.argv[2]
config = json.load(open("docs/no_go_zones.json", encoding="utf-8"))
pts = [tuple(map(float, p)) for z in config.get("zones", []) for p in z.get("polygon", [])]
if not pts:
    sys.exit("no zone to clip around")
lng = [p[0] for p in pts]
lat = [p[1] for p in pts]
box = (min(lng), min(lat), max(lng), max(lat))
st = os.stat(pbf)
try:
    meta = json.load(open("osrm-data/zones/region.json", encoding="utf-8"))
    have = meta["bbox"]
    same_pbf = meta.get("pbf") == os.path.basename(pbf) and \
        meta.get("pbf_size") == st.st_size and int(meta.get("pbf_mtime", -1)) == int(st.st_mtime)
    covered = have[0] <= box[0] and have[1] <= box[1] and have[2] >= box[2] and have[3] >= box[3]
    if same_pbf and covered and os.path.exists("osrm-data/zones/region.osm.pbf"):
        print("reuse")
        sys.exit(0)
except (OSError, ValueError, KeyError, TypeError):
    pass
print("clip %.4f %.4f %.4f %.4f" % (box[0] - margin, box[1] - margin, box[2] + margin, box[3] + margin))
PY
}

clip_record() {
  zones_query "$@" <<'PY'
import json, os, sys, time
pbf = sys.argv[1]
st = os.stat(pbf)
meta = {
    "pbf": os.path.basename(pbf),
    "pbf_size": st.st_size,
    "pbf_mtime": int(st.st_mtime),
    "bbox": [float(v) for v in sys.argv[2:6]],
    "built_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
}
with open("osrm-data/zones/region.json", "w", encoding="utf-8") as f:
    json.dump(meta, f, indent=2)
    f.write("\n")
PY
}

# Makes sure osrm-data/zones/region.osm.pbf covers the zones.
ensure_clip() {
  local check
  check="$(clip_check "$CLIP_MARGIN" "osrm-data/$PBF")"
  if [ "$check" = "reuse" ]; then
    echo "    using the cached extract of the region around the zones"
    return 0
  fi
  # shellcheck disable=SC2086
  set -- $check
  local left="$2" bottom="$3" right="$4" top="$5"
  echo "    clipping the extract to the region around the zones ($left,$bottom .. $right,$top), once"
  build_image
  mkdir -p "$CLIP_DIR"
  docker run --rm -v "${MOUNT}:/work" -w /work --entrypoint osmium "$ZONES_IMAGE" \
    extract --bbox "$left,$bottom,$right,$top" --strategy simple --overwrite \
    -o "osrm-data/zones/region.osm.pbf" "osrm-data/$PBF"
  clip_record "osrm-data/$PBF" "$left" "$bottom" "$right" "$top"
}

# ----------------------------------------------------------- the build

# Hard-links (or, failing that, copies) the base dataset into the
# profile directory, copying the files osrm-customize will rewrite.
# Whatever is there is removed first: a copy over an existing hard link
# would write through it into the base dataset.
link_dataset() {
  local dir="$1" src name dst linked=0 copied=0
  for src in "$DATA_DIR/$BASE.osrm" "$DATA_DIR/$BASE.osrm."*; do
    name="$(basename "$src")"
    dst="$dir/$name"
    rm -f "$dst"
    case " $(echo $CUSTOMIZE_WRITES) " in
      *" ${name#$BASE} "*)
        cp -p "$src" "$dst"
        copied=$((copied + 1))
        ;;
      *)
        if ln "$src" "$dst" 2> /dev/null; then
          linked=$((linked + 1))
        else
          cp -p "$src" "$dst"
          copied=$((copied + 1))
        fi
        ;;
    esac
  done
  echo "    $linked files hard-linked, $copied copied"
}

# The base dataset must come out of a build untouched: with hard links
# in play, a file osrm-customize rewrote that was not in
# CUSTOMIZE_WRITES would have been rewritten in the base too, and every
# vehicle would then be routing on a penalised map. Modification times
# before and after are compared to make that loud.
base_stamp() {
  local f
  for f in "$DATA_DIR/$BASE.osrm" "$DATA_DIR/$BASE.osrm."*; do
    printf '%s %s %s\n' "$(basename "$f")" "$(stat -c %Y "$f")" "$(stat -c %s "$f")"
  done
}

build_profile() {
  local profile="$1"
  local dir="$DATA_DIR/$profile"
  local t0
  t0=$(date +%s)

  echo "==> $profile"
  for f in "$DATA_DIR/$BASE.osrm.mldgr" "$DATA_DIR/$BASE.osrm.partition" "$DATA_DIR/$PBF"; do
    if [ ! -f "$f" ]; then
      echo "    missing $(basename "$f"); build the base data first (frontend/setup.sh)" >&2
      exit 1
    fi
  done

  ensure_clip

  echo "    collecting the road segments inside the zones"
  build_image
  mkdir -p "$dir"
  docker run --rm -i -v "${MOUNT}:/work" -w /work "$ZONES_IMAGE" \
    scripts/zone_segments.py \
    --zones "docs/no_go_zones.json" \
    --profile "$profile" \
    --pbf "osrm-data/zones/region.osm.pbf" \
    --out "osrm-data/$profile/zone_segments.csv"

  echo "    linking the routing data into $dir"
  rm -f "$dir/zones.meta.json"
  link_dataset "$dir"
  local before
  before="$(base_stamp)"

  echo "    applying the penalty (osrm-customize)"
  docker run --rm -v "${MOUNT}/osrm-data:/data" "$OSRM_IMAGE" \
    osrm-customize "/data/$profile/$BASE.osrm" \
    --segment-speed-file "/data/$profile/zone_segments.csv"

  if [ "$(base_stamp)" != "$before" ]; then
    echo "    ERROR: osrm-customize modified the base dataset through a hard link." >&2
    echo "    Rebuild the base data (frontend/setup.sh --rebuild-osrm) and add the" >&2
    echo "    files listed below to CUSTOMIZE_WRITES in $0:" >&2
    diff <(echo "$before") <(base_stamp) | grep '^>' >&2 || true
    exit 1
  fi

  printf '{\n  "profile": "%s",\n  "zones_hash": "%s",\n  "built_at": "%s"\n}\n' \
    "$profile" "$(zones_hash "$profile")" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    > "$dir/zones.meta.json"
  echo "    done in $(( $(date +%s) - t0 )) s: $dir"
}

# ----------------------------------------------------------- serving it

# Waits until the OSRM instance published on a host port answers a
# routing request (a plain TCP accept is not enough: osrm-routed
# listens before its data is loaded).
wait_for_port() {
  local port="$1" i
  for i in $(seq 1 60); do
    if curl -fs -m 5 "http://localhost:$port/nearest/v1/car/-9.15,38.72" > /dev/null 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

# (Re)starts the container serving a profile so that it loads the data
# just built. `up --force-recreate` covers both a container that is not
# running yet and one still holding the previous dataset.
serve_profile() {
  local profile="$1" recreate="$2" service port
  read -r service port <<< "$(profile_service "$profile")"
  if [ "$service" = "-" ]; then
    echo "    $profile has no \"service\" in no_go_zones.json; start its container yourself"
    return 0
  fi
  if [ "$recreate" = 1 ]; then
    echo "    restarting $service with the new data"
    "${COMPOSE[@]}" --profile zones up -d --no-deps --force-recreate "$service"
  else
    echo "    making sure $service is running"
    "${COMPOSE[@]}" --profile zones up -d --no-deps "$service"
  fi
  if [ "$port" = "-" ]; then
    return 0
  fi
  echo "    waiting for $service to answer on port $port"
  if wait_for_port "$port"; then
    echo "    $service answers on port $port"
  else
    echo "    $service does not answer on port $port; see: docker compose logs $service" >&2
    return 1
  fi
}

# vroom-express reads vroom-conf/config.yml, which binds every routing
# profile to its server, once at start-up. A profile added there after
# the solver started makes every plan using it fail with "Invalid
# profile", so the solver is restarted when the file is newer than it.
ensure_solver() {
  local id started started_s conf_s i
  id="$("${COMPOSE[@]}" ps -q vroom 2> /dev/null || true)"
  [ -z "$id" ] && return 0 # not running: frontend/setup.sh starts it
  started="$(docker inspect -f '{{.State.StartedAt}}' "$id")"
  started_s="$(date -d "$started" +%s)"
  conf_s="$(stat -c %Y "$ROOT/vroom-conf/config.yml")"
  if [ "$conf_s" -le "$started_s" ]; then
    return 0
  fi
  echo "==> vroom-conf/config.yml changed since the solver started; restarting vroom"
  "${COMPOSE[@]}" restart vroom
  echo "    waiting for vroom-express on port 3000"
  for i in $(seq 1 30); do
    if curl -fs -m 5 http://localhost:3000/health > /dev/null 2>&1; then
      echo "    vroom-express answers"
      return 0
    fi
    sleep 2
  done
  echo "    vroom-express does not answer; see: docker compose logs vroom" >&2
  return 1
}

MODE=build
case "${1:-}" in
  --status)
    read_profiles
    echo "Zone datasets:"
    status
    exit 0
    ;;
  --image)
    build_image force
    exit 0
    ;;
  --apply)
    MODE=apply
    shift
    ;;
esac

read_profiles

WANTED="${1:-}"
FOUND=0
FAILED=0
for profile in $PROFILES; do
  if [ -n "$WANTED" ] && [ "$profile" != "$WANTED" ]; then
    continue
  fi
  FOUND=1
  if [ "$MODE" = apply ]; then
    state="$(profile_state "$profile")"
    if [ "$state" = "up to date" ]; then
      echo "==> $profile: routing data up to date"
      serve_profile "$profile" 0 || FAILED=1
    else
      echo "==> $profile: routing data $state, building it"
      build_profile "$profile"
      serve_profile "$profile" 1 || FAILED=1
    fi
  else
    build_profile "$profile"
  fi
done

if [ "$FOUND" = 0 ]; then
  if [ -n "$WANTED" ]; then
    echo "No zone blocks profile $WANTED; nothing to build." >&2
    exit 1
  fi
  echo "No zone blocks any profile: nothing to build, every vehicle uses the default one."
  exit 0
fi

if [ "$MODE" = apply ]; then
  ensure_solver || FAILED=1
  if [ "$FAILED" = 1 ]; then
    echo "Some routing instance did not come up." >&2
    exit 1
  fi
  echo
  echo "Zones applied: the routing data is current and served."
else
  echo
  echo "Restart the OSRM containers serving these profiles so they pick up the"
  echo "new cell metrics, e.g.:  scripts/build_zone_graphs.sh --apply"
fi
