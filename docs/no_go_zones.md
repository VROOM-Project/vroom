# No-go zones: areas some trucks may not drive through

How the waste transport planner keeps certain vehicles out of certain
areas — today the trucks going out with a chico, tomorrow whatever else
the company needs. Companion to
[waste_transport_problem.md](./waste_transport_problem.md) and
[waste_transport_vroom_analysis.md](./waste_transport_vroom_analysis.md).

Contents:
- [Why this is not a solver constraint](#why-this-is-not-a-solver-constraint)
- [How it works](#how-it-works)
- [The zone file](#the-zone-file)
- [Drawing a zone](#drawing-a-zone)
- [Building the routing data](#building-the-routing-data)
- [Checking that it works](#checking-that-it-works)
- [Limits](#limits)
- [Restricting another kind of vehicle](#restricting-another-kind-of-vehicle)

## Why this is not a solver constraint

VROOM never sees a map. It sees a matrix of travel times and distances
between the locations of the problem, obtained from OSRM, and — with
`-g` — a polyline it asks OSRM to draw afterwards. Between two stops it
has no representation of the path at all: no roads, no geometry,
nothing to test a polygon against.

So "this truck may not drive through here" cannot be expressed in the
solver, at any price. It is a property of the road graph, and that is
where it is enforced. Nothing in `src/` changed for this feature.

What VROOM does provide is the hook: every vehicle has a `profile`, and
VROOM builds one routing wrapper per profile, each with its own host and
port (`src/routing/wrapper.h`, the `-a` and `-p` flags, and
`routingServers` in [vroom-conf/config.yml](../vroom-conf/config.yml)).
Give the restricted vehicles a profile of their own, point it at an OSRM
instance whose roads inside the zones are unusable, and their travel
times, their chosen order of stops and their drawn route all avoid those
areas by construction.

## How it works

```
docs/no_go_zones.json          the areas and the profiles they are closed to
  |
  |  scripts/build_zone_graphs.sh --apply   (the Zones tab's Apply button)
  |    1. osmium: clip the country extract to the region around the
  |       zones, once, cached under osrm-data/zones/
  |    2. scripts/zone_segments.py: which road segments of that clip
  |       are inside a zone
  |    3. hard-link osrm-data/portugal-latest.osrm* into
  |       osrm-data/<profile>/, copying the files customize rewrites
  |    4. osrm-customize --segment-speed-file: those roads become unusable
  |    5. docker compose up --force-recreate the profile's service,
  |       wait until it answers
  v
osrm-data/chico/               a routing dataset where the zones are unusable
  |
  |  the osrm-chico container serves it on port 5001
  v
vroom-conf/config.yml          profile "chico" -> that container
  |
  v
frontend/public/waste_model.js gives chico vehicles profile "chico"
```

Four details worth knowing:

- **Prohibitively slow, not removed.** The roads inside a zone keep
  their place in the graph but are overridden to 1 km/h (and a matching
  weight `rate`), so crossing 500 m of zone costs half an hour and no
  route ever chooses it. Removing them outright would be stronger, but a
  client whose only access road passes through a zone would then have no
  route at all, and OSRM returning no route makes the whole plan fail
  rather than that one operation. The speed and the rate are in the
  `penalty` block of the zone file. See [Limits](#limits).
- **Only `osrm-customize` is re-run.** The dataset is MLD, and its
  partition does not depend on the segment speeds, so a zone change
  costs a customize (a minute or two) and not a full extraction (hours).
- **The country is not scanned for every edit.** Finding the roads
  inside the zones only needs the map around them, so the extract is
  clipped once to a box around every zone plus a 5 km margin and the
  clip is kept in `osrm-data/zones/` until a zone is drawn outside it
  (or the extract itself changes). OSM node ids survive the clip, so
  the segment file it yields applies to the full dataset. Most of the
  dataset is hard-linked into the profile directory rather than copied:
  only the files `osrm-customize` rewrites (edge weights, cell metrics)
  are real copies, about 500 MB per profile instead of 1.3 GB. The
  build script checks afterwards that the base dataset is untouched.
- **Operations inside a zone are excluded outright.** A client inside an
  area closed to the chicos cannot be served by one at all, so the
  planner does not merely make it expensive: each zone becomes a VROOM
  skill, carried by the vehicles allowed in and required by the
  operations sitting inside. A chico is then never even considered for
  them. The planner says so as a warning; if *no* truck of the fleet that
  can carry the container is allowed in, it is an error instead.

## The zone file

[no_go_zones.json](./no_go_zones.json) is the third source of truth,
next to [waste_rules.json](./waste_rules.json) (what the trucks may
carry) and [waste_defaults.json](./waste_defaults.json) (what the
planner may change day to day). It holds:

| Key | What it is |
| --- | --- |
| `penalty` | how a road inside a zone is made unusable: `speed_kmh` and the weight `rate` |
| `profiles` | one entry per OSRM instance: the default `car` plus one per restricted profile, with the compose `service` serving it and the `host_port` it is published on. Names are letters only (see below) |
| `vehicle_profiles` | which vehicle configuration uses which profile: ordered `rules` with a `match` selector, plus a `default` |
| `zones` | the areas: `id`, `name`, `blocked_profiles`, `polygon` |

A `polygon` is a single ring of `[longitude, latitude]` pairs — the same
order as every VROOM location — closed implicitly and without holes.
Selectors available in `vehicle_profiles.rules` are `chico` (any vehicle
with a chico), `chico:<key>` (one chico type of `waste_rules.json`) and
`truck:<type>` (a truck type, with or without a chico); the first
matching rule wins.

A profile name is **letters only**: VROOM puts it in the OSRM request
URL (`/table/v1/<profile>/...`) and OSRM's URL grammar accepts nothing
else there, so `car_chico` fails at solve time with `URL string
malformed`. The build script refuses such a name up front.

Unlike the Config tab of the planner, whose edits live in the browser
only, this file is **written by the planner**: the areas have to reach
the build script, which runs server-side. `frontend/server.js` replaces
the `zones` array and leaves the rest of the file (and its comments)
alone.

With `"zones": []` — how the repository ships — the whole feature is
inert: every vehicle uses the default profile, no second OSRM instance
is needed and the request is exactly what it was before zones existed.

## Drawing a zone

In the planner's **Zones** tab: *Draw a zone*, click the corners on the
map, *Finish* (or `Enter`, or double-click the last corner; `Escape`
cancels). Then tick which vehicles the area is closed to, and rename it
if you like. Every change is saved to `docs/no_go_zones.json`
immediately, so it is shared with everyone using that planner.

To reshape an area, delete it and draw it again — there is no vertex
editing.

The box at the top of the tab says whether the routing data behind each
restricted profile is built, whether it matches the areas as they are
now, and whether its container is answering. When it is not, press
**Apply the areas to the routing data**: the planner's server runs the
build script (below) and shows its output in the tab; a minute or two
later the box is green and the plan can be made. Until then *Plan the
day* refuses to run for a fleet whose vehicles use that profile, since
the plan would either fail (no routing instance) or silently avoid the
previous areas (stale data).

## Building the routing data

The Apply button runs, from the repository root:

```bash
scripts/build_zone_graphs.sh --apply
```

which rebuilds the data of every restricted profile that is missing or
older than the zones, (re)starts the compose service named in the
profile's entry of the zone file, and waits until it answers on its
host port. `frontend/setup.sh` runs the same command as part of the
normal start-up, and it can be run by hand at any time, with a profile
name to limit it to that one. Other modes:

```bash
scripts/build_zone_graphs.sh --status
```

```bash
scripts/build_zone_graphs.sh chico
```

`--status` says what is built and whether it is stale; a bare profile
name (or no argument, for all of them) rebuilds unconditionally without
touching the containers. The script needs the base dataset to exist
first, and a small tooling image built from
[scripts/zones/Dockerfile](../scripts/zones/Dockerfile) (Debian,
`osmium-tool`, `python3-pyosmium` and `python3-shapely`) which it builds
on demand.

The `osrm-chico` service sits in the compose profile `zones`, so a plain
`docker compose up` neither starts it nor fails when its data does not
exist; `--apply` starts it with `--profile zones`.

Costs: about **500 MB of disk per restricted profile** (the files
`osrm-customize` rewrites, the rest of `osrm-data/<profile>/` being
hard links to the base dataset), a few MB for the clipped extract, a
minute or two per build, and one container. The first build also clips
the extract, which takes about as long again.

The planner's server (`frontend/server.js`) runs the script through
bash. On Windows it looks for Git's `bash.exe` in the usual install
locations rather than the one on `PATH`, which may be WSL's; set
`ZONES_BASH` to the right one if it is installed somewhere else.

One thing to know about `vroom-conf/config.yml`: the vroom-express
container seeds it from its own default on first run, and that default
only knows the plain `car` profile. It is therefore **tracked in the
repository** (only the `access.log` next to it is ignored), so that the
zone profiles come with a checkout. A vehicle asking for a profile that
is not declared there makes the solver answer `Invalid profile:
chico.` — that error means this file, not the zones. vroom-express
reads it once, at start-up, so the same error appears when the file
gained a profile after the container started; `--apply` compares the
two times and restarts the solver when needed.

## Checking that it works

Two independent checks, both worth running the first time:

**The detour exists.** Ask both instances for a route that would
naturally cross a zone, and compare:

```bash
curl -s "http://localhost:5000/route/v1/car/-7.99,37.03;-7.96,37.04?overview=false" | head -c 200
```

```bash
curl -s "http://localhost:5001/route/v1/car/-7.99,37.03;-7.96,37.04?overview=false" | head -c 200
```

The second should come back with a visibly longer duration, or the same
one if the straight path never entered the zone. (The `car` in the URL
is OSRM's own path segment and has nothing to do with the VROOM profile
name: each instance serves the dataset it was started with.)

**The plan stays out.** The post-check reads the zones and verifies that
no route of a restricted vehicle stops in, or drives through, an area
closed to it — using the route geometry, so run the solver with `-g`:

```bash
python scripts/waste_solution_check.py input.json out.json docs/no_go_zones.json
```

It tests the geometry against the zone edges, not only its vertices, so
a straight leg cutting a corner of a small area is caught too.

## Limits

- **The penalty is not a proof.** Where a zone is genuinely unavoidable,
  the route goes through it at 1 km/h rather than failing. That is a
  deliberate trade (see [How it works](#how-it-works)); the post-check
  above is what turns it into a visible fact instead of a silent one.
  Lower `penalty.speed_kmh` to push harder.
- **Stale data is possible.** The plan uses whatever the OSRM instance
  was built with, which is why the Zones tab compares a fingerprint of
  the current areas against the one stored in
  `osrm-data/<profile>/zones.meta.json`, says when they differ, and
  refuses to plan with a profile in that state. Nothing stops a request
  sent to vroom-express directly, though.
- **A zone cannot be closed to the default profile.** That profile is
  served by the untouched dataset, so it would change nothing; the file
  check, the planner and the build script all refuse it. Give the
  vehicles that must avoid the area a profile of their own instead.
- **One instance per restricted profile.** Zones are shared by the
  multiban and the poliban chicos today, which is why there is one extra
  profile and not two.
- **Zones apply to the whole day.** There is no notion of an area that
  is closed only at certain hours.

## Restricting another kind of vehicle

Only chicos are restricted today, but nothing in the mechanism is about
chicos. To keep, say, polibans out of a historic centre:

1. add a profile to `no_go_zones.json`:
   `"poliban": { "host_port": 5002, "service": "osrm-poliban", "description": "poliban trucks" }`;
2. add a rule before the chico one if it should win, or after:
   `{ "match": "truck:poliban", "profile": "poliban" }`;
3. add a service `osrm-poliban` to
   [docker-compose.yml](../docker-compose.yml) modelled on `osrm-chico`,
   serving `/data/poliban/portugal-latest.osrm` on port 5002, in the
   `zones` compose profile;
4. declare it in [vroom-conf/config.yml](../vroom-conf/config.yml) with
   host `osrm-poliban` and port 5000 (the port *inside* the compose
   network, not the published one);
5. draw the area in the planner, tick the new profile, and Apply.

Steps 1, 2 and 5 are the model; 3 and 4 are the price of another road
graph. Note that a vehicle uses exactly one profile, so a truck that
must respect two different sets of areas needs a profile whose dataset
penalises both.
