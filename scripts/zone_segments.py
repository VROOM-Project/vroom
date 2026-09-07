#!/usr/bin/env python3
"""Segment speed file for the roads inside the no-go zones of a profile.

Reads an OSM extract and the zones of docs/no_go_zones.json, and writes
the CSV that ``osrm-customize --segment-speed-file`` expects:

    from_osm_node_id,to_osm_node_id,speed_kmh[,rate]

one line per direction of every road segment (a pair of consecutive
nodes of a way) having at least one endpoint inside a zone. Applying it
to a copy of the routing data makes those roads prohibitively slow
without disconnecting them, which is how a "the truck may not drive
through here" area is enforced: VROOM only sees travel times, so the
area has to be expressed in the road graph itself. See
docs/no_go_zones.md.

Run it through scripts/build_zone_graphs.sh rather than directly: it
needs pyosmium and shapely, which live in the small image built from
scripts/zones/Dockerfile.

    zone_segments.py --zones docs/no_go_zones.json --profile chico \
                     --pbf osrm-data/portugal-latest.osm.pbf \
                     --out osrm-data/chico/zone_segments.csv
"""

import argparse
import json
import sys

try:
    import osmium
except ImportError:  # pragma: no cover - depends on the environment
    sys.exit("pyosmium is missing: run this through scripts/build_zone_graphs.sh")

try:
    from shapely.geometry import Point as ShapelyPoint, Polygon
    from shapely.prepared import prep
except ImportError:  # pragma: no cover - depends on the environment
    sys.exit("shapely is missing: run this through scripts/build_zone_graphs.sh")

# Ways carrying these highway values are not roads a truck could drive
# on, so penalising them would only bloat the file.
SKIPPED_HIGHWAYS = {
    "footway",
    "cycleway",
    "path",
    "pedestrian",
    "steps",
    "bridleway",
    "corridor",
    "proposed",
    "construction",
    "elevator",
    "platform",
    "raceway",
}


def zones_of_profile(config, profile):
    """Zones the given profile may not enter."""
    zones = []
    for zone in config.get("zones", []):
        if profile in zone.get("blocked_profiles", []):
            ring = [(float(lng), float(lat)) for lng, lat in zone["polygon"]]
            if len(ring) < 3:
                sys.exit(
                    f"zone {zone.get('id')} ({zone.get('name')}): a polygon "
                    "needs at least three points"
                )
            zones.append((zone, Polygon(ring)))
    return zones


class ZoneSegments(osmium.SimpleHandler):
    """Collect the node pairs of every road with an endpoint in a zone."""

    def __init__(self, zones):
        super().__init__()
        # Bounding boxes first: a point outside every box (the vast
        # majority) is settled with four comparisons.
        self.zones = [
            (poly.bounds, prep(poly)) for _, poly in zones
        ]
        self.segments = []
        self.ways_seen = 0
        self.ways_hit = 0

    def _inside(self, lng, lat):
        for (min_lng, min_lat, max_lng, max_lat), poly in self.zones:
            if min_lng <= lng <= max_lng and min_lat <= lat <= max_lat:
                if poly.contains(ShapelyPoint(lng, lat)):
                    return True
        return False

    def way(self, w):
        highway = w.tags.get("highway")
        if highway is None or highway in SKIPPED_HIGHWAYS:
            return
        self.ways_seen += 1

        try:
            nodes = [(n.ref, n.lon, n.lat) for n in w.nodes]
        except osmium.InvalidLocationError:
            # A way reaching outside the extract: keep the nodes we do
            # have rather than dropping the whole road.
            nodes = []
            for n in w.nodes:
                try:
                    nodes.append((n.ref, n.lon, n.lat))
                except osmium.InvalidLocationError:
                    nodes.append(None)

        inside = [
            None if n is None else self._inside(n[1], n[2]) for n in nodes
        ]
        hit = False
        for i in range(len(nodes) - 1):
            a, b = nodes[i], nodes[i + 1]
            if a is None or b is None:
                continue
            if inside[i] or inside[i + 1]:
                self.segments.append((a[0], b[0]))
                hit = True
        if hit:
            self.ways_hit += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zones", required=True, help="no_go_zones.json")
    parser.add_argument("--profile", required=True, help="profile to build for")
    parser.add_argument("--pbf", required=True, help="OSM extract (.osm.pbf)")
    parser.add_argument("--out", required=True, help="segment speed CSV to write")
    args = parser.parse_args()

    with open(args.zones, encoding="utf-8") as f:
        config = json.load(f)

    penalty = config.get("penalty", {})
    speed = penalty.get("speed_kmh", 1)
    rate = penalty.get("rate")

    zones = zones_of_profile(config, args.profile)
    if not zones:
        sys.exit(f"no zone blocks profile {args.profile}")

    print(f"    zones blocking {args.profile}:")
    for zone, poly in zones:
        min_lng, min_lat, max_lng, max_lat = poly.bounds
        print(
            f"      {zone.get('id')} {zone.get('name', '')} "
            f"[{min_lng:.5f},{min_lat:.5f} .. {max_lng:.5f},{max_lat:.5f}]"
        )

    handler = ZoneSegments(zones)
    # idx: pyosmium keeps the node coordinates in a memory-backed index
    # so that way callbacks see them; flex_mem grows as needed and
    # handles a country extract comfortably.
    handler.apply_file(args.pbf, locations=True, idx="flex_mem")

    with open(args.out, "w", encoding="utf-8", newline="\n") as out:
        for a, b in handler.segments:
            if rate is None:
                out.write(f"{a},{b},{speed}\n")
                out.write(f"{b},{a},{speed}\n")
            else:
                out.write(f"{a},{b},{speed},{rate}\n")
                out.write(f"{b},{a},{speed},{rate}\n")

    print(
        f"    {handler.ways_hit} roads of {handler.ways_seen} touch a zone, "
        f"{2 * len(handler.segments)} directed segments written to {args.out}"
    )
    if not handler.segments:
        sys.exit(
            "no road segment found inside the zones: check the polygons "
            "(longitude first) and that the extract covers them"
        )


if __name__ == "__main__":
    main()
