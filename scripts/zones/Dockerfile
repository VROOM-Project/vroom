# Tooling image for scripts/build_zone_graphs.sh: osmium-tool clips the
# country extract down to the region around the zones (once, cached),
# and scripts/zone_segments.py reads that clip and the no-go zones to
# write the segment speed file that penalises the roads inside them.
# Kept apart from the OSRM image, which has neither osmium nor pyosmium
# nor shapely. Debian packages only, so the versions are whatever the
# base image pins and no network install happens at run time. Built on
# demand by scripts/build_zone_graphs.sh.
FROM debian:bookworm-slim

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
       osmium-tool \
       python3 \
       python3-pyosmium \
       python3-shapely \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /work
ENTRYPOINT ["python3"]
