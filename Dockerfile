# Image for the shipeedo fork of VROOM: the vroom binary built from this
# repository, served over HTTP by vroom-express on port 3000.
#
# Drop-in replacement for vroomvrp/vroom-docker: same entrypoint, same
# /conf/config.yml mount, same VROOM_ROUTER variable.

FROM debian:trixie-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      git \
      libasio-dev \
      libglpk-dev \
      libssl-dev \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

ARG VROOM_EXPRESS_RELEASE=v0.12.0
RUN git clone --depth 1 --branch "$VROOM_EXPRESS_RELEASE" \
      https://github.com/VROOM-Project/vroom-express.git /vroom-express

# Build vroom from this checkout (submodules must be checked out).
COPY . /vroom
RUN test -f /vroom/include/cxxopts/include/cxxopts.hpp \
      || (echo "Missing submodules: run 'git submodule update --init'" && exit 1)
RUN make -C /vroom/src -j"$(nproc)" \
    && /vroom/bin/vroom --help > /dev/null

FROM node:24-trixie-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl \
      libglpk40 \
      libssl3t64 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /vroom/bin/vroom /usr/local/bin/vroom
COPY --from=builder /vroom-express /vroom-express

WORKDIR /vroom-express
RUN npm config set loglevel error \
    && npm install --omit=dev --ignore-scripts \
    && mkdir /conf

COPY docker/docker-entrypoint.sh /docker-entrypoint.sh

ENV VROOM_ROUTER=osrm \
    VROOM_LOG=/conf

HEALTHCHECK --start-period=10s CMD curl --fail -s http://localhost:3000/health || exit 1

EXPOSE 3000
ENTRYPOINT ["/bin/bash"]
CMD ["/docker-entrypoint.sh"]
