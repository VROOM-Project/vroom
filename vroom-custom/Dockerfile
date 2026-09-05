FROM debian:trixie-slim AS builder

WORKDIR /

RUN apt-get update > /dev/null && apt-get install -y --no-install-recommends \
    ca-certificates \
    git-core \
    build-essential \
    g++ \
    libssl-dev \
    libasio-dev \
    libglpk-dev \
    pkg-config \
    patch \
    > /dev/null && \
    rm -rf /var/lib/apt/lists/*

ARG VROOM_EXPRESS_RELEASE=master
ARG VROOM_REF=07be776fc20b8d6c85d9f78797e38a9f4e44ec44

RUN git clone --branch ${VROOM_EXPRESS_RELEASE} --single-branch https://github.com/VROOM-Project/vroom-express.git
RUN git clone --recurse-submodules https://github.com/VROOM-Project/vroom.git && \
    cd /vroom && \
    git checkout ${VROOM_REF} && \
    git submodule update --init --recursive

COPY route-duration-quadratic.patch /tmp/route-duration-quadratic.patch

RUN cd /vroom && \
    git apply /tmp/route-duration-quadratic.patch && \
    sed -i 's/-O3 /-O3 -DNDEBUG /' /vroom/src/makefile && \
    make -C /vroom/src -j$(nproc)

FROM debian:trixie-slim AS runstage

COPY --from=builder /vroom-express/. /vroom-express
COPY --from=builder /vroom/bin/vroom /usr/local/bin
COPY docker-entrypoint.sh /docker-entrypoint.sh

WORKDIR /vroom-express

RUN apt-get update > /dev/null && \
    apt-get install -y --no-install-recommends \
    nodejs \
    npm \
    libssl3 \
    curl \
    libglpk40 \
    > /dev/null && \
    rm -rf /var/lib/apt/lists/* && \
    npm config set loglevel error && \
    npm install && \
    mkdir /conf

ENV VROOM_DOCKER=osrm \
    VROOM_LOG=/conf

HEALTHCHECK --start-period=10s CMD curl --fail -s http://localhost:3000/health || exit 1

EXPOSE 3000
ENTRYPOINT ["/bin/bash"]
CMD ["/docker-entrypoint.sh"]
