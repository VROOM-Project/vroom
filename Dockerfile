FROM ubuntu:18.04 as build
LABEL maintainer="nils@gis-ops.com"

WORKDIR /vroom

RUN apt-get -y update; apt-get install -y \
    build-essential \
		g++ \
    libssl-dev \
		libboost-all-dev \
		pkg-config

COPY . .

RUN make -C /vroom/src

FROM node:12.13.1-alpine

COPY --from=build /vroom/bin/vroom /usr/local/bin/vroom
