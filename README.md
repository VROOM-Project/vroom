# Vehicle Routing Open-source Optimization Machine

_Good solutions, fast._

---

## About

VROOM is an open-source optimization engine written in C++17 that aim
at providing good solutions to various real-life [vehicle routing
problems](https://en.wikipedia.org/wiki/Vehicle_routing_problem) (VRP)
within a small computing time.

The project has been initiated by [Verso](https://verso-optim.com/) to
power its [route optimization
API](https://blog.verso-optim.com/category/route-optimization/api/).

## Supported problem types

VROOM can solve several well-known types of vehicle routing problems
(VRP).

- TSP (travelling salesman problem)
- CVRP (capacitated VRP)
- VRPTW (VRP with time windows)
- MDHVRPTW (multi-depot heterogeneous vehicle VRPTW)
- PDPTW (pickup-and-delivery problem with TW)

VROOM can also solve any mix of the above problem types.

## Features

VROOM models a VRP with a description of resources (`vehicles`),
single-location pickup and/or delivery tasks (`jobs`) and
pickup-and-delivery tasks that should happen within the same route
(`shipments`).

### Job and shipment

- Delivery/pickup amounts on arbitrary number of metrics
- Service time windows
- Service duration
- Skills
- Priority

### Vehicle

- Capacity on arbitrary number of metrics
- Skills
- Working hours
- Driver breaks
- Start and end defined on a per-vehicle basis
- Start and end can be different
- Open trip optimization (only start or only end defined)

## Supported routing engines

VROOM works out-of-the-box on top of several open-source routing
engines.

- [OSRM](http://project-osrm.org/)
- [Openrouteservice](https://openrouteservice.org/)

VROOM can also use a custom cost matrix computed from any other
source.

## Getting started

### Demo

- The [demo frontend](http://map.vroom-project.org/) provides a simple
user interface for quick tests.
- The [demo
server](https://github.com/VROOM-Project/vroom/wiki/Demo-server) makes
it easy to send sample optimization requests for testing purposes.

### Setup your own VROOM stack

#### Solving engine

Several options are available to get `vroom` running on command-line.

1. Use
[`vroom-docker`](https://github.com/VROOM-Project/vroom-docker).
2. Build from source following [the wiki
instructions](https://github.com/VROOM-Project/vroom/wiki/Building).

#### Http wrapper

[`vroom-express`](https://github.com/VROOM-Project/vroom-express) is a
simple wrapper to use `vroom` with http requests. It's already bundled
in the `vroom-docker` setup.

#### Use from C++

The project can be used as a library as shown in [this
example](https://github.com/VROOM-Project/vroom/blob/master/libvroom_examples/libvroom.cpp).

### Usage

Refer to [this wiki
page](https://github.com/VROOM-Project/vroom/wiki/Usage)

## Tests

### CI builds

[![Build Status](https://travis-ci.org/VROOM-Project/vroom.svg?branch=master)](https://travis-ci.org/VROOM-Project/vroom)

[Travis builds](https://travis-ci.org/github/VROOM-Project/vroom) are
used to check the build across various compilers and settings.

### Functional tests

Several sets of instances are used.

1. Benchmark instances from papers (see [wiki page with
results](https://github.com/VROOM-Project/vroom/wiki/Benchmarks)).
2. Custom random instances generated to target typical use-cases and
constraints settings.
3. Real-life instances.

Academic and custom benchmarks are heavily used during development for
each new core feature. Every new release is checked against all
benchmarks classes to spot potential regressions with regard to both
solution quality and computing times.
