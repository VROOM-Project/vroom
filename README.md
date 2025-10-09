# Complex Route Optimization in Milliseconds

_Good solutions, fast._

---

## About

VROOM is an open-source route optimization engine written in C++20
that solves complex [vehicle routing
problems](https://en.wikipedia.org/wiki/Vehicle_routing_problem) (VRP)
in milliseconds.

The project is maintained by [Verso](https://verso-optim.com). If you
want to get started as quickly as possible with route optimization,
you want white-glove support to increase the ROI from your
optimization project and/or you need access to the best possible data
for even more accurate route timing, you should use the [VROOM Premium
API](https://verso-optim.com/api/).

## Supported problem types

VROOM solves several well-known types of vehicle routing problems
(VRP).

- TSP (travelling salesman problem)
- CVRP (capacitated VRP)
- VRPTW (VRP with time windows)
- MDHVRPTW (multi-depot heterogeneous vehicle VRPTW)
- PDPTW (pickup-and-delivery problem with TW)

VROOM solves all of the above routing problems at the same time — and
delivers the optimized route in milliseconds, even when complex
variables are involved.

## How it works

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
- [Valhalla](https://github.com/valhalla/valhalla)

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

1. Build from source following [the wiki
instructions](https://github.com/VROOM-Project/vroom/wiki/Building).
2. Use
[`vroom-docker`](https://github.com/VROOM-Project/vroom-docker).

### Command-line usage

Refer to [this wiki
page](https://github.com/VROOM-Project/vroom/wiki/Usage)

#### Http wrapper

[`vroom-express`](https://github.com/VROOM-Project/vroom-express) is a
simple wrapper to use `vroom` with http requests. It's already bundled
in the `vroom-docker` setup.

#### Using libvroom from C++

The project can also used as a library from any C++ project, refer to
[this wiki
page](https://github.com/VROOM-Project/vroom/wiki/Using-libvroom).

## Tests

### CI builds

[![vroom](https://github.com/VROOM-Project/vroom/actions/workflows/vroom.yml/badge.svg)](https://github.com/VROOM-Project/vroom/actions/workflows/vroom.yml)

[![vroom + libosrm](https://github.com/VROOM-Project/vroom/actions/workflows/vroom_libosrm.yml/badge.svg?branch=master)](https://github.com/VROOM-Project/vroom/actions/workflows/vroom_libosrm.yml)

[Github Actions](https://github.com/VROOM-Project/vroom/actions) are
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

## Reference in publications

To cite VROOM in publications, please use:

```bibtex
@manual{vroom_v1.14,
   title = {{VROOM v1.14, Vehicle Routing Open-source Optimization Machine}},
   author = {Coupey, Julien and Nicod, Jean-Marc and Varnier, Christophe},
   year = 2024,
   organization = {Verso (\url{https://verso-optim.com/})},
   address = {Besançon, France},
   note = {\url{http://vroom-project.org/}}
 }
 ```
