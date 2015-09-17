# Vehicle Routing Open-source Optimization Machine

_Good solutions, fast._

[Overview](#overview)

[Features](#features)

[Examples](#examples)

[Installation and usage](#installation-and-usage)

[Informal Roadmap](#informal-roadmap)

---

## Overview

### What?

VROOM is an optimization engine written in C++ that aim at providing
good solutions to various real-life [vehicle routing
problems](https://en.wikipedia.org/wiki/Vehicle_routing_problem)
within a small computing time. It is free software, distributed under
the term of the GNU General Public License V3.

### How?

VROOM is an optimization layer working on top of virtually any tool
handling basic routing queries.

It is very much a work in progress but is already a good proof of
concept of what can be achieved in term of efficiency on real-life
vehicle routing problems. The idea is to carefully select and
implement strategies derived from academic research, targeting the
choices with quality and computing times in mind.

### Why?

Even if the problems are simple to state, their [combinatorial
complexity](https://en.wikipedia.org/wiki/Combinatorial_optimization)
make them very hard to solve. Finding optimal solutions is usually
beyond reach for real-life problem instances, hence the need of robust
approximation algorithms finding good solutions in reasonable time.

Besides, loads of researches have been carried out in that area but
they usually remain academic "theoretical" discussions on the possible
strategies. With the current rise of OpenStreetMap and efficient
open-source routing tools, it's now time to go one step further with
real-life open-source routing optimization!

---

## Features

* use actual road topography from OpenStreetMap data (no inadequate
  beeline distance estimate);

* benefits from all the routing options of the underlying tool for the
  profile (car, bicycle, walking...) and for the optimization target
  (travel time, distance, cost...);

* currently defaults to [OSRM](http://project-osrm.org/) to perform
  efficient travel time matrix computation;

* return a ready-to-use solution with quality indicators and a
  detailed route;

* at the moment, solves problems with one vehicle visiting several
  places a.k.a [travelling salesman
  problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem);

* can scale to handle very big instances (don't worry about using
  several thousands locations).

---

## Examples

### French towns

A tour of the 244 biggest towns in metropolitan France, by car,
computed in about 1.2s. [Browse on umap](http://u.osmfr.org/m/51654/).

![French towns](http://coupey.fr/vroom/french_towns.png)

Travel informations:

* total time: 118.5 hours;

* total distance: 9602.5km.

Tour computing time:

* compute and check travel time matrix: 530ms;

* tour ordering: 88ms;

* detailed route geometry: 556ms.

Download:

* [request](http://coupey.fr/vroom/french_towns.req)

* [output](http://coupey.fr/vroom/french_towns_solution.json)


### Irish pubs

Bike trip around 2232 irish pubs, computed in about 45s. [Browse on
umap](http://u.osmfr.org/m/51914).

![Irish pubs](http://coupey.fr/vroom/irish_pubs.png)

Travel informations:

* total time: ~28 days;

* total distance: 9467km.

Tour computing time:

* compute and check travel time matrix: 7.5s;

* tour ordering: 35s;

* detailed route geometry: 2.3s.

Download:

* [request](http://coupey.fr/vroom/irish_pubs.req)

* [output](http://coupey.fr/vroom/irish_pubs_solution.json)

---

## Installation and usage

### Build

To clone the project and build the executable using gcc 4.7 or later,
run:

```bash
git clone https://github.com/jcoupey/vroom.git
cd vroom
mkdir bin && cd src/
make
cd ../
```

### Command-line usage

If your OSRM server is listening at 1.2.3.4 on port 4321 (see the
[OSRM wiki](https://github.com/Project-OSRM/osrm-backend/wiki) on how
to setup your own server), use the following with your list of
locations as argument:

```bash
./bin/vroom -a 1.2.3.4 -p 4321 "loc=lat,lon&loc=lat,lon[&loc=lat,lon...]"
```

If you run OSRM locally with defaults value (0.0.0.0 on port 5000),
you don't need to specify anything, as shown in the complete usage and
options list:

```bash
$ vroom -h
VROOM Copyright (C) 2015, Julien Coupey
Usage :
	vroom [OPTION]... "loc=lat,lon&loc=lat,lon[&loc=lat,lon...]"
	vroom [OPTION]... -i FILE
Options:
	-a=ADDRESS	 OSRM server address ("0.0.0.0")
	-p=PORT,	 OSRM listening port (5000)
	-g,		     get detailed route geometry for the solution
	-e,		     use eulidean distance rather than travel times
			     from OSRM
	-o=OUTPUT,	 output file name
	-i=FILE,	 read locations list from FILE rather than from
			     command-line
	-v,		     turn on verbose output

This program is distributed under the terms of the GNU General Public
License, version 3, and comes with ABSOLUTELY NO WARRANTY.

```

### Output

The solution is described in `json` format.

* `route`: array containing the ordered list of locations;

* `total_time`: total travel time in seconds;

* `total_distance`: total distance in meters;

* `route_geometry`: detailed route compressed as
  [polyline](http://code.google.com/apis/maps/documentation/utilities/polylinealgorithm.html);

* `solution_cost`: cost of the found solution, estimated from the cost
  matrix used while solving;

* `computing_times`

    * `matrix_loading`: time required to compute the cost matrix;

    * `route`

        * `heuristic`: time required to run the solving heuristic;

        * `local_search`: time used to improve the initial solution;

    * `detailed_geometry`: time required to compute `route_geometry`,
      `total_time` and `total_distance`.

If you are familiar with the [OSRM API for viaroute
queries](https://github.com/Project-OSRM/osrm-backend/wiki/Server-api#service-viaroute),
you already guessed that `route_geometry`, `total_time` and
`total_distance` are actually obtained with a viaroute query on the
`route` array once the solution is found.

---

## Informal Roadmap

Miscellaneous interesting perspectives:

* setup a demo server somewhere;

* give better quality indicators using a good lower bound on solution
  cost;

* benchmark the approach on
  [TSPLIB instances](http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/),
  both for quality and computing times;
  
* use clustering algorithms (or other options) to handle problems
  involving several vehicles;

* setup a testing environment for the components and algorithms;

* offer out-of-the box use for other routing tools.


