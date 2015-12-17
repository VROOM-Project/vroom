# Vehicle Routing Open-source Optimization Machine

_Good solutions, fast._

---

## Overview

### What?

VROOM is an optimization engine written in C++14 that aim at providing
good solutions to various real-life [vehicle routing
problems](https://en.wikipedia.org/wiki/Vehicle_routing_problem)
within a small computing time. It is free software, distributed under
the term of the GNU General Public License V3.

### How?

VROOM is an optimization layer working on top of virtually any tool
handling basic routing queries. The idea is to carefully select and
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

### How fast and how good?

Solving problems is a matter of milliseconds, or seconds when reaching
hundreds of locations.

While the [NP-hardness](https://en.wikipedia.org/wiki/NP-hardness) of
the problem makes it very difficult (or too long) to get the optimal
solution, VROOM solutions are shown to be very good approximations
(see [benchmark
results](https://github.com/jcoupey/vroom/wiki/Benchmarks )).

---

## Features

* use actual road topography from OpenStreetMap data (no inadequate
  beeline distance estimate);

* benefits from all the routing options of the underlying tool for the
  profile (car, bicycle, walking...) and for the optimization target
  (travel time, distance, cost...);

* return a ready-to-use solution with quality indicators and a
  detailed route;

* at the moment, solves problems with one vehicle visiting several
  places a.k.a [travelling salesman
  problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem);

* can scale to handle very big instances (don't worry about using a
  few thousands locations);

* support user-defined matrices for travel costs using
  [TSPLIB](http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/)
  format.

---

### Gallery

See animated examples in the [optimized trips
gallery](http://coupey.fr/vroom/gallery).