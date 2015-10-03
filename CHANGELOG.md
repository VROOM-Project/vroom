# Changelog

## [0.2] - 2015-10-03

### Added

- New loader to handle TSPLIB format, providing support for
  user-defined matrices (#2).
- Dependency on boost.

### Changed

- Switch to boost.asio for http queries handling.
- Simplified matrix implementation.
- Use of -std=c++14 flag.

### Fix

- Socket reading issues (#1).
- Potentially incorrect request for route summary (#5).

## [0.1] - 2015-09-05

### Added

- Solving problems with one vehicle visiting several places a.k.a
  [travelling salesman problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem).
- Support matrix computation using [OSRM](http://project-osrm.org/).
- Solution output to `json` with location ordering, cost of the
  solution and execution details.
- Optional ready-to-use detailed route.
- Optional use of euclidean distance for matrix computation.

