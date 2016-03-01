# Changelog

## [0.3.1] - 2016-03-01

### Changed

- Switch to BSD 2-clause license.
- Solving TSPLIB instances does not require the `-t` flag anymore.
- Several components of the local search code can now use
  multi-threading (#26).

### Fixed

- Improve 2-opt operator for symmetric cases (#27).

## [0.3] - 2016-02-03

### Added

- Compute optimized "open" trips with user-defined start and/or end.
- Special extra handling for asymmetric problems in the local search
  phase.
- New local search operator to improve results in specific asymmetric
  context (e.g. many locations in a dense urban area with lots of
  one-way streets).
- OSRM v4.9.* compatibility.
- Use rapidjson for json i/o (#19)
- Append the `tour` key to the solution in any case.

### Changed

- U-turns enabled when retrieving detailed route geometry from OSRM
  (#10).
- Evolution of the local-search strategy providing lower dispersion in
  solution quality and improving on worst-case solutions (overall
  worst-case on TSPLIB went from +9.56% over the optimal in v0.2 to
  +6.57% in this release).
- Core refactor for undirected graph (#13), tsp structure and tsplib
  loader (#24), heuristics, local search and 2-opt
  implementation. Results in a less intensive memory usage and faster
  computing times (on TSPLIB files, the computing times dropped by
  more than a factor of 2 on average).
- Cleanup verbose output, using Boost.Log for better display (#18).
- Switch to boost::regex for input parsing.

### Fixed

- Wrong output tour size for problems with 2 locations (#16).
- Segfault with explicit matrix in TSPLIB format (#14).
- Invalid syntax for newline at the end of input file (#17).
- Trouble with the regexes used for TSPLIB parsing (#7).
- Incorrect DIMENSION key in TSPLIB format raising stoul exception.
- Segfault for DIMENSION: 1 problem in TSPLIB format (#25).

## [0.2] - 2015-10-03

### Added

- New loader to handle TSPLIB format, providing support for
  user-defined matrices (#2).
- Dependency on boost.

### Changed

- Switch to boost.asio for http queries handling.
- Simplified matrix implementation.
- Use of -std=c++14 flag.

### Fixed

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

