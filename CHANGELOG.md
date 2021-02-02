# Changelog

## [Unreleased]

### Added

- Ability to choose ETA and report violations for custom routes using `-c` (#430)
- Custom route description using new `steps` key for a `vehicle` in input (#430)
- A `violations` object is reported in output at `step`, `route` and `summary` level (#430)
- `libglpk` used as an optional dependency, required for `-c` (#430)

### Changed

- Reduce computing time by refactoring `LocalSearch::try_job_additions` (#392)
- Reduce build time by refactoring includes (#425)
- Improve error message with wrong profile using libosrm (#397)
- Check for duplicate ids across tasks of the same type: `job`, `pickup`, `delivery` (#430)
- Check for duplicate ids across `break` tasks for the same vehicle (#430)
- Report `service` and `waiting_time` for all `step` objects in output (#430)
- Always report a `start` and `end` step for the route, regardless of vehicle description (#430)

### Fixed

- Rapidjson assert on invalid syntax for first vehicle (#418)
- Compatibility with Visual Studio 2019 (#448)

## [v1.8.0] - 2020-09-29

### Added

- Local search move removing a job in a route and adding an unassigned one (#324)
- Support for string description of vehicles and tasks (#235)
- Priority sum report in output (#390)

### Changed

- Improved job earliest/latest dates handling internally (#330)

### Fixed

- Break ordering problem (#385)
- Obvious suboptimal solution with many unassigned jobs (#319)
- Documentation mismatch (#361)
- Profile type not checked in input (#355)
- Error reported by UBSan (#371)

## [v1.7.0] - 2020-07-08

### Added

- Support for (multiple) driver breaks at vehicle level (#186)
- Dependency to `libasio`, replacing boost/asio (#325)
- More details on features and workflow in README (#335)

### Changed

- Switch to C++17 (#311)
- Use `std::optional` and drop dependency to boost/optional (#312)
- Refactor routing wrapper classes (#320)

### Deprecated

- Steps `job` key is replaced by `id` for consistency with breaks

### Removed

- Drop boost dependency (#325)

### Fixed

- Erroneous call to TSP code with amount-less shipments (#333)
- Missing propagation of ealiest/latest dates in corner case (#339)

## [v1.6.0] - 2020-02-18

### Added

- Support for pickup and delivery tasks (#274)
- `shipments` array in input (#274)
- Use https for routing requests over port 443 (#289)
- New local search operator for route exchange (#288)

### Changed

- Steps `type` in json output can also have value `pickup` and `delivery` (#274)
- Extended range for valid priority values (#287)
- Use `operator&&` for short-circuit evaluation (#293)
- Earlier local search aborts based on incompatibilities (#281)
- Travis script update (#301)

### Fixed

- Missing valid moves for intra Or-opt (#286)
- Unwanted routing engine request with 1x1 custom matrix (#291)
- Error on single-location `table` request (#295)

## [v1.5.0] - 2019-10-14

### Added

- Support for mixing independent pickups and deliveries (#241)
- `pickup` and `delivery` keys for `job` in input and for `route` and `summary` in output (#262)
- `load` key at `step` level in output (#262)
- `priority` key for jobs to gain some control on which jobs are unassigned (#246)
- `HttpWrapper` class to factor code previously duplicated across routing wrappers (#224)

### Changed

- Speed up solving by 25% for CVRP and up to 30% for VRPTW benchmark instances (#255)
- Update Travis configuration to use Ubuntu Bionic (#260)
- Cut down validity checks time (#266)

### Deprecated

- `amount` key at `job` level in input and at `summary` and `route` level in output (#262)

### Removed

- Clustering heuristics for CVRP (#267)

### Fixed

- Implicit instantiation of undefined template error for macos g++ compiler (#231)
- Parsing vehicle ids as `uint64_t` (#228)
- `osrm::EngineConfig` initialization for use with recent `libosrm` versions (#259)

## [v1.4.0] - 2019-03-26

### Added

- Optional `profile` key for vehicles to allow picking routing profile at query-time (#196)
- `-r` command-line flag for explicit routing engine choice (#196)
- Support for multiple named datasets when using `libosrm` (#181)
- Generic `vroom` namespace and several other specializations (#135)
- Support for OpenRouteService as routing engine (#204)
- Spot more job/vehicle incompatibilities derived from constraints (#201)
- Filter out irrelevant local search neighbourhoods for vehicles with disjoint job candidates (#202)
- Specific status codes by error type (#182)
- Avoid locations duplicates for matrix requests (#200)
- Nearest job route seeding option for VRPTW heuristic (#210)

### Changed

- Refactor to remove duplicate code for heuristic and local search (#176)
- Refactor to enforce naming conventions that are now explicitly stated in `CONTRIBUTING.md` (#135)
- Options `-a` and `-p` can be used to define profile-dependant servers (#196)

### Removed

- `-l` and `-m` command-line flags (#196)

### Fixed

- Missing capacity check for initialization in parallel clustering heuristic (#198)
- Segfault on job empty `time_windows` array (#221)

## [v1.3.0] - 2018-12-10

### Added

- Support for VRPTW (#113)
- CI builds using Travis (#134)
- Adjust solving depending on whether vehicles locations are all identical (#152)
- New local search operator (#164)
- Specific intra-route local search operators (#170)

### Changed

- Update `clang-format` to 6.0 for automatic code formatting (#143)
- Keys `duration` and `arrival` are no longer optional in output and based on matrix values.
- Speed up TSP solving by over 35% on all TSPLIB instances (#142)
- Speed up CVRP solving by over 65%, then another ~8% on all CVRPLIB instances (#146, #147)
- New heuristic for CVRP (#153)
- Take advantage of CVRP speed-up and new heuristic to adjust quality/computing time trade-offs (#167)
- Default exploration level set to 5 (max value)

### Fixed

- Wrong ETA with service time and no start (#148)

## [v1.2.0]

### Added

- Support for multiple vehicles
- Support for multi-dimensional capacity constraints (#74)
- Support for skills to model jobs/vehicles compatibility (#90)
- Support for user-defined matrices (#47)
- New flag `-x` to set the trade-off between computing time and exploration depth (#131)
- Provide ETA at step level in the routes, using optional service time for each job (#101, #103)
- Experimental* support to use `vroom` directly from C++ as a library (#42)
- Automatic code formatting script based on `clang-format` (#66)
- PR template

*: read "functional with no C++ API stability guarantee"

### Changed

- Update `rapidjson` to a patched `v1.1.0` (#128)
- Improve dependency handling (#78)
- Improve compilation time and switch from relative to absolute paths for includes (#108)
- Various refactors (#64, #72, #88, #106)

### Removed

- Drop Boost.Log dependency (#130)

### Fixed

- Memory leak upon `vrp` destruction (#69)
- Prevent overflows with huge costs (#50)
- Infinite loop on TSP edge case (#122)
- Various build warnings and errors with both `gcc` and `clang` (#94, #114)

## [v1.1.0]

### Added

- Support `libosrm` as of v5.4 for faster `table` and `route` queries
  (#34)
- Add contributing guidelines (#56)
- Compile also with `-std=c++11`, useful in some environments (#55)

### Changed

- Internals refactor setting up a scalable data model for future
  features (#44)
- Renamed solution indicators key in json output `solution`->`summary`
- Global cleanup with regard to coding standard (#56)

### Removed

- Drop support for TSPLIB files (#48)
- Clean unused code and heuristics

## [v1.0.0]

### Added

- Support for OSRM v5.*
- Dedicated folder for API documentation

### Changed

- New input and output json API (#30)
- Switch to [lon, lat] for all coordinates (#33)

### Removed

- Drop support for OSRM v4.*
- Flags `-s` and `-e` (see new API)

### Fixed

- Compilation trouble with rapidjson and some types (#31)
- Correct usage display obtained with `-h` (#39)

## [v0.3.1]

### Changed

- Switch to BSD 2-clause license.
- Solving TSPLIB instances does not require the `-t` flag anymore.
- Several components of the local search code can now use
  multi-threading (#26).

### Fixed

- Improve 2-opt operator for symmetric cases (#27).

## [v0.3]

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

## [v0.2]

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

## [v0.1]

### Added

- Solving problems with one vehicle visiting several places a.k.a
  [travelling salesman problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem).
- Support matrix computation using [OSRM](http://project-osrm.org/).
- Solution output to `json` with location ordering, cost of the
  solution and execution details.
- Optional ready-to-use detailed route.
- Optional use of euclidean distance for matrix computation.

