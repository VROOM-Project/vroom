# Changelog

## [Unreleased]

### Added

#### Internals

- Apply heuristics to partial solutions provided in input (#977)
- LOG_LS flag to generate debug info on the internal solving process (#1124)

### Changed

#### Internals

- Bypass matrix request in `plan` mode (#444)
- Refactor heuristics to reduce code duplication (#1181)
- Refactor `Matrix` template class (#1089)
- Refactor to use `std::format` whenever possible (#1081)
- Reduce complexity for recreation process (#1155)
- Refactor `SolutionIndicators` (#1169)
- Remove amount consistency checks in `parse` in favor of upstream checks in `Input` (#1086)
- Reduce code duplication in routing wrappers (#1184)
- Allow passing `path` in `Server` ctor (#1192)

#### CI

- Update GitHub Actions (#1094)
- Speed up OSRM build (#1096)
- Update Ubuntu image to 24.04 (#1080)
- `vroom` workflow uses g++-14 and clang++-18 (#1080)
- `vroom + libosrm` workflow uses g++-13 and clang++-17 (#1080)
- Update clang-format to 18 (#1148)

### Fixed

#### Core solving

- Solution quality regression when using lots of skills (#1193)
- Crash due to wrong delivery values in some validity checks (#1164)
- Crash when input is valid JSON but not an object (#1172)
- Capacity array check consistency (#1086)
- Segfault when using the C++ API with empty vehicles (#1187)

#### Internals

- Iterator type required by `TWRoute::replace` function (#1103)

#### CI

- Wrong compiler used for clang-based OSRM builds (#1098)

#### Routing

- ORS error handling (#1083)

## [v1.14.0] - 2024-01-16

### Added

#### Features

- Support for cost per km for vehicles (#908)
- Support for `max_distance` at vehicle level (#354)
- `MAX_DISTANCE` violation cause in plan mode (#995)
- Recommendation on how to cite in publications (#943)
- Changelog sub-categories (#1018)

#### Core solving

- `PriorityReplace` local search operator (#988)
- Experimental `TSPFix` local search operator (#737)

#### Internals

- Store distance matrices (#956)
- Default radius of 35km for OSRM snapping (#922)
- Support for URL path in host (#966)

### Changed

#### Core solving

- Significant speedup by pruning local search moves (#509)
- Reduce `compute_best_route_split_choice` complexity (#962)
- `Eval::operator<` sorts on cost, then duration (#914)
- Improved `vrptw::PDShift` implementation (#852)
- Refactor heuristics to be able to operate on a subset of jobs and vehicles (#837)
- Account for vehicle/job compatibility in heuristic regrets values (#982)
- Slightly reduce computing times for SWAP* operator (#987)
- Refactor `RouteSplit` operator (#996)

#### Internals

- Switch to C++20 (#851)
- Exposed internal variables to get feature parity for pyvroom (#901)
- Improve some error messages (#848)
- Improved error messages for file-related IO errors (#553)
- Add job id to error message for unreachable step (#946)
- Reserve `vector` capacity whenever possible (#915)
- Distances in output are from internal matrices, not routing requests (#957)
- Remove unused `tw_length` member from `Job` and associated code
- Scale `TimeWindow::length` from `UserDuration` to `Duration` (#1015)

#### Routing

- ORS: (previously) hard-coded `/ors/v2` slug now has to be added to the path using `-a` (#1036)

#### Dependencies

- Submodule and update Rapidjson (#929)
- Update polylineencoder to v2.0.1 (#931)
- Update polylineencoder to v2.0.2 (#1006)
- Update cxxopts to 3.1.1 (#997)

#### CI

- Update GitHub Actions (#857)
- Setup a `clang-tidy` workflow (#789)
- Add running `apt-get update` in CI jobs (#863)
- Update formatting script to use `clang-format` 14 (#894)
- Update gcc to version 12 in CI (#1002)
- Update clang to version 15 in CI (#1022)

### Fixed

#### Core solving

- `max_travel_time` parameter not taken into account in edge case (#884)
- `max_travel_time` not accounted for with vehicle steps in solving mode (#954)
- `max_travel_time` not accounted for in `RouteSplit` (#941)
- Wrong capacity checks in `RouteSplit` (#981)
- Overflow related to scaling default time windows (#1020)

#### Internals

- Internal matrix problem with inconsistent `location_index` and `location` values (#909)
- Silent fail on invalid output file (#553)
- Comparison of index-based and coordinates-based locations (#935)
- Meaningless `location_index` provided in output for break steps (#877)

#### CI

- Address sonarcloud "bugs" reports (#984)
- Address some sonarcloud "code smell" reports (#986)

## [v1.13.0] - 2023-01-31

### Added

- Support for `max_travel_time` at vehicle level (#273)
- Support for vehicle fixed costs (#528)
- Support for cost per hour for vehicles (#732)
- Support for `max_load` constraint at break level (#786)
- `RouteSplit` local search operator (#788)
- Advertise `libvroom` in README and wiki (#42)

### Changed

- Use new struct to evaluate edges internally (#738)
- Use `std::chrono::milliseconds` for `Timeout` value (#728)
- Use `struct` for storing `Coordinates` instead of an `std::array` (#730)
- Refactor `SolutionIndicators` struct (#750)
- Do not duplicate local search for identical heuristic solutions (#750)
- Add message on invalid routing response (#764)
- Consistent exception type on invalid profile (#771)
- Pass zero amount directly instead of its size (#776)
- Add named constants for default threads number and exploration level (#805)
- Refactor `TSP` cost functions (#812)
- CI builds now use clang++ 14 and g++ 11 on Ubuntu 22.04 (#816)
- Refactor `CVRP::solve` and `VRPTW::solve` functions (#818)
- Refactor `CostWrapper` (#828)

### Fixed

- Missing break validity check (#754)
- Unecessary waiting with multiple breaks and shipments (#840)
- Mark `JobAmount` and `JobTime` comparison operators as `const` (#724)
- Update `ssl_send_and_receive` to throw RoutingExceptions (#770)
- Timeout not observed with multiple long heuristics per thread (#792)
- Wrong validity check range in `vrptw::MixedExchange` (#821)
- Underflow in insertion regrets (#831)
- Crash with missing location coordinates and only `costs` custom matrix (#826)

## [v1.12.0] - 2022-05-31

### Added

- `IntraTwoOpt` local search operator (#706)
- `description` key for unassigned tasks in output, if provided (#403)
- `location_index` key for unassigned tasks and each step, if provided (#625)
- Shared target to makefile, for creating Position Independent Code (#617)
- Exposing some internals for Python through compile flags (#640)
- Stats on local search operators use for dev/debug purposes (#658)
- Project can be compiled without routing support to limit dependencies (#676)
- Internal `max_tasks` constraints derived from input to speed up local-search (#648)

### Changed

- Prune local search moves based on TW constraints (#583)
- Prune local search moves based on capacity constraints (#709)
- Refactor exception class (#639)
- CI builds now run against `libosrm` v5.26.0 (#651)
- Reduce computing time on PDPTW benchmarks by around 20% (#559)
- Change Input and parser signature to simplify downstream usage (#665)
- Consider move options in SWAP* that were previously wrongly discarded (#682)
- Use cxxopts as command line parser instead of getopt (#602)
- Change polylineencoder usage to submodule instead of plain header (#686)

### Fixed

- Remove duplicate definition of LocalSearch (#638)
- Move priority check to Job constructor instead of input parser (#628)
- Wrong index values without custom matrix (#683)
- Assignments instead of equality checks in some plan mode assertions (#701)
- Initialization of single-entry matrix (#699)

## [v1.11.0] - 2021-11-19

### Added

- `setup` key for tasks to refine service time modeling (#358)
- `max_tasks` key limiting route size at vehicle level (#421, #566)
- Support for custom cost matrices (#415)
- Number of routes in solution summary (#524)
- Implementation for extended SWAP* local search operator (#507)
- `-l` command-line flag for user-provided timeout (#594)
- Ability to start the search from user-defined solution (#376)
- Github Actions CI (#436)
- Check for libvroom example build in CI (#514)

### Changed

- `vehicle.steps` are now used in solving mode (#606)
- CI builds now run on Ubuntu 20.04 (#455)
- Simplified time window handling logic in TWRoute (#557)

### Removed

- Travis CI builds (#436)
- Exchange local search operator (#507)

### Fixed

- "Infeasible route" error while an existing route plan exists (#506)
- Break omitted with no other time window (#497)
- Biased evaluation in `try_job_additions` (#572)
- Routing error with custom matrix and `-g` (#561)
- Crash on empty custom matrix (#570)
- Properly allow empty skills arrays (#586)
- Restrict `speed_factor` in the range `(0, 5]` (#591)

## [v1.10.0] - 2021-05-06

### Added

- Support for heterogeneous vehicle routing profiles (#394) (#490)
- Optional `speed_factor` key for vehicle-level tuning (#450)
- Support Valhalla as routing engine (#306)
- Report `type` for unassigned tasks in output (#469)

### Changed

- A mix of empty and non-empty `skills` arrays is allowed in input (#460)
- Formatting script updated to use version 10 of clang-format (#452)
- vroom will now read json input from stdin if no other input is specified (#457)
- Clearer error message with invalid json response from http routing request (#471)

### Deprecated

- Top-level `matrix` key should be replaced using the new `matrices` syntax (#450)

### Fixed

- Compatibility with Visual Studio 2019 (#448)
- The pd_shift operation can now insert a shipment at the end of a route (#462)
- Truncated distance value for end step (#463)
- Zero distance value for last break in open-ended route (#475)
- Multi-thread exception handling (#417, #478)

## [v1.9.0] - 2021-03-04

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

