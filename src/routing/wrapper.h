#ifndef WRAPPER_H
#define WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "structures/generic/matrix.h"
#include "structures/vroom/location.h"
#include "structures/vroom/matrices.h"
#include "structures/vroom/solution/route.h"
#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

namespace vroom::routing {

class Wrapper {

public:
  std::string profile;

  virtual Matrices get_matrices(const std::vector<Location>& locs) const = 0;

  Matrices
  get_sparse_matrices(const std::vector<Location>& locs,
                      const std::vector<Vehicle>& vehicles,
                      const std::vector<Job>& jobs,
                      std::vector<std::string>& vehicles_geometry) const {
    const std::size_t m_size = locs.size();
    Matrices m(m_size);

    std::mutex matrix_m;

    auto run_on_vehicle_at_rank =
      [this, &vehicles, &jobs, &matrix_m, &m, &vehicles_geometry](
        Index v_rank) {
        try {
          const Vehicle& v = vehicles[v_rank];

          std::vector<Location> route_locs;
          route_locs.reserve(v.steps.size());

          bool has_job_steps = false;
          for (const auto& step : v.steps) {
            switch (step.type) {
              using enum STEP_TYPE;
            case START:
              if (v.has_start()) {
                route_locs.push_back(v.start.value());
              }
              break;
            case END:
              if (v.has_end()) {
                route_locs.push_back(v.end.value());
              }
              break;
            case BREAK:
              break;
            case JOB:
              has_job_steps = true;
              route_locs.push_back(jobs[step.rank].location);
              break;
            }
          }

          if (has_job_steps) {
            assert(route_locs.size() >= 2);

            this->update_sparse_matrix(route_locs,
                                       m,
                                       matrix_m,
                                       vehicles_geometry[v_rank]);
          }
        } catch (const RoutingException& e) {
          std::cerr << "[Warning] Sparse matrix routing failed for vehicle "
                    << v_rank << ": " << e.message
                    << ". Using fallback values." << std::endl;
          fill_sparse_fallback(vehicles[v_rank], jobs, m, matrix_m);
        } catch (const std::exception& e) {
          std::cerr << "[Warning] Routing error for vehicle "
                    << v_rank << ": " << e.what()
                    << ". Using fallback values." << std::endl;
          fill_sparse_fallback(vehicles[v_rank], jobs, m, matrix_m);
        }
      };

    std::vector<std::thread> vehicles_threads;
    vehicles_threads.reserve(vehicles.size());

    for (Index v_rank = 0; v_rank < vehicles.size(); ++v_rank) {
      if (vehicles[v_rank].profile == this->profile) {
        vehicles_threads.emplace_back(run_on_vehicle_at_rank, v_rank);
      }
    }

    for (auto& t : vehicles_threads) {
      t.join();
    }

    return m;
  };

  // Updates matrices with data from a single route request and stores
  // corresponding route geometry.
  virtual void update_sparse_matrix(const std::vector<Location>& route_locs,
                                    Matrices& m,
                                    std::mutex& matrix_m,
                                    std::string& vehicle_geometry) const = 0;

  virtual void add_geometry(Route& route) const = 0;

  virtual ~Wrapper() = default;

protected:
  explicit Wrapper(std::string profile) : profile(std::move(profile)) {
  }

  static void fill_sparse_fallback(const Vehicle& v,
                                   const std::vector<Job>& jobs,
                                   Matrices& m,
                                   std::mutex& matrix_m) {
    std::vector<Location> route_locs;
    for (const auto& step : v.steps) {
      if (step.type == STEP_TYPE::START && v.has_start()) {
        route_locs.push_back(v.start.value());
      } else if (step.type == STEP_TYPE::END && v.has_end()) {
        route_locs.push_back(v.end.value());
      } else if (step.type == STEP_TYPE::JOB) {
        route_locs.push_back(jobs[step.rank].location);
      }
    }
    const std::scoped_lock<std::mutex> lock(matrix_m);
    for (std::size_t i = 0; i + 1 < route_locs.size(); ++i) {
      m.durations[route_locs[i].index()][route_locs[i + 1].index()] =
        UNFOUND_ROUTE_DURATION;
      m.distances[route_locs[i].index()][route_locs[i + 1].index()] =
        UNFOUND_ROUTE_DISTANCE;
    }
  }

  static void warn_unfound(const std::vector<Location>& locs,
                           const std::vector<unsigned>& nb_unfound_from_loc,
                           const std::vector<unsigned>& nb_unfound_to_loc) {
    assert(nb_unfound_from_loc.size() == nb_unfound_to_loc.size());
    unsigned total_unfound = 0;
    unsigned worst_loc = 0;
    unsigned worst_count = 0;
    std::string worst_direction;
    for (unsigned i = 0; i < nb_unfound_from_loc.size(); ++i) {
      total_unfound += nb_unfound_from_loc[i];
      if (nb_unfound_from_loc[i] > worst_count) {
        worst_count = nb_unfound_from_loc[i];
        worst_loc = i;
        worst_direction = "from ";
      }
      if (nb_unfound_to_loc[i] > worst_count) {
        worst_count = nb_unfound_to_loc[i];
        worst_loc = i;
        worst_direction = "to ";
      }
    }
    if (total_unfound > 0) {
      std::cerr << "[Warning] " << total_unfound
                << " unfound route(s) in matrix, worst: "
                << worst_direction
                << std::format("location [{:.6f},{:.6f}]",
                               locs[worst_loc].lon(),
                               locs[worst_loc].lat())
                << ". Using fallback values for unreachable pairs."
                << std::endl;
    }
  }
};

} // namespace vroom::routing

#endif
