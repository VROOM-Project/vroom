#ifndef TW_ROUTE_H
#define TW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include <iostream> // TODO remove

#include "structures/abstract/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/vehicle.h"

class tw_route {
private:
  // Compute new earliest and latest date for job at job_rank when
  // inserted in route at rank. Only takes into account existing
  // timing constraints for surrounding jobs/start/end, not actual job
  // time-windows (handled after call in is_valid_addition and add).
  duration_t new_earliest_candidate(const input& input,
                                    index_t job_rank,
                                    index_t rank) const;
  duration_t new_latest_candidate(const input& input,
                                  index_t job_rank,
                                  index_t rank) const;

  void fwd_update_earliest_from(const input& input, index_t rank);
  void bwd_update_latest_from(const input& input, index_t rank);
  void fwd_update_earliest_with_TW_from(const input& input, index_t rank);
  void bwd_update_latest_with_TW_from(const input& input, index_t rank);

public:
  index_t vehicle_rank;
  bool has_start;
  bool has_end;
  duration_t v_start;
  duration_t v_end;

  raw_route_t route;
  std::vector<duration_t> earliest;
  std::vector<duration_t> latest;
  std::vector<index_t> tw_ranks;

  tw_route(const input& input, index_t i);

  void log(const input& input) const;

  // Check validity for addition of job at job_rank in current route
  // at rank.
  bool is_valid_addition_for_tw(const input& input,
                                const index_t job_rank,
                                const index_t rank) const;

  // Check validity for inclusion of the range [first_job; last_job)
  // in the existing route at rank first_rank and before last_rank *in
  // place of* the current jobs that may be there.
  template <class InputIterator>
  bool is_valid_addition_for_tw(const input& input,
                                InputIterator first_job,
                                InputIterator last_job,
                                const index_t first_rank,
                                const index_t last_rank) const;

  void add(const input& input, const index_t job_rank, const index_t rank);

  void remove(const input& input, const index_t rank, const unsigned count);

  // Add the range [first_job; last_job) in the existing route at rank
  // first_rank and before last_rank *in place of* the current jobs
  // that may be there.
  template <class InputIterator>
  void replace(const input& input,
               InputIterator first_job,
               InputIterator last_job,
               const index_t first_rank,
               const index_t last_rank);
};

#endif
