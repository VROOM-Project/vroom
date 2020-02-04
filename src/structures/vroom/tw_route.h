#ifndef TW_ROUTE_H
#define TW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/raw_route.h"

namespace vroom {

class TWRoute : public RawRoute {
private:
  // Compute new earliest and latest date for job at job_rank when
  // inserted in route at rank. Only takes into account existing
  // timing constraints for surrounding jobs/start/end, not actual job
  // time-windows (handled after call in is_valid_addition and add).
  Duration new_earliest_candidate(const Input& input,
                                  Index job_rank,
                                  Index rank) const;
  Duration new_latest_candidate(const Input& input,
                                Index job_rank,
                                Index rank) const;

  void fwd_update_earliest_from(const Input& input, Index rank);
  void bwd_update_latest_from(const Input& input, Index rank);
  void fwd_update_earliest_with_TW_from(const Input& input, Index rank);
  void bwd_update_latest_with_TW_from(const Input& input, Index rank);

  bool is_fwd_valid_removal(const Input& input,
                            const Index rank,
                            const unsigned count) const;

  bool is_bwd_valid_removal(const Input& input,
                            const Index rank,
                            const unsigned count) const;

public:
  Duration v_start;
  Duration v_end;

  std::vector<Duration> earliest;
  std::vector<Duration> latest;
  std::vector<Index> tw_ranks;

  TWRoute(const Input& input, Index i);

  bool empty() const {
    return route.empty();
  }

  std::size_t size() const {
    return route.size();
  }

  // Check validity for addition of job at job_rank in current route
  // at rank.
  bool is_valid_addition_for_tw(const Input& input,
                                const Index job_rank,
                                const Index rank) const;

  // Check validity for inclusion of the range [first_job; last_job)
  // in the existing route at rank first_rank and before last_rank *in
  // place of* the current jobs that may be there.
  template <class InputIterator>
  bool is_valid_addition_for_tw(const Input& input,
                                InputIterator first_job,
                                InputIterator last_job,
                                const Index first_rank,
                                const Index last_rank) const;

  void add(const Input& input, const Index job_rank, const Index rank);

  // Check validity for removing a set of jobs from current route at
  // rank. Required because removing a job can actually lead to an
  // invalid solution (see #172).
  bool is_valid_removal(const Input& input,
                        const Index rank,
                        const unsigned count) const;

  void remove(const Input& input, const Index rank, const unsigned count);

  // Add the range [first_job; last_job) in the existing route at rank
  // first_rank and before last_rank *in place of* the current jobs
  // that may be there.
  template <class InputIterator>
  void replace(const Input& input,
               InputIterator first_job,
               InputIterator last_job,
               const Index first_rank,
               const Index last_rank);
};

} // namespace vroom

#endif
