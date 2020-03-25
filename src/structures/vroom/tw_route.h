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

  // Compute new earliest and latest date for job at job_rank when
  // inserted in route at rank after break_position breaks. Only takes
  // into account existing timing constraints for surrounding
  // jobs/breaks/start/end, not actual job time-windows.
  Duration new_earliest_candidate(const Input& input,
                                  const Index job_rank,
                                  const Index rank,
                                  const Index break_position) const;
  Duration new_latest_candidate(const Input& input,
                                const Index job_rank,
                                const Index rank,
                                const Index break_position) const;

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

  // Compute (potentially negative) time margin for addition of job at
  // job_rank in current route at rank and after "break_position"
  // breaks.
  Margin addition_margin(const Input& input,
                         const Index job_rank,
                         const Index rank,
                         const Index break_position) const;

public:
  Duration v_start;
  Duration v_end;

  // Margin for job at rank i in route: earliest[i] and latest[i]
  // store earliest and latest date, considering we use time window at
  // rank tw_ranks[i] for this job.
  std::vector<Duration> earliest;
  std::vector<Duration> latest;
  std::vector<Index> tw_ranks;

  // A vector with route.size() + 1 elements. breaks_at_rank[i] is the
  // number of breaks that are to be taken right before job at
  // route[i]. breaks_at_rank[route.size()] is the number of breaks
  // right before the end of the route. breaks_counts holds the
  // cumulated values for breaks_at_rank.
  std::vector<unsigned> breaks_at_rank;
  std::vector<unsigned> breaks_counts;

  // Margin for break at rank i in vehicle breaks: break_earliest[i]
  // and break_latest[i] store earliest and latest date, considering
  // we use time window at rank break_tw_ranks[i] for this break.
  std::vector<Duration> break_earliest;
  std::vector<Duration> break_latest;
  std::vector<Index> break_tw_ranks;

  // When a break's earliest date is delayed (resp. latest date is
  // advanced) because of it's time window start (resp. end), then
  // some amount of time before (resp. after) could be used for
  // travel.
  std::vector<Duration> breaks_travel_margin_before;
  std::vector<Duration> breaks_travel_margin_after;

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
