#ifndef CVRP_PRIORITY_REPLACE_H
#define CVRP_PRIORITY_REPLACE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class PriorityReplace : public ls::Operator {
private:
  bool _start_gain_computed{false};
  bool _end_gain_computed{false};
  const Priority _start_priority_gain;
  const Priority _end_priority_gain;
  const unsigned _start_assigned_number;
  const unsigned _end_assigned_number;

protected:
  const Index _u; // Unassigned job to insert.
  const Priority _best_known_priority_gain;
  std::unordered_set<Index>& _unassigned;

  bool replace_start_valid{false};
  bool replace_end_valid{false};

  void compute_gain() override;

  // Compute possible gains depending on whether we replace start or
  // end of route.
  void compute_start_gain();

  void compute_end_gain();

public:
  PriorityReplace(const Input& input,
                  const utils::SolutionState& sol_state,
                  std::unordered_set<Index>& unassigned,
                  RawRoute& s_raw_route,
                  Index s_vehicle,
                  Index s_rank, // last rank (included) when replacing
                                // route start
                  Index t_rank, // first rank when replacing route end
                  Index u,
                  Priority best_known_priority_gain);

  bool is_valid() override;

  void apply() override;

  Priority priority_gain();

  unsigned assigned() const;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;

  std::vector<Index> required_unassigned() const override;
};

} // namespace vroom::cvrp

#endif
