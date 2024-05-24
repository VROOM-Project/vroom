#ifndef TSP_H
#define TSP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrp.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {

using RawSolution = std::vector<RawRoute>;

class TSP : public VRP {
private:
  const Index _vehicle_rank;
  // Holds the matching from index in _matrix to rank in input::_jobs.
  const std::vector<Index> _job_ranks;
  bool _is_symmetric{true};
  const bool _has_start;
  Index _start;
  const bool _has_end;
  Index _end;
  Matrix<UserCost> _matrix;
  Matrix<UserCost> _symmetrized_matrix;
  bool _round_trip;

  UserCost cost(const std::list<Index>& tour) const;

  UserCost symmetrized_cost(const std::list<Index>& tour) const;

public:
  TSP(const Input& input, std::vector<Index>&& job_ranks, Index vehicle_rank);

  std::vector<Index> raw_solve(unsigned nb_threads,
                               const Timeout& timeout) const;

  Solution solve(unsigned,
                 unsigned,
                 unsigned nb_threads,
                 const Timeout& timeout,
                 const std::vector<HeuristicParameters>&) const override;
};

} // namespace vroom

#endif
