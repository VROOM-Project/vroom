#ifndef TSP_LOCAL_SEARCH_H
#define TSP_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {
namespace tsp {

class LocalSearch {
private:
  const Matrix<Cost>& _matrix;
  const std::pair<bool, Index> _avoid_start_relocate;
  std::vector<Index> _edges;
  unsigned _nb_threads;
  std::vector<Index> _rank_limits;
  std::vector<Index> _sym_two_opt_rank_limits;

public:
  LocalSearch(const Matrix<Cost>& matrix,
              std::pair<bool, Index> avoid_start_relocate,
              const std::list<Index>& tour,
              unsigned nb_threads);

  Cost relocate_step();

  Cost perform_all_relocate_steps(const Deadline& deadline);

  Cost avoid_loop_step();

  Cost perform_all_avoid_loop_steps(const Deadline& deadline);

  Cost two_opt_step();

  Cost asym_two_opt_step();

  Cost perform_all_two_opt_steps(const Deadline& deadline);

  Cost perform_all_asym_two_opt_steps(const Deadline& deadline);

  Cost or_opt_step();

  Cost perform_all_or_opt_steps(const Deadline& deadline);

  std::list<Index> get_tour(Index first_index) const;
};

} // namespace tsp
} // namespace vroom

#endif
