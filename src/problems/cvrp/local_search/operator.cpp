/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "operator.h"

std::vector<std::vector<amount_t>> ls_operator::fwd_amounts;
std::vector<std::vector<amount_t>> ls_operator::bwd_amounts;
std::vector<std::vector<gain_t>> ls_operator::edge_costs_around_node;
std::vector<std::vector<gain_t>> ls_operator::node_gains;
std::vector<index_t> ls_operator::node_candidates;
std::vector<std::vector<gain_t>> ls_operator::edge_costs_around_edge;
std::vector<std::vector<gain_t>> ls_operator::edge_gains;
std::vector<index_t> ls_operator::edge_candidates;

ls_operator::ls_operator(const input& input,
                         raw_solution& sol,
                         index_t source_vehicle,
                         index_t source_rank,
                         index_t target_vehicle,
                         index_t target_rank)
  : _input(input),
    _sol(sol),
    source_vehicle(source_vehicle),
    source_rank(source_rank),
    target_vehicle(target_vehicle),
    target_rank(target_rank),
    gain_computed(false) {
}

gain_t ls_operator::gain() {
  if (!gain_computed) {
    this->compute_gain();
  }
  return stored_gain;
}
