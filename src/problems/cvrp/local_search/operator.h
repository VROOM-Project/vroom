#ifndef LS_OPERATOR_H
#define LS_OPERATOR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../structures/vroom/input/input.h"

class ls_operator {
protected:
  const input& _input;
  raw_solution& _sol;

  const index_t source_vehicle;
  const index_t source_rank;
  const index_t target_vehicle;
  const index_t target_rank;
  bool gain_computed;
  gain_t stored_gain;

  virtual void compute_gain() = 0;

public:
  ls_operator(const input& input,
              raw_solution& sol,
              index_t source_vehicle,
              index_t source_rank,
              index_t target_vehicle,
              index_t target_rank);

  // amounts[v][i] stores the total amount up to rank i in the route
  // for vehicle v .
  static std::vector<std::vector<amount_t>> amounts;

  // edge_costs_around_node[v][i] stores the sum of costs for edges
  // that appear before and after job at rank i in route for vehicle v
  // (handling cases where those edges are absent or linked with
  // start/end of vehicle). node_gains[v][i] stores potential gain
  // when removing job at rank i in route for vehicle
  // v. node_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  static std::vector<std::vector<gain_t>> edge_costs_around_node;
  static std::vector<std::vector<gain_t>> node_gains;
  static std::vector<index_t> node_candidates;

  // edge_costs_around_edge[v][i] stores the sum of costs for edges
  // that appear before and after edge starting at rank i in route for
  // vehicle v (handling cases where those edges are absent or linked
  // with start/end of vehicle). edge_gains[v][i] stores potential
  // gain when removing edge starting at rank i in route for vehicle
  // v. edge_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  static std::vector<std::vector<gain_t>> edge_costs_around_edge;
  static std::vector<std::vector<gain_t>> edge_gains;
  static std::vector<index_t> edge_candidates;

  gain_t gain();

  virtual bool is_valid() const = 0;

  virtual void apply() const = 0;

  virtual void log() const = 0;

  virtual std::vector<index_t> addition_candidates() const = 0;

  virtual ~ls_operator() {
  }
};

#endif
