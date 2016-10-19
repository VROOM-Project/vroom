#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./typedefs.h"
#include "./abstract/matrix.h"
#include "../structures/vroom/job.h"
#include "../structures/vroom/vehicle.h"
#include "../structures/vroom/output/output.h"


class vrp{
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const std::vector<job>& _jobs;
  const std::vector<vehicle>& _vehicles;
  matrix<distance_t> _matrix;

public:
  vrp(const std::vector<job>& jobs,
      const std::vector<vehicle>& vehicles,
      const matrix<distance_t>& matrix):
    _jobs(jobs),
    _vehicles(vehicles),
    _matrix(matrix){
    assert(_vehicles.size() > 0);
  }

  virtual output solve() const = 0;
};

#endif
