#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <vector>
#include <boost/optional.hpp>
#include "../../../include/rapidjson/document.h"
#include "../../../include/rapidjson/error/en.h"
#include "../typedefs.h"
#include "../abstract/matrix.h"
#include "./job.h"
#include "./vehicle.h"
#include "../../utils/exceptions.h"

using optional_coords_t = boost::optional<std::array<coordinate_t, 2>>;

class input{
private:
  index_t _location_number;
  rapidjson::Document _json_input;

protected:
  std::vector<job> _jobs;
  std::vector<vehicle> _vehicles;
  matrix<distance_t> _matrix;
  // List of locations added through add_* matching the matrix
  // ordering.
  std::vector<std::reference_wrapper<location>> _ordered_locations;

public:
  input();

  void add_job(index_t id, optional_coords_t coords);

  void add_vehicle(index_t id,
                   optional_coords_t start_coords,
                   optional_coords_t end_coords);

  void parse(const std::string& input);
};

#endif
