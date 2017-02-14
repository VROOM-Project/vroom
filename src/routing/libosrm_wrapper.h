#ifndef LIBOSRM_WRAPPER_H
#define LIBOSRM_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./osrm_wrapper.h"

#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/status.hpp"
#include "osrm/osrm.hpp"

class libosrm_wrapper : public osrm_wrapper{

private:
  osrm::EngineConfig _config;
  const osrm::OSRM _osrm;

public:
  libosrm_wrapper(const std::string& osrm_profile);

  virtual matrix<distance_t> get_matrix(const std::vector<location_t>& locs) const override;

  virtual void add_route_geometry(route_t& rte) const override;
};

#endif
