#ifndef LIBOSRM_WRAPPER_H
#define LIBOSRM_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"

#include "routing/osrm_wrapper.h"

class libosrm_wrapper : public osrm_wrapper {

private:
  osrm::EngineConfig _config;
  const osrm::OSRM _osrm;

public:
  libosrm_wrapper(const std::string& osrm_profile);

  virtual matrix<cost_t>
  get_matrix(const std::vector<location_t>& locs) const override;

  virtual void add_route_info(route_t& rte) const override;
};

#endif
