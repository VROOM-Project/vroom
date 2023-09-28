#ifndef LIBOSRM_WRAPPER_H
#define LIBOSRM_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"

#include "routing/wrapper.h"

namespace vroom {
namespace routing {

class LibosrmWrapper : public Wrapper {

private:
  osrm::EngineConfig _config;
  const osrm::OSRM _osrm;

  static osrm::EngineConfig get_config(const std::string& profile);

public:
  explicit LibosrmWrapper(const std::string& profile);

  Matrices get_matrices(const std::vector<Location>& locs) const override;

  void add_geometry(Route& route) const override;
};

} // namespace routing
} // namespace vroom

#endif
