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

// Unable to define an OSRM object as a class member and use it within
// get_matrix and get_route_infos because Table and Route are not
// const (see #34). This should be fixed in libosrm in the future (see
// OSRM #2861 and #2862). In the meantime, this workaround suggested
// by @daniel-j-h allows to support all libosrm 5.* versions.
struct S{
  mutable osrm::OSRM osrm;
  S(osrm::EngineConfig engine): osrm(engine){}
};

class libosrm_wrapper : public osrm_wrapper{

private:
  const osrm::EngineConfig _config;
  const S _s;

public:
  libosrm_wrapper(const std::string& osrm_profile);

  virtual matrix<distance_t> get_matrix(const std::vector<std::reference_wrapper<location>>& locs) const override;

  virtual void get_route_infos(const std::vector<std::reference_wrapper<location>>& locs,
                               const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const override;
};

#endif
