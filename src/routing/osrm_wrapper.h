#ifndef OSRM_LOADER_H
#define OSRM_LOADER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include "../../include/rapidjson/error/en.h"
#include "./routing_io.h"
#include "../structures/abstract/matrix.h"
#include "../utils/exceptions.h"

class osrm_loader : public routing_io<distance_t>{

protected:
  const std::string _osrm_profile; // OSRM profile name

  static distance_t round_to_distance(double value){
    return static_cast<distance_t>(value + 0.5);
  }

  osrm_loader(const std::string& osrm_profile):
    _osrm_profile(osrm_profile){}

  virtual matrix<distance_t> get_matrix(const std::vector<std::reference_wrapper<location>>& locs) const = 0;

  inline void check_unfound(const std::vector<std::reference_wrapper<location>>& locs,
                            const std::vector<unsigned>& nb_unfound_from_loc,
                            const std::vector<unsigned>& nb_unfound_to_loc) const{
    assert(nb_unfound_from_loc.size() == nb_unfound_to_loc.size());
    unsigned max_unfound_routes_for_a_loc = 0;
    index_t error_loc = 0;    // Initial value never actually used.
    std::string error_direction;
    // Finding the "worst" location for unfound routes.
    for(unsigned i = 0; i < nb_unfound_from_loc.size(); ++i){
      if(nb_unfound_from_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_from_loc[i];
        error_loc = i;
        error_direction = "from";
      }
      if(nb_unfound_to_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_to_loc[i];
        error_loc = i;
        error_direction = "to";
      }
    }
    if(max_unfound_routes_for_a_loc > 0){
      std::string error_msg = "OSRM has unfound route(s) ";
      error_msg += error_direction;
      error_msg += "location ["
        + std::to_string(locs[error_loc][0])
        + ";"
        + std::to_string(locs[error_loc][1])
        + "]";
    }
    throw custom_exception(error_msg);
  }

  virtual void get_route_infos(const std::vector<std::reference_wrapper<location>>& locs,
                               const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const = 0;
};

#endif
