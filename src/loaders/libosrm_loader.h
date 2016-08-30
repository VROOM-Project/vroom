#ifndef LIBOSRM_LOADER_H
#define LIBOSRM_LOADER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./osrm_loader.h"

#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/status.hpp"
#include "osrm/osrm.hpp"

using namespace osrm;

class libosrm_loader : public osrm_loader{

public:
  libosrm_loader(const std::string& osrm_profile,
                  const std::string& input):
    osrm_loader(osrm_profile, input) {}

  virtual matrix<distance_t> get_matrix() const override{
    EngineConfig config;
    OSRM osrm{config};

    TableParameters params;
    for(auto const& location: _locations){
      params.coordinates.push_back({util::FloatLongitude(location.lon),
            util::FloatLatitude(location.lat)});
    }

    json::Object result;
    Status status;
    try{
      status = osrm.Table(params, result);
    }
    catch(const std::exception &e){
      throw custom_exception(e.what());
    }

    if(status == Status::Error){
      throw custom_exception("libOSRM: "
                             + result.values["code"].get<json::String>().value
                             + ": "
                             + result.values["message"].get<json::String>().value);
    }

    auto& table = result.values["durations"].get<json::Array>();

    // Expected matrix size.
    std::size_t m_size = _locations.size();
    assert(table.values.size() == m_size);

    // Build matrix while checking for unfound routes to avoid
    // unexpected behavior (OSRM raises 'null').
    matrix<distance_t> m {m_size};

    std::vector<unsigned> nb_unfound_from_loc (m_size, 0);
    std::vector<unsigned> nb_unfound_to_loc (m_size, 0);

    std::string reason;
    for(std::size_t i = 0; i < m_size; ++i){
      const auto& line = table.values.at(i).get<json::Array>();
      assert(line.values.size() == m_size);
      for(std::size_t j = 0; j < m_size; ++j){
        const auto& el = line.values.at(j);
        if(el.is<json::Null>()){
          // No route found between i and j. Just storing info as we
          // don't know yet which location is responsible between i
          // and j.
          ++nb_unfound_from_loc[i];
          ++nb_unfound_to_loc[j];
        }
        else{
          m[i][j] = round_to_distance(el.get<json::Number>().value);
        }
      }
    }

    check_unfound(nb_unfound_from_loc, nb_unfound_to_loc);

    return m;
  }

  virtual void get_route_infos(const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const override{
    EngineConfig config;
    OSRM osrm{config};

    // Default options for routing.
    RouteParameters params(false, // steps
                           false, // alternatives
                           RouteParameters::GeometriesType::Polyline,
                           RouteParameters::OverviewType::Full,
                           false // continue_straight
                           );

    // Ordering locations for the given steps.
    for(auto& step: steps){
      params.coordinates.push_back({util::FloatLongitude(_locations[step].lon),
            util::FloatLatitude(_locations[step].lat)});
    }

    json::Object result;
    Status status;
    try{
      status = osrm.Route(params, result);
    }
    catch(const std::exception &e){
      throw custom_exception(e.what());
    }

    if(status == Status::Error){
      throw custom_exception("libOSRM: "
                             + result.values["code"].get<json::String>().value
                             + ": "
                             + result.values["message"].get<json::String>().value);
    }

    auto &routes = result.values["routes"].get<json::Array>();
    auto &route = routes.values.at(0).get<json::Object>();

    value.AddMember("duration",
                    round_to_distance(route.values["duration"].get<json::Number>().value),
                    allocator);
    value.AddMember("distance",
                    round_to_distance(route.values["distance"].get<json::Number>().value),
                    allocator);
    value.AddMember("geometry",
                    rapidjson::Value(route.values["geometry"].get<json::String>().value.c_str(),
                                     allocator),
                    allocator);
  }
};

#endif
