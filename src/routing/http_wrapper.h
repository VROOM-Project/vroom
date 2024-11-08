#ifndef HTTP_WRAPPER_H
#define HTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include "../include/rapidjson/include/rapidjson/document.h"

#include "routing/wrapper.h"
#include "structures/typedefs.h"
#include "utils/helpers.h"

namespace vroom::routing {

class HttpWrapper : public Wrapper {
private:
  std::string send_then_receive(const std::string& query) const;

  std::string ssl_send_then_receive(const std::string& query) const;

  static const std::string HTTPS_PORT;

protected:
  const Server _server;
  const std::string _matrix_service;
  const std::string _matrix_durations_key;
  const std::string _matrix_distances_key;
  const std::string _route_service;
  const std::string _routing_args;

  HttpWrapper(const std::string& profile,
              Server server,
              std::string matrix_service,
              std::string matrix_durations_key,
              std::string matrix_distances_key,
              std::string route_service,
              std::string routing_args);

  std::string run_query(const std::string& query) const;

  static void parse_response(rapidjson::Document& json_result,
                             const std::string& json_content);

  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service) const = 0;

  virtual void check_response(const rapidjson::Document& json_result,
                              const std::vector<Location>& locs,
                              const std::string& service) const = 0;

  Matrices get_matrices(const std::vector<Location>& locs) const override;

  void update_sparse_matrix(const std::vector<Location>& route_locs,
                            Matrices& m,
                            std::mutex& matrix_m,
                            std::string& vehicles_geometry) const override;

  virtual bool
  duration_value_is_null(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    return matrix_entry.IsNull();
  }

  virtual bool
  distance_value_is_null(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    return matrix_entry.IsNull();
  }

  virtual UserDuration
  get_duration_value(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    return utils::round<UserDuration>(matrix_entry.GetDouble());
  }

  virtual UserDistance
  get_distance_value(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    return utils::round<UserDistance>(matrix_entry.GetDouble());
  }

  virtual const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const = 0;

  virtual UserDuration get_leg_duration(const rapidjson::Value& leg) const {
    // Same implementation for both OSRM and ORS.
    assert(leg.HasMember("duration"));
    return utils::round<UserDuration>(leg["duration"].GetDouble());
  }

  virtual UserDistance get_leg_distance(const rapidjson::Value& leg) const {
    // Same implementation for both OSRM and ORS.
    assert(leg.HasMember("distance"));
    return utils::round<UserDistance>(leg["distance"].GetDouble());
  }

  virtual std::string get_geometry(rapidjson::Value& result) const {
    // Same implementation for both OSRM and ORS.
    return result["routes"][0]["geometry"].GetString();
  }

  void add_geometry(Route& route) const override;
};

} // namespace vroom::routing

#endif
