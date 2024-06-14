#ifndef HTTP_WRAPPER_H
#define HTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <boost/json.hpp>

#include "routing/wrapper.h"
#include "structures/typedefs.h"

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

  static void parse_response(boost::json::object& json_result,
                             const std::string& json_content);

  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service) const = 0;

  virtual void check_response(const boost::json::object& json_result,
                              const std::vector<Location>& locs,
                              const std::string& service) const = 0;

  Matrices get_matrices(const std::vector<Location>& locs) const override;

  virtual bool
  duration_value_is_null(const boost::json::value& matrix_entry) const = 0;

  virtual bool
  distance_value_is_null(const boost::json::value& matrix_entry) const = 0;

  virtual UserDuration
  get_duration_value(const boost::json::value& matrix_entry) const = 0;

  virtual UserDistance
  get_distance_value(const boost::json::value& matrix_entry) const = 0;

  virtual unsigned get_legs_number(const boost::json::object& result) const = 0;

  virtual std::string get_geometry(boost::json::object& result) const = 0;

  void add_geometry(Route& route) const override;
};

} // namespace vroom::routing

#endif
