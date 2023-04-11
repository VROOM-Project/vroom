#ifndef HTTP_WRAPPER_H
#define HTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include "../include/rapidjson/document.h"

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
  const std::string _route_service;
  const std::string _extra_args;

  HttpWrapper(const std::string& profile,
              Server server,
              std::string matrix_service,
              std::string matrix_durations_key,
              std::string route_service,
              std::string extra_args);

  std::string run_query(const std::string& query) const;

  static void parse_response(rapidjson::Document& json_result,
                             const std::string& json_content);

  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service,
                                  const std::string& extra_args = "") const = 0;

  virtual void check_response(const rapidjson::Document& json_result,
                              const std::string& service) const = 0;

  Matrix<UserCost> get_matrix(const std::vector<Location>& locs) const override;

  virtual bool
  duration_value_is_null(const rapidjson::Value& matrix_entry) const = 0;

  virtual UserDuration
  get_duration_value(const rapidjson::Value& matrix_entry) const = 0;

  virtual double get_total_distance(const rapidjson::Value& result) const = 0;

  virtual unsigned get_legs_number(const rapidjson::Value& result) const = 0;

  virtual double get_distance_for_leg(const rapidjson::Value& result,
                                      rapidjson::SizeType i) const = 0;

  virtual std::string get_geometry(rapidjson::Value& result) const = 0;

  void add_route_info(Route& route) const override;
};

} // namespace vroom::routing

#endif
