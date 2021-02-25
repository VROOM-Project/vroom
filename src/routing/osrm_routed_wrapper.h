#ifndef OSRM_ROUTED_WRAPPER_H
#define OSRM_ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/http_wrapper.h"

namespace vroom {
namespace routing {

class OsrmRoutedWrapper : public HttpWrapper {
private:
  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service,
                                  const std::string& extra_args) const override;

  virtual void check_response(const rapidjson::Document& json_result,
                              const std::string& service) const override;

  virtual bool
  duration_value_is_null(const rapidjson::Value& matrix_entry) const override;

  virtual Cost
  get_duration_value(const rapidjson::Value& matrix_entry) const override;

  virtual double
  get_total_distance(const rapidjson::Value& result) const override;

  virtual unsigned
  get_legs_number(const rapidjson::Value& result) const override;

  virtual double get_distance_for_leg(const rapidjson::Value& result,
                                      rapidjson::SizeType i) const override;

  virtual std::string get_geometry(rapidjson::Value& result) const override;

public:
  OsrmRoutedWrapper(const std::string& profile, const Server& server);
};

} // namespace routing
} // namespace vroom

#endif
