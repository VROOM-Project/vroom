/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../include/polylineencoder/src/polylineencoder.h"

#include "routing/valhalla_wrapper.h"
#include "utils/helpers.h"
#include "utils/output_json.h"

namespace vroom::routing {

constexpr unsigned km_to_m = 1000;
constexpr unsigned polyline_precision = 5;
constexpr unsigned valhalla_polyline_precision = 6;

ValhallaWrapper::ValhallaWrapper(const std::string& profile,
                                 const Server& server)
  : HttpWrapper(profile,
                server,
                "sources_to_targets",
                "sources_to_targets",
                "sources_to_targets",
                "route") {
}

rapidjson::Document ValhallaWrapper::build_matrix_body(
  const std::vector<Location>& locations) const {
  rapidjson::Document body;
  body.SetObject();
  auto& allocator = body.GetAllocator();
  auto to_json = [&](const Location& location) {
    rapidjson::Value j_location(rapidjson::kObjectType);
    j_location.AddMember("lon", location.lon(), allocator);
    j_location.AddMember("lat", location.lat(), allocator);
    return j_location;
  };

  auto build_locations = [&](const std::vector<Location>& locations) {
    rapidjson::Value j_locations(rapidjson::kArrayType);
    for (const Location& location : locations) {
      j_locations.PushBack(to_json(location), allocator);
    }

    return j_locations;
  };
  body.AddMember("sources", build_locations(locations), allocator);
  body.AddMember("targets", build_locations(locations), allocator);
  body.AddMember("costing",
                 rapidjson::Value(profile.c_str(), allocator),
                 allocator);
  return body;
}

rapidjson::Document ValhallaWrapper::build_route_body(
  const std::vector<Location>& locations) const {
  rapidjson::Document body;
  body.SetObject();
  auto& allocator = body.GetAllocator();

  rapidjson::Value j_locations(rapidjson::kArrayType);
  for (const auto& location : locations) {
    rapidjson::Value j_location(rapidjson::kObjectType);
    j_location.AddMember("lon", location.lon(), allocator);
    j_location.AddMember("lat", location.lat(), allocator);
    j_location.AddMember("type", "break", allocator);
    j_locations.PushBack(j_location, allocator);
  }

  body.AddMember("locations", j_locations, allocator);
  body.AddMember("costing",
                 rapidjson::Value(profile.c_str(), allocator),
                 allocator);
  body.AddMember("directions_type", "none", allocator);

  return body;
}

std::string ValhallaWrapper::build_query(const std::vector<Location>& locations,
                                         const std::string& service) const {
  assert(service == _matrix_service || service == _route_service);
  const rapidjson::Document body = (service == _matrix_service)
                                     ? build_matrix_body(locations)
                                     : build_route_body(locations);
  return build_get_query(service + "?json=",
                         io::json_to_string(body, COORDINATES_PRECISION));
}

void ValhallaWrapper::check_response(const rapidjson::Document& json_result,
                                     const std::vector<Location>&,
                                     const std::string& service) const {
  assert(service == _matrix_service || service == _route_service);

  if (constexpr unsigned HTTP_OK = 200;
      json_result.HasMember("status_code") &&
      json_result["status_code"].IsUint() &&
      json_result["status_code"].GetUint() != HTTP_OK) {
    // Valhalla responses seem to only have a status_code key when a
    // problem is encountered. In that case it's not really clear what
    // keys can be expected so we're playing guesses. This happens
    // e.g. when requested matrix/route size goes over the server
    // limit.
    const std::string service_str =
      (service == _route_service) ? "route" : "matrix";
    std::string error = "Valhalla " + service_str + " error (";

    if (json_result.HasMember("error") && json_result["error"].IsString()) {
      error += json_result["error"].GetString();
      error += ").";
    }
    throw RoutingException(error);
  }

  if (service == _route_service) {
    assert(json_result.HasMember("trip") &&
           json_result["trip"].HasMember("status"));
    if (json_result["trip"]["status"] != 0) {
      throw RoutingException(
        std::string(json_result["trip"]["status_message"].GetString()));
    }
  }
}

bool ValhallaWrapper::duration_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry.HasMember("time"));
  return matrix_entry["time"].IsNull();
}

bool ValhallaWrapper::distance_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry.HasMember("distance"));
  return matrix_entry["distance"].IsNull();
}

UserDuration ValhallaWrapper::get_duration_value(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry["time"].IsUint());
  return matrix_entry["time"].GetUint();
}

UserDistance ValhallaWrapper::get_distance_value(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry["distance"].IsDouble());
  return utils::round<UserDistance>(km_to_m *
                                    matrix_entry["distance"].GetDouble());
}

const rapidjson::Value&
ValhallaWrapper::get_legs(const rapidjson::Value& result) const {
  assert(result.HasMember("trip") && result["trip"].HasMember("legs") and
         result["trip"]["legs"].IsArray());

  return result["trip"]["legs"];
}

UserDuration
ValhallaWrapper::get_leg_duration(const rapidjson::Value& leg) const {
  assert(leg.HasMember("summary") && leg["summary"].HasMember("time"));
  return utils::round<UserDuration>(leg["summary"]["time"].GetDouble());
}

UserDistance
ValhallaWrapper::get_leg_distance(const rapidjson::Value& leg) const {
  assert(leg.HasMember("summary") && leg["summary"].HasMember("length"));
  return utils::round<UserDistance>(km_to_m *
                                    leg["summary"]["length"].GetDouble());
}

std::string ValhallaWrapper::get_geometry(rapidjson::Value& result) const {
  // Valhalla returns one polyline per route leg so we need to merge
  // them. Also taking the opportunity to adjust the encoding
  // precision as Valhalla uses 6 and we use 5 based on other routing
  // engine output. Note: getting directly a single polyline (e.g. by
  // not sending type=break for the route request) is not an option
  // since we have to force allowing u-turns in order to get a
  // geometry that is consistent with the time/distance values in
  // matrices.

  auto full_polyline =
    gepaf::PolylineEncoder<valhalla_polyline_precision>::decode(
      result["trip"]["legs"][0]["shape"].GetString());

  for (rapidjson::SizeType i = 1; i < result["trip"]["legs"].Size(); ++i) {
    auto decoded_pts =
      gepaf::PolylineEncoder<valhalla_polyline_precision>::decode(
        result["trip"]["legs"][i]["shape"].GetString());

    if (!full_polyline.empty()) {
      full_polyline.pop_back();
    }
    full_polyline.insert(full_polyline.end(),
                         std::make_move_iterator(decoded_pts.begin()),
                         std::make_move_iterator(decoded_pts.end()));
  }

  gepaf::PolylineEncoder<polyline_precision> encoder;
  for (const auto& p : full_polyline) {
    encoder.addPoint(p.latitude(), p.longitude());
  }

  return encoder.encode();
}

} // namespace vroom::routing
