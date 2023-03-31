/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../include/polylineencoder/src/polylineencoder.h"

#include "routing/valhalla_wrapper.h"

namespace vroom {
namespace routing {

ValhallaWrapper::ValhallaWrapper(const std::string& profile,
                                 const Server& server, const std::string& extra_args)
  : HttpWrapper(profile,
                server,
                "sources_to_targets",
                "sources_to_targets",
                "route",
                extra_args
                ) {
}

std::string ValhallaWrapper::get_matrix_query(
  const std::vector<Location>& locations) const {
  // Building matrix query for Valhalla.
  std::string query = "GET /" + _matrix_service + "?json=";

  // List locations.
  std::string all_locations;
  for (auto const& location : locations) {
    all_locations += "{\"lon\":" + std::to_string(location.lon()) + "," +
                     "\"lat\":" + std::to_string(location.lat()) + "},";
  }
  all_locations.pop_back(); // Remove trailing ','.

  query += "{\"sources\":[" + all_locations;
  query += "],\"targets\":[" + all_locations;
  query += "],\"costing\":\"" + profile + "\"}";

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: Close\r\n\r\n";

  return query;
}

std::string
ValhallaWrapper::get_route_query(const std::vector<Location>& locations,
                                 const std::string& extra_args) const {
  // Building matrix query for Valhalla.
  std::string query = "GET /" + _route_service + "?json={\"locations\":[";

  for (auto const& location : locations) {
    query += "{\"lon\":" + std::to_string(location.lon()) + "," +
             "\"lat\":" + std::to_string(location.lat()) +
             ",\"type\":\"break\"},";
  }
  query.pop_back(); // Remove trailing ','.

  query += "],\"costing\":\"" + profile + "\",\"directions_type\":\"none\"";
  if (!extra_args.empty()) {
    query += "," + extra_args;
  }
  query += "}";

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: Close\r\n\r\n";

  return query;
}

std::string ValhallaWrapper::build_query(const std::vector<Location>& locations,
                                         const std::string& service,
                                         const std::string& extra_args) const {
  assert(service == _matrix_service or service == _route_service);

  return (service == _matrix_service) ? get_matrix_query(locations)
                                      : get_route_query(locations, extra_args);
}

void ValhallaWrapper::check_response(const rapidjson::Document& json_result,
                                     const std::string& service) const {
  assert(service == _matrix_service or service == _route_service);

  if (json_result.HasMember("status_code") and
      json_result["status_code"].IsUint() and
      json_result["status_code"].GetUint() != 200) {
    // Valhalla responses seem to only have a status_code key when a
    // problem is encountered. In that case it's not really clear what
    // keys can be expected so we're playing guesses. This happens
    // e.g. when requested matrix/route size goes over the server
    // limit.
    std::string service_str = (service == _route_service) ? "route" : "matrix";
    std::string error = "Valhalla " + service_str + " error (";

    if (json_result.HasMember("error") and json_result["error"].IsString()) {
      error += json_result["error"].GetString();
      error += ").";
    }
    throw RoutingException(error);
  }

  if (service == _route_service) {
    assert(json_result.HasMember("trip") and
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

UserDuration ValhallaWrapper::get_duration_value(
  const rapidjson::Value& matrix_entry) const {
  assert(matrix_entry["time"].IsUint());
  return matrix_entry["time"].GetUint();
}

double
ValhallaWrapper::get_total_distance(const rapidjson::Value& result) const {
  return 1000 * result["trip"]["summary"]["length"].GetDouble();
}

unsigned
ValhallaWrapper::get_legs_number(const rapidjson::Value& result) const {
  return result["trip"]["legs"].Size();
}

unsigned
ValhallaWrapper::get_steps_number(const rapidjson::Value& result, rapidjson::SizeType i) const {
  return result["trip"]["legs"].Size();
}

double ValhallaWrapper::get_distance_for_leg(const rapidjson::Value& result,
                                             rapidjson::SizeType i) const {
  return 1000 * result["trip"]["legs"][i]["summary"]["length"].GetDouble();
}

std::string ValhallaWrapper::get_geometry_for_leg(const rapidjson::Value& result,
                                             rapidjson::SizeType i, rapidjson::SizeType s) const {
  return result["trip"]["legs"][i]["shape"].GetString();
}

std::string ValhallaWrapper::get_geometry(rapidjson::Value& result) const {
  // Valhalla returns one polyline per route leg so we need to merge
  // them. Also taking the opportunity to adjust the encoding
  // precision as Valhalla uses 6 and we use 5 based on other routing
  // engine output. Note: getting directly a single polyline (e.g. by
  // not sending type=break for the route request) is not an option:
  // first we need all the legs for intermediate time and distance
  // values, then we have to force allowing u-turns in order to get
  // consistent time/distance values between matrix and route request.

  auto full_polyline = gepaf::PolylineEncoder<6>::decode(
    result["trip"]["legs"][0]["shape"].GetString());

  for (rapidjson::SizeType i = 1; i < result["trip"]["legs"].Size(); ++i) {
    auto decoded_pts = gepaf::PolylineEncoder<6>::decode(
      result["trip"]["legs"][i]["shape"].GetString());

    if (!full_polyline.empty()) {
      full_polyline.pop_back();
    }
    full_polyline.insert(full_polyline.end(),
                         std::make_move_iterator(decoded_pts.begin()),
                         std::make_move_iterator(decoded_pts.end()));
  }

  gepaf::PolylineEncoder<5> encoder;
  for (const auto& p : full_polyline) {
    encoder.addPoint(p.latitude(), p.longitude());
  }

  return encoder.encode();
}

} // namespace routing
} // namespace vroom
