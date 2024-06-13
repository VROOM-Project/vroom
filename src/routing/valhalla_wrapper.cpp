/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../include/polylineencoder/src/polylineencoder.h"

#include "routing/valhalla_wrapper.h"
#include "utils/helpers.h"

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
                "route",
                R"("directions_type":"none")") {
}

std::string ValhallaWrapper::get_matrix_query(
  const std::vector<Location>& locations) const {
  // Building matrix query for Valhalla.
  std::string query = "GET /" + _server.path + _matrix_service + "?json=";

  // List locations.
  std::string all_locations;
  for (auto const& location : locations) {
    all_locations +=
      std::format(R"({{"lon":{},"lat":{}}},)", location.lon(), location.lat());
  }
  all_locations.pop_back(); // Remove trailing ','.

  query += "{\"sources\":[" + all_locations;
  query += "],\"targets\":[" + all_locations;
  query += R"(],"costing":")" + profile + "\"}";

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: Close\r\n\r\n";

  return query;
}

std::string
ValhallaWrapper::get_route_query(const std::vector<Location>& locations) const {
  // Building matrix query for Valhalla.
  std::string query =
    "GET /" + _server.path + _route_service + "?json={\"locations\":[";

  for (auto const& location : locations) {
    query += std::format(R"({{"lon":{},"lat":{},"type":"break"}},)",
                         location.lon(),
                         location.lat());
  }
  query.pop_back(); // Remove trailing ','.

  query += R"(],"costing":")" + profile + "\"";
  query += "," + _routing_args;
  query += "}";

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: Close\r\n\r\n";

  return query;
}

std::string ValhallaWrapper::build_query(const std::vector<Location>& locations,
                                         const std::string& service) const {
  assert(service == _matrix_service || service == _route_service);

  return (service == _matrix_service) ? get_matrix_query(locations)
                                      : get_route_query(locations);
}

void ValhallaWrapper::check_response(const boost::json::object& json_result,
                                     const std::vector<Location>&,
                                     const std::string& service) const {
  assert(service == _matrix_service || service == _route_service);

  if (constexpr unsigned HTTP_OK = 200;
      json_result.contains("status_code") &&
      json_result.at("status_code").is_uint64() &&
      json_result.at("status_code").as_uint64() != HTTP_OK) {
    // Valhalla responses seem to only have a status_code key when a
    // problem is encountered. In that case it's not really clear what
    // keys can be expected so we're playing guesses. This happens
    // e.g. when requested matrix/route size goes over the server
    // limit.
    std::string service_str = (service == _route_service) ? "route" : "matrix";
    std::string error = "Valhalla " + service_str + " error (";

    if (json_result.contains("error") && json_result.at("error").is_string()) {
      error += json_result.at("error").as_string().subview();
      error += ").";
    }
    throw RoutingException(error);
  }

  std::cout << 2 << std::endl;

  if (service == _route_service) {
    assert(json_result.contains("trip") &&
           json_result.at("trip").as_object().contains("status"));
    if (json_result.at("trip").at("status") != 0) {
      throw RoutingException(
        json_result.at("trip").at("status_message").as_string().subview());
    }
  }
}

bool ValhallaWrapper::duration_value_is_null(
  const boost::json::value& matrix_entry) const {
  assert(matrix_entry.as_object().contains("time"));
  return matrix_entry.at("time").is_null();
}

bool ValhallaWrapper::distance_value_is_null(
  const boost::json::value& matrix_entry) const {
  assert(matrix_entry.as_object().contains("distance"));
  return matrix_entry.at("distance").is_null();
}

UserDuration ValhallaWrapper::get_duration_value(
  const boost::json::value& matrix_entry) const {
  if (matrix_entry.is_uint64())
    return utils::round<UserDuration>(matrix_entry.get_uint64());
  return utils::round<UserDuration>(matrix_entry.get_double());
}

UserDistance ValhallaWrapper::get_distance_value(
  const boost::json::value& matrix_entry) const {
  if (matrix_entry.is_uint64())
    return utils::round<UserDuration>(km_to_m *
                                    matrix_entry.at("distance").get_uint64());
  return utils::round<UserDuration>(km_to_m *
                                    matrix_entry.at("distance").get_double());
}

unsigned
ValhallaWrapper::get_legs_number(const boost::json::object& result) const {
  return result.at("trip").at("legs").as_array().size();
}

std::string ValhallaWrapper::get_geometry(boost::json::object& result) const {
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
      result.at("trip").at("legs").at(0).at("shape").as_string().subview());

  for (rapidjson::SizeType i = 1; i < result.at("trip").at("legs").as_array().size(); ++i) {
    auto decoded_pts =
      gepaf::PolylineEncoder<valhalla_polyline_precision>::decode(
        result.at("trip").at("legs").at(i).at("shape").as_string().subview());

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
