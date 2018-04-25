/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./libosrm_wrapper.h"

libosrm_wrapper::libosrm_wrapper(const std::string& osrm_profile)
  : osrm_wrapper(osrm_profile), _config(), _osrm(_config) {
}

matrix<cost_t>
libosrm_wrapper::get_matrix(const std::vector<location_t>& locs) const {
  osrm::TableParameters params;
  for (auto const& location : locs) {
    assert(location.has_coordinates());
    params.coordinates
      .emplace_back(osrm::util::FloatLongitude({location.lon()}),
                    osrm::util::FloatLatitude({location.lat()}));
  }

  osrm::json::Object result;
  osrm::Status status;
  try {
    status = _osrm.Table(params, result);
  } catch (const std::exception& e) {
    throw custom_exception(e.what());
  }

  if (status == osrm::Status::Error) {
    throw custom_exception(
      "libOSRM: " + result.values["code"].get<osrm::json::String>().value +
      ": " + result.values["message"].get<osrm::json::String>().value);
  }

  auto& table = result.values["durations"].get<osrm::json::Array>();

  // Expected matrix size.
  std::size_t m_size = locs.size();
  assert(table.values.size() == m_size);

  // Build matrix while checking for unfound routes to avoid
  // unexpected behavior (OSRM raises 'null').
  matrix<cost_t> m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  std::string reason;
  for (std::size_t i = 0; i < m_size; ++i) {
    const auto& line = table.values.at(i).get<osrm::json::Array>();
    assert(line.values.size() == m_size);
    for (std::size_t j = 0; j < m_size; ++j) {
      const auto& el = line.values.at(j);
      if (el.is<osrm::json::Null>()) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        auto cost = round_cost(el.get<osrm::json::Number>().value);
        m[i][j] = cost;
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void libosrm_wrapper::add_route_info(route_t& rte) const {
  // Default options for routing.
  osrm::RouteParameters params(false, // steps
                               false, // alternatives
                               false, // annotations
                               osrm::RouteParameters::GeometriesType::Polyline,
                               osrm::RouteParameters::OverviewType::Full,
                               false // continue_straight
                               );

  // Ordering locations for the given steps.
  for (auto& step : rte.steps) {
    params.coordinates
      .emplace_back(osrm::util::FloatLongitude({step.location.lon()}),
                    osrm::util::FloatLatitude({step.location.lat()}));
  }

  osrm::json::Object result;
  osrm::Status status;
  try {
    status = _osrm.Route(params, result);
  } catch (const std::exception& e) {
    throw custom_exception(e.what());
  }

  if (status == osrm::Status::Error) {
    throw custom_exception(
      "libOSRM: " + result.values["code"].get<osrm::json::String>().value +
      ": " + result.values["message"].get<osrm::json::String>().value);
  }

  auto& result_routes = result.values["routes"].get<osrm::json::Array>();
  auto& route = result_routes.values.at(0).get<osrm::json::Object>();

  rte.duration =
    round_cost(route.values["duration"].get<osrm::json::Number>().value);
  rte.distance =
    round_cost(route.values["distance"].get<osrm::json::Number>().value);
  rte.geometry =
    std::move(route.values["geometry"].get<osrm::json::String>().value);

  auto& legs = route.values["legs"].get<osrm::json::Array>();
  auto nb_legs = legs.values.size();
  assert(nb_legs == rte.steps.size() - 1);
  double current_distance = 0;
  double current_duration = 0;

  rte.steps[0].distance = round_cost(current_distance);
  rte.steps[0].arrival = round_cost(current_duration);

  for (unsigned i = 0; i < nb_legs; ++i) {
    auto& leg = legs.values.at(i).get<osrm::json::Object>();
    current_distance += leg.values["distance"].get<osrm::json::Number>().value;
    current_duration += leg.values["duration"].get<osrm::json::Number>().value;

    rte.steps[i + 1].distance = round_cost(current_distance);
    rte.steps[i + 1].arrival = round_cost(current_duration);
  }
}
