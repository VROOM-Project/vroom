/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <boost/json.hpp>

#include "utils/input_parser.h"

namespace vroom::io {

// Helper to get optional array of coordinates.
inline Coordinates parse_coordinates(const boost::json::object& object,
                                     const char* key) {
  boost::json::array const array = object.at(key).get_array();
  if ((array.size() < 2) || !array[0].is_number() || !array[1].is_number()) {
    throw InputException("Invalid " + std::string(key) + " array.");
  }
  return {array[0].to_number<double>(),
          array[1].to_number<double>()};
}

inline std::string get_string(const boost::json::object& object,
                              const char* key) {
  std::string value;
  if (object.contains(key)) {
    if (!object.at(key).is_string()) {
      throw InputException("Invalid " + std::string(key) + " value.");
    }
    value = object.at(key).get_string().subview();
  }
  return value;
}

inline double get_double(const boost::json::object& object, const char* key) {
  double value = 1.;
  if (object.contains(key)) {
    if (!object.at(key).is_number()) {
      throw InputException("Invalid " + std::string(key) + " value.");
    }
    value = object.at(key).to_number<double>();
  }
  return value;
}

inline Amount get_amount(const boost::json::object& object,
                         const char* key,
                         unsigned amount_size) {
  // Default to zero amount with provided size.
  Amount amount(amount_size);

  if (object.contains(key)) {
    if (!object.at(key).is_array()) {
      throw InputException("Invalid " + std::string(key) + " array.");
    }

    boost::json::array const array = object.at(key).get_array();

    if (array.size() != amount_size) {
      throw InputException(std::format("Inconsistent {} length: {} and {}.",
                                       key,
                                       array.size(),
                                       amount_size));
    }

    for (size_t i = 0; i < array.size(); ++i) {
      if (!array[i].is_number()) {
        throw InputException("Invalid " + std::string(key) + " value.");
      }
      amount[i] = array[i].to_number<uint32_t>();
    }
  }

  return amount;
}

inline Skills get_skills(const boost::json::object& object) {
  Skills skills;
  if (object.contains("skills")) {
    if (!object.at("skills").is_array()) {
      throw InputException("Invalid skills object.");
    }

    boost::json::array const array = object.at("skills").get_array();

    for (size_t i = 0; i < array.size(); ++i) {
      if (!array[i].is_number()) {
        throw InputException("Invalid skill value.");
      }
      skills.insert(array[i].to_number<uint32_t>());
    }
  }

  return skills;
}

inline UserDuration get_duration(const boost::json::object& object,
                                 const char* key) {
  UserDuration duration = 0;
  if (object.contains(key)) {
    if (!object.at(key).is_number()) {
      throw InputException("Invalid " + std::string(key) + " duration.");
    }
    duration = object.at(key).to_number<uint32_t>();
  }
  return duration;
}

inline Priority get_priority(const boost::json::object& object) {
  Priority priority = 0;
  if (object.contains("priority")) {
    if (!object.at("priority").is_number()) {
      throw InputException("Invalid priority value.");
    }
    priority = object.at("priority").to_number<uint32_t>();
  }
  return priority;
}

template <typename T>
inline std::optional<T> get_value_for(const boost::json::object& object,
                                      const char* key) {
  std::optional<T> value;
  if (object.contains(key)) {
    if (!object.at(key).is_number()) {
      throw InputException("Invalid " + std::string(key) + " value.");
    }
    value = object.at(key).to_number<uint32_t>();
  }
  return value;
}

inline void check_id(const boost::json::value& v, const std::string& type) {
  if (!v.is_object()) {
    throw InputException("Invalid " + type + ".");
  }
  if (!v.get_object().contains("id") || !v.at("id").is_number()) {
    throw InputException("Invalid or missing id for " + type + ".");
  }
}

inline void check_shipment(const boost::json::value& v) {
  if (!v.is_object()) {
    throw InputException("Invalid shipment.");
  }
  if (!v.get_object().contains("pickup") || !v.at("pickup").is_object()) {
    throw InputException("Missing pickup for shipment.");
  }
  if (!v.get_object().contains("delivery") || !v.at("delivery").is_object()) {
    throw InputException("Missing delivery for shipment.");
  }
}

inline void check_location(const boost::json::object& v,
                           const std::string& type) {
  if (!v.contains("location") || !v.at("location").is_array()) {
    throw InputException(std::format("Invalid location for {} {}.",
                                     type,
                                     v.at("id").to_number<uint64_t>()));
  }
}

inline TimeWindow get_time_window(const boost::json::value& tw) {
  if (!tw.is_array() || tw.get_array().size() < 2 || !tw.at(0).is_number() ||
      !tw.at(1).is_number()) {
    throw InputException("Invalid time-window.");
  }
  return TimeWindow(tw.at(0).to_number<uint32_t>(),
                    tw.at(1).to_number<uint32_t>());
}

inline TimeWindow get_vehicle_time_window(const boost::json::object& v) {
  TimeWindow v_tw;
  if (v.contains("time_window")) {
    v_tw = get_time_window(v.at("time_window"));
  }
  return v_tw;
}

inline std::vector<TimeWindow> get_time_windows(const boost::json::object& o) {
  std::vector<TimeWindow> tws;
  if (o.contains("time_windows")) {
    if (!o.at("time_windows").is_array() ||
        o.at("time_windows").get_array().empty()) {
      throw InputException(
        std::format("Invalid time_windows array for object {}.",
                    o.at("id").to_number<uint64_t>()));
    }

    std::transform(o.at("time_windows").get_array().begin(),
                   o.at("time_windows").get_array().end(),
                   std::back_inserter(tws),
                   [](auto& tw) { return get_time_window(tw); });

    std::sort(tws.begin(), tws.end());
  } else {
    tws = std::vector<TimeWindow>(1, TimeWindow());
  }

  return tws;
}

inline Break get_break(const boost::json::value& b, unsigned amount_size) {
  check_id(b, "break");

  const auto max_load = b.get_object().contains("max_load")
                          ? get_amount(b.get_object(), "max_load", amount_size)
                          : std::optional<Amount>();

  return Break(b.at("id").to_number<uint64_t>(),
               get_time_windows(b.get_object()),
               get_duration(b.get_object(), "service"),
               get_string(b.get_object(), "description"),
               max_load);
}

inline std::vector<Break> get_vehicle_breaks(const boost::json::object& v,
                                             unsigned amount_size) {
  std::vector<Break> breaks;
  if (v.contains("breaks")) {
    if (!v.at("breaks").is_array()) {
      throw InputException(std::format("Invalid breaks for vehicle {}.",
                                       v.at("id").to_number<uint64_t>()));
    }

    std::transform(v.at("breaks").get_array().begin(),
                   v.at("breaks").get_array().end(),
                   std::back_inserter(breaks),
                   [&](auto& b) { return get_break(b, amount_size); });
  }

  std::ranges::sort(breaks, [](const auto& a, const auto& b) {
    return a.tws[0].start < b.tws[0].start ||
           (a.tws[0].start == b.tws[0].start && a.tws[0].end < b.tws[0].end);
  });

  return breaks;
}

inline VehicleCosts get_vehicle_costs(const boost::json::object& v) {
  UserCost fixed = 0;
  UserCost per_hour = DEFAULT_COST_PER_HOUR;
  UserCost per_km = DEFAULT_COST_PER_KM;

  if (v.contains("costs")) {
    if (!v.at("costs").is_object()) {
      throw InputException(std::format("Invalid costs for vehicle {}.",
                                       v.at("id").to_number<uint64_t>()));
    }

    if (v.at("costs").get_object().contains("fixed")) {
      if (!v.at("costs").at("fixed").is_number()) {
        throw InputException(std::format("Invalid fixed cost for vehicle {}.",
                                         v.at("id").to_number<uint64_t>()));
      }

      fixed = v.at("costs").at("fixed").to_number<uint32_t>();
    }

    if (v.at("costs").get_object().contains("per_hour")) {
      if (!v.at("costs").at("per_hour").is_number()) {
        throw InputException(
          std::format("Invalid per_hour cost for vehicle {}.",
                      v.at("id").to_number<uint64_t>()));
      }

      per_hour = v.at("costs").at("per_hour").to_number<uint32_t>();
    }

    if (v.at("costs").get_object().contains("per_km")) {
      if (!v.at("costs").at("per_km").is_number()) {
        throw InputException(std::format("Invalid per_km cost for vehicle {}.",
                                         v.at("id").to_number<uint64_t>()));
      }

      per_km = v.at("costs").at("per_km").to_number<uint32_t>();
    }
  }

  return VehicleCosts(fixed, per_hour, per_km);
}

inline std::vector<VehicleStep>
get_vehicle_steps(const boost::json::object& v) {
  std::vector<VehicleStep> steps;

  if (v.contains("steps")) {
    if (!v.at("steps").is_array()) {
      throw InputException(std::format("Invalid steps for vehicle {}.",
                                       v.at("id").to_number<uint64_t>()));
    }

    steps.reserve(v.at("steps").get_array().size());

    for (size_t i = 0; i < v.at("steps").get_array().size(); ++i) {
      const auto& json_step = v.at("steps").at(i).get_object();

      std::optional<UserDuration> at;
      if (json_step.contains("service_at")) {
        if (!json_step.at("service_at").is_number()) {
          throw InputException("Invalid service_at value.");
        }

        at = json_step.at("service_at").to_number<uint32_t>();
      }
      std::optional<UserDuration> after;
      if (json_step.contains("service_after")) {
        if (!json_step.at("service_after").is_number()) {
          throw InputException("Invalid service_after value.");
        }

        after = json_step.at("service_after").to_number<uint32_t>();
      }
      std::optional<UserDuration> before;
      if (json_step.contains("service_before")) {
        if (!json_step.at("service_before").is_number()) {
          throw InputException("Invalid service_before value.");
        }

        before = json_step.at("service_before").to_number<uint32_t>();
      }
      ForcedService forced_service(at, after, before);

      const auto type_str = get_string(json_step, "type");

      if (type_str == "start") {
        steps.emplace_back(STEP_TYPE::START, std::move(forced_service));
        continue;
      }
      if (type_str == "end") {
        steps.emplace_back(STEP_TYPE::END, std::move(forced_service));
        continue;
      }

      if (!json_step.contains("id") || !json_step.at("id").is_number()) {
        throw InputException(std::format("Invalid id in steps for vehicle {}.",
                                         v.at("id").to_number<uint64_t>()));
      }

      if (type_str == "job") {
        steps.emplace_back(JOB_TYPE::SINGLE,
                           json_step.at("id").to_number<uint64_t>(),
                           std::move(forced_service));
      } else if (type_str == "pickup") {
        steps.emplace_back(JOB_TYPE::PICKUP,
                           json_step.at("id").to_number<uint64_t>(),
                           std::move(forced_service));
      } else if (type_str == "delivery") {
        steps.emplace_back(JOB_TYPE::DELIVERY,
                           json_step.at("id").to_number<uint64_t>(),
                           std::move(forced_service));
      } else if (type_str == "break") {
        steps.emplace_back(STEP_TYPE::BREAK,
                           json_step.at("id").to_number<uint64_t>(),
                           std::move(forced_service));
      } else {
        throw InputException(
          std::format("Invalid type in steps for vehicle {}.",
                      v.at("id").to_number<uint64_t>()));
      }
    }
  }

  return steps;
}

inline Vehicle get_vehicle(const boost::json::object& json_vehicle,
                           unsigned amount_size) {
  check_id(json_vehicle, "vehicle");
  auto v_id = json_vehicle.at("id").to_number<uint64_t>();

  // Check what info are available for vehicle start, then build
  // optional start location.
  bool has_start_coords = json_vehicle.contains("start");
  bool has_start_index = json_vehicle.contains("start_index");
  if (has_start_index && !json_vehicle.at("start_index").is_number()) {
    throw InputException(
      std::format("Invalid start_index for vehicle {}.", v_id));
  }

  std::optional<Location> start;
  if (has_start_index) {
    // Custom provided matrices and index.
    Index start_index = json_vehicle.at("start_index").to_number<uint32_t>();
    if (has_start_coords) {
      start = Location({start_index, parse_coordinates(json_vehicle, "start")});
    } else {
      start = Location(start_index);
    }
  } else {
    if (has_start_coords) {
      start = Location(parse_coordinates(json_vehicle, "start"));
    }
  }

  // Check what info are available for vehicle end, then build
  // optional end location.
  bool has_end_coords = json_vehicle.contains("end");
  bool has_end_index = json_vehicle.contains("end_index");
  if (has_end_index && !json_vehicle.at("end_index").is_number()) {
    throw InputException(
      std::format("Invalid end_index for vehicle {}.", v_id));
  }

  std::optional<Location> end;
  if (has_end_index) {
    // Custom provided matrices and index.
    Index end_index = json_vehicle.at("end_index").to_number<uint32_t>();
    if (has_end_coords) {
      end = Location({end_index, parse_coordinates(json_vehicle, "end")});
    } else {
      end = Location(end_index);
    }
  } else {
    if (has_end_coords) {
      end = Location(parse_coordinates(json_vehicle, "end"));
    }
  }

  std::string profile = get_string(json_vehicle, "profile");
  if (profile.empty()) {
    profile = DEFAULT_PROFILE;
  }

  return Vehicle(v_id,
                 start,
                 end,
                 profile,
                 get_amount(json_vehicle, "capacity", amount_size),
                 get_skills(json_vehicle),
                 get_vehicle_time_window(json_vehicle),
                 get_vehicle_breaks(json_vehicle, amount_size),
                 get_string(json_vehicle, "description"),
                 get_vehicle_costs(json_vehicle),
                 get_double(json_vehicle, "speed_factor"),
                 get_value_for<size_t>(json_vehicle, "max_tasks"),
                 get_value_for<UserDuration>(json_vehicle, "max_travel_time"),
                 get_value_for<UserDistance>(json_vehicle, "max_distance"),
                 get_vehicle_steps(json_vehicle));
}

inline Location get_task_location(const boost::json::object& v,
                                  const std::string& type) {
  // Check what info are available to build task location.
  bool has_location_coords = v.contains("location");
  bool has_location_index = v.contains("location_index");
  if (has_location_index && !v.at("location_index").is_number()) {
    throw InputException(std::format("Invalid location_index for {} {}.",
                                     type,
                                     v.at("id").to_number<uint64_t>()));
  }

  if (has_location_index) {
    // Custom provided matrices and index.
    Index location_index = v.at("location_index").to_number<uint32_t>();
    if (has_location_coords) {
      return Location({location_index, parse_coordinates(v, "location")});
    }
    return Location(location_index);
  }
  check_location(v, type);
  return Location(parse_coordinates(v, "location"));
}

inline Job get_job(const boost::json::object& json_job, unsigned amount_size) {
  check_id(json_job, "job");

  // Only for retro-compatibility: when no pickup and delivery keys
  // are defined and (deprecated) amount key is present, it should be
  // interpreted as a delivery.
  bool need_amount_compat = json_job.contains("amount") &&
                            !json_job.contains("delivery") &&
                            !json_job.contains("pickup");

  return Job(json_job.at("id").to_number<uint64_t>(),
             get_task_location(json_job, "job"),
             get_duration(json_job, "setup"),
             get_duration(json_job, "service"),
             need_amount_compat ? get_amount(json_job, "amount", amount_size)
                                : get_amount(json_job, "delivery", amount_size),
             get_amount(json_job, "pickup", amount_size),
             get_skills(json_job),
             get_priority(json_job),
             get_time_windows(json_job),
             get_string(json_job, "description"));
}

template <class T> inline Matrix<T> get_matrix(boost::json::value& m) {
  if (!m.is_array()) {
    throw InputException("Invalid matrix.");
  }
  // Load custom matrix while checking if it is square.
  size_t matrix_size = m.get_array().size();

  Matrix<T> matrix(matrix_size);
  for (size_t i = 0; i < matrix_size; ++i) {
    if (!m.at(i).is_array() || m.at(i).get_array().size() != matrix_size) {
      throw InputException("Unexpected matrix line length.");
    }
    boost::json::array mi = m.at(i).get_array();
    for (size_t j = 0; j < matrix_size; ++j) {
      if (!mi[j].is_number()) {
        throw InputException("Invalid matrix entry.");
      }
      matrix[i][j] = mi[j].to_number<uint32_t>();
    }
  }

  return matrix;
}

void parse(Input& input, const std::string& input_str, bool geometry) {
  // Input json object.
  boost::json::error_code ec;
  boost::json::monotonic_resource mr;
  boost::json::parse_options opt;
  opt.numbers = boost::json::number_precision::precise;
  auto const content = boost::json::parse(input_str, ec, &mr, opt);

  if (ec) {
    std::string error_msg =
      std::format("{} (offset: {})", ec.message(), ec.location().to_string());
    throw InputException(error_msg);
  }

  if (!content.is_object())
    throw InputException("Invalid input.");
  boost::json::object json_input = content.get_object();

  // Main checks for valid json input.
  bool has_jobs = json_input.contains("jobs") &&
                  json_input.at("jobs").is_array() &&
                  !json_input.at("jobs").get_array().empty();
  bool has_shipments = json_input.contains("shipments") &&
                       json_input.at("shipments").is_array() &&
                       !json_input.at("shipments").get_array().empty();
  if (!has_jobs && !has_shipments) {
    throw InputException("Invalid jobs or shipments.");
  }

  if (!json_input.contains("vehicles") ||
      !json_input.at("vehicles").is_array() ||
      json_input.at("vehicles").get_array().empty()) {
    throw InputException("Invalid vehicles.");
  }
  const auto& first_vehicle = json_input.at("vehicles").at(0).get_object();
  check_id(first_vehicle, "vehicle");
  bool first_vehicle_has_capacity =
    (first_vehicle.contains("capacity") &&
     first_vehicle.at("capacity").is_array() &&
     first_vehicle.at("capacity").get_array().size() > 0);
  const unsigned amount_size =
    first_vehicle_has_capacity ? first_vehicle.at("capacity").get_array().size()
                               : 0;

  input.set_amount_size(amount_size);
  input.set_geometry(geometry);

  // Add all vehicles.
  for (size_t i = 0; i < json_input.at("vehicles").get_array().size(); ++i) {
    auto& json_vehicle = json_input.at("vehicles").at(i).get_object();

    input.add_vehicle(get_vehicle(json_vehicle, amount_size));
  }

  // Add all tasks.
  if (has_jobs) {
    // Add the jobs.
    for (size_t i = 0; i < json_input.at("jobs").get_array().size(); ++i) {
      input.add_job(
        get_job(json_input.at("jobs").at(i).get_object(), amount_size));
    }
  }

  if (has_shipments) {
    // Add the shipments.
    for (size_t i = 0; i < json_input.at("shipments").get_array().size(); ++i) {
      auto& json_shipment = json_input.at("shipments").at(i).get_object();
      check_shipment(json_shipment);

      // Retrieve common stuff for both pickup and delivery.
      auto amount = get_amount(json_shipment, "amount", amount_size);
      auto skills = get_skills(json_shipment);
      auto priority = get_priority(json_shipment);

      // Defining pickup job.
      auto& json_pickup = json_shipment.at("pickup").get_object();
      check_id(json_pickup, "pickup");

      Job pickup(json_pickup.at("id").to_number<uint64_t>(),
                 JOB_TYPE::PICKUP,
                 get_task_location(json_pickup, "pickup"),
                 get_duration(json_pickup, "setup"),
                 get_duration(json_pickup, "service"),
                 amount,
                 skills,
                 priority,
                 get_time_windows(json_pickup),
                 get_string(json_pickup, "description"));

      // Defining delivery job.
      auto& json_delivery = json_shipment.at("delivery").get_object();
      check_id(json_delivery, "delivery");

      Job delivery(json_delivery.at("id").to_number<uint64_t>(),
                   JOB_TYPE::DELIVERY,
                   get_task_location(json_delivery, "delivery"),
                   get_duration(json_delivery, "setup"),
                   get_duration(json_delivery, "service"),
                   amount,
                   skills,
                   priority,
                   get_time_windows(json_delivery),
                   get_string(json_delivery, "description"));

      input.add_shipment(pickup, delivery);
    }
  }

  if (json_input.contains("matrices")) {
    if (!json_input.at("matrices").is_object()) {
      throw InputException("Unexpected matrices value.");
    }
    for (auto& profile_entry : json_input.at("matrices").get_object()) {
      if (profile_entry.value().is_object()) {
        if (profile_entry.value().get_object().contains("durations")) {
          input.set_durations_matrix(profile_entry.key(),
                                     get_matrix<UserDuration>(
                                       profile_entry.value().at("durations")));
        }
        if (profile_entry.value().get_object().contains("distances")) {
          input.set_distances_matrix(profile_entry.key(),
                                     get_matrix<UserDistance>(
                                       profile_entry.value().at("distances")));
        }
        if (profile_entry.value().get_object().contains("costs")) {
          input.set_costs_matrix(profile_entry.key(),
                                 get_matrix<UserCost>(
                                   profile_entry.value().at("costs")));
        }
      }
    }
  } else {
    // Deprecated `matrix` key still interpreted as
    // `matrices.DEFAULT_PROFILE.duration` for retro-compatibility.
    if (json_input.contains("matrix")) {
      input.set_durations_matrix(DEFAULT_PROFILE,
                                 get_matrix<UserDuration>(
                                   json_input.at("matrix")));
    }
  }
}

} // namespace vroom::io
