/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "../include/rapidjson/include/rapidjson/document.h"
#include "../include/rapidjson/include/rapidjson/error/en.h"

#include "utils/input_parser.h"

namespace vroom::io {

// Helper to get optional array of coordinates.
inline Coordinates parse_coordinates(const rapidjson::Value& object,
                                     const char* key) {
  if (!object[key].IsArray() || (object[key].Size() < 2) ||
      !object[key][0].IsNumber() || !object[key][1].IsNumber()) {
    throw InputException("Invalid " + std::string(key) + " array.");
  }
  return {object[key][0].GetDouble(), object[key][1].GetDouble()};
}

inline std::string get_string(const rapidjson::Value& object, const char* key) {
  std::string value;
  if (object.HasMember(key)) {
    if (!object[key].IsString()) {
      throw InputException("Invalid " + std::string(key) + " value.");
    }
    value = object[key].GetString();
  }
  return value;
}

inline double get_double(const rapidjson::Value& object, const char* key) {
  double value = 1.;
  if (object.HasMember(key)) {
    if (!object[key].IsNumber()) {
      throw InputException("Invalid " + std::string(key) + " value.");
    }
    value = object[key].GetDouble();
  }
  return value;
}

inline Amount get_amount(const rapidjson::Value& object,
                         const char* key,
                         unsigned amount_size) {
  const bool has_amount_key = object.HasMember(key);

  if (has_amount_key) {
    if (!object[key].IsArray()) {
      throw InputException("Invalid " + std::string(key) + " array.");
    }

    amount_size = object[key].Size();
  }

  // This will default to zero amount with provided size if expected
  // key is omitted.
  Amount amount(amount_size);

  if (has_amount_key) {
    for (rapidjson::SizeType i = 0; i < object[key].Size(); ++i) {
      if (!object[key][i].IsUint()) {
        throw InputException("Invalid " + std::string(key) + " value.");
      }
      amount[i] = object[key][i].GetUint();
    }
  }

  return amount;
}

inline Skills get_skills(const rapidjson::Value& object) {
  Skills skills;
  if (object.HasMember("skills")) {
    if (!object["skills"].IsArray()) {
      throw InputException("Invalid skills object.");
    }
    for (rapidjson::SizeType i = 0; i < object["skills"].Size(); ++i) {
      if (!object["skills"][i].IsUint()) {
        throw InputException("Invalid skill value.");
      }
      skills.insert(object["skills"][i].GetUint());
    }
  }

  return skills;
}

inline UserDuration get_duration(const rapidjson::Value& object,
                                 const char* key) {
  UserDuration duration = 0;
  if (object.HasMember(key)) {
    if (!object[key].IsUint()) {
      throw InputException("Invalid " + std::string(key) + " duration.");
    }
    duration = object[key].GetUint();
  }
  return duration;
}

inline TypeToUserDurationMap
get_duration_per_type(const rapidjson::Value& json_task,
                      const char* key,
                      const std::string& task_type) {
  TypeToUserDurationMap type_to_user_duration;

  if (json_task.HasMember(key)) {
    if (!json_task[key].IsObject()) {
      throw InputException(std::format("Invalid {} for {} {}.",
                                       key,
                                       task_type,
                                       json_task["id"].GetUint64()));
    }

    for (const auto& pair : json_task[key].GetObject()) {
      if (!pair.value.IsUint()) {
        throw InputException(std::format("Invalid value in {} for {} {}.",
                                         key,
                                         task_type,
                                         json_task["id"].GetUint64()));
      }

      type_to_user_duration.try_emplace(pair.name.GetString(),
                                        pair.value.GetUint());
    }
  }

  return type_to_user_duration;
}

inline Priority get_priority(const rapidjson::Value& object) {
  Priority priority = 0;
  if (object.HasMember("priority")) {
    if (!object["priority"].IsUint()) {
      throw InputException("Invalid priority value.");
    }
    priority = object["priority"].GetUint();
  }
  return priority;
}

template <typename T>
inline std::optional<T> get_value_for(const rapidjson::Value& object,
                                      const char* key) {
  std::optional<T> value;
  if (object.HasMember(key)) {
    if (!object[key].IsUint()) {
      throw InputException("Invalid " + std::string(key) + " value.");
    }
    value = object[key].GetUint();
  }
  return value;
}

inline void check_id(const rapidjson::Value& v, const std::string& type) {
  if (!v.IsObject()) {
    throw InputException("Invalid " + type + ".");
  }
  if (!v.HasMember("id") || !v["id"].IsUint64()) {
    throw InputException("Invalid or missing id for " + type + ".");
  }
}

inline void check_shipment(const rapidjson::Value& v) {
  if (!v.IsObject()) {
    throw InputException("Invalid shipment.");
  }
  if (!v.HasMember("pickup") || !v["pickup"].IsObject()) {
    throw InputException("Missing pickup for shipment.");
  }
  if (!v.HasMember("delivery") || !v["delivery"].IsObject()) {
    throw InputException("Missing delivery for shipment.");
  }
}

inline void check_location(const rapidjson::Value& v,
                           const std::string& task_type) {
  if (!v.HasMember("location") || !v["location"].IsArray()) {
    throw InputException(std::format("Invalid location for {} {}.",
                                     task_type,
                                     v["id"].GetUint64()));
  }
}

inline TimeWindow get_time_window(const rapidjson::Value& tw) {
  if (!tw.IsArray() || tw.Size() < 2 || !tw[0].IsUint() || !tw[1].IsUint()) {
    throw InputException("Invalid time-window.");
  }
  return TimeWindow(tw[0].GetUint(), tw[1].GetUint());
}

inline TimeWindow get_vehicle_time_window(const rapidjson::Value& v) {
  TimeWindow v_tw;
  if (v.HasMember("time_window")) {
    v_tw = get_time_window(v["time_window"]);
  }
  return v_tw;
}

inline std::vector<TimeWindow> get_time_windows(const rapidjson::Value& o,
                                                const std::string& task_type) {
  std::vector<TimeWindow> tws;
  if (o.HasMember("time_windows")) {
    if (!o["time_windows"].IsArray()) {
      throw InputException(std::format("Invalid time_windows array for {} {}.",
                                       task_type,
                                       o["id"].GetUint64()));
    }

    std::transform(o["time_windows"].Begin(),
                   o["time_windows"].End(),
                   std::back_inserter(tws),
                   [](auto& tw) { return get_time_window(tw); });

    std::sort(tws.begin(), tws.end());
  } else {
    tws = std::vector<TimeWindow>(1, TimeWindow());
  }

  return tws;
}

inline Break get_break(const rapidjson::Value& b, unsigned amount_size) {
  check_id(b, "break");

  const auto max_load = b.HasMember("max_load")
                          ? get_amount(b, "max_load", amount_size)
                          : std::optional<Amount>();

  return Break(b["id"].GetUint64(),
               get_time_windows(b, "break"),
               get_duration(b, "service"),
               get_string(b, "description"),
               max_load);
}

inline std::vector<Break> get_vehicle_breaks(const rapidjson::Value& v,
                                             unsigned amount_size) {
  std::vector<Break> breaks;
  if (v.HasMember("breaks")) {
    if (!v["breaks"].IsArray()) {
      throw InputException(
        std::format("Invalid breaks for vehicle {}.", v["id"].GetUint64()));
    }

    std::transform(v["breaks"].Begin(),
                   v["breaks"].End(),
                   std::back_inserter(breaks),
                   [&](auto& b) { return get_break(b, amount_size); });
  }

  std::ranges::sort(breaks, [](const auto& a, const auto& b) {
    return a.tws[0].start < b.tws[0].start ||
           (a.tws[0].start == b.tws[0].start && a.tws[0].end < b.tws[0].end);
  });

  return breaks;
}

inline VehicleCosts get_vehicle_costs(const rapidjson::Value& v) {
  UserCost fixed = 0;
  UserCost per_hour = DEFAULT_COST_PER_HOUR;
  UserCost per_km = DEFAULT_COST_PER_KM;
  UserCost per_task_hour = DEFAULT_COST_PER_TASK_HOUR;

  if (v.HasMember("costs")) {
    if (!v["costs"].IsObject()) {
      throw InputException(
        std::format("Invalid costs for vehicle {}.", v["id"].GetUint64()));
    }

    if (v["costs"].HasMember("fixed")) {
      if (!v["costs"]["fixed"].IsUint()) {
        throw InputException(std::format("Invalid fixed cost for vehicle {}.",
                                         v["id"].GetUint64()));
      }

      fixed = v["costs"]["fixed"].GetUint();
    }

    if (v["costs"].HasMember("per_hour")) {
      if (!v["costs"]["per_hour"].IsUint()) {
        throw InputException(
          std::format("Invalid per_hour cost for vehicle {}.",
                      v["id"].GetUint64()));
      }

      per_hour = v["costs"]["per_hour"].GetUint();
    }

    if (v["costs"].HasMember("per_km")) {
      if (!v["costs"]["per_km"].IsUint()) {
        throw InputException(std::format("Invalid per_km cost for vehicle {}.",
                                         v["id"].GetUint64()));
      }

      per_km = v["costs"]["per_km"].GetUint();
    }

    if (v["costs"].HasMember("per_task_hour")) {
      if (!v["costs"]["per_task_hour"].IsUint()) {
        throw InputException(
          std::format("Invalid per_task_hour cost for vehicle {}.",
                      v["id"].GetUint64()));
      }

      per_task_hour = v["costs"]["per_task_hour"].GetUint();
    }
  }

  return VehicleCosts(fixed, per_hour, per_km, per_task_hour);
}

inline std::vector<VehicleStep> get_vehicle_steps(const rapidjson::Value& v) {
  std::vector<VehicleStep> steps;

  if (v.HasMember("steps")) {
    if (!v["steps"].IsArray()) {
      throw InputException(
        std::format("Invalid steps for vehicle {}.", v["id"].GetUint64()));
    }

    steps.reserve(v["steps"].Size());

    for (rapidjson::SizeType i = 0; i < v["steps"].Size(); ++i) {
      const auto& json_step = v["steps"][i];

      std::optional<UserDuration> at;
      if (json_step.HasMember("service_at")) {
        if (!json_step["service_at"].IsUint()) {
          throw InputException("Invalid service_at value.");
        }

        at = json_step["service_at"].GetUint();
      }
      std::optional<UserDuration> after;
      if (json_step.HasMember("service_after")) {
        if (!json_step["service_after"].IsUint()) {
          throw InputException("Invalid service_after value.");
        }

        after = json_step["service_after"].GetUint();
      }
      std::optional<UserDuration> before;
      if (json_step.HasMember("service_before")) {
        if (!json_step["service_before"].IsUint()) {
          throw InputException("Invalid service_before value.");
        }

        before = json_step["service_before"].GetUint();
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

      if (!json_step.HasMember("id") || !json_step["id"].IsUint64()) {
        throw InputException(std::format("Invalid id in steps for vehicle {}.",
                                         v["id"].GetUint64()));
      }

      if (type_str == "job") {
        steps.emplace_back(JOB_TYPE::SINGLE,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else if (type_str == "pickup") {
        steps.emplace_back(JOB_TYPE::PICKUP,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else if (type_str == "delivery") {
        steps.emplace_back(JOB_TYPE::DELIVERY,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else if (type_str == "break") {
        steps.emplace_back(STEP_TYPE::BREAK,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else {
        throw InputException(
          std::format("Invalid type in steps for vehicle {}.",
                      v["id"].GetUint64()));
      }
    }
  }

  return steps;
}

inline Vehicle get_vehicle(const rapidjson::Value& json_vehicle,
                           unsigned amount_size) {
  check_id(json_vehicle, "vehicle");
  auto v_id = json_vehicle["id"].GetUint64();

  // Check what info are available for vehicle start, then build
  // optional start location.
  const bool has_start_coords = json_vehicle.HasMember("start");
  const bool has_start_index = json_vehicle.HasMember("start_index");
  if (has_start_index && !json_vehicle["start_index"].IsUint()) {
    throw InputException(
      std::format("Invalid start_index for vehicle {}.", v_id));
  }

  std::optional<Location> start;
  if (has_start_index) {
    // Custom provided matrices and index.
    const Index start_index = json_vehicle["start_index"].GetUint();
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
  const bool has_end_coords = json_vehicle.HasMember("end");
  const bool has_end_index = json_vehicle.HasMember("end_index");
  if (has_end_index && !json_vehicle["end_index"].IsUint()) {
    throw InputException(
      std::format("Invalid end_index for vehicle {}.", v_id));
  }

  std::optional<Location> end;
  if (has_end_index) {
    // Custom provided matrices and index.
    const Index end_index = json_vehicle["end_index"].GetUint();
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
                 get_amount(json_vehicle, "capacity", 0),
                 get_skills(json_vehicle),
                 get_vehicle_time_window(json_vehicle),
                 get_vehicle_breaks(json_vehicle, amount_size),
                 get_string(json_vehicle, "description"),
                 get_vehicle_costs(json_vehicle),
                 get_double(json_vehicle, "speed_factor"),
                 get_value_for<size_t>(json_vehicle, "max_tasks"),
                 get_value_for<UserDuration>(json_vehicle, "max_travel_time"),
                 get_value_for<UserDistance>(json_vehicle, "max_distance"),
                 get_vehicle_steps(json_vehicle),
                 get_string(json_vehicle, "type"));
}

inline Location get_task_location(const rapidjson::Value& v,
                                  const std::string& task_type) {
  // Check what info are available to build task location.
  const bool has_location_coords = v.HasMember("location");
  const bool has_location_index = v.HasMember("location_index");
  if (has_location_index && !v["location_index"].IsUint()) {
    throw InputException(std::format("Invalid location_index for {} {}.",
                                     task_type,
                                     v["id"].GetUint64()));
  }

  if (has_location_index) {
    // Custom provided matrices and index.
    const Index location_index = v["location_index"].GetUint();
    if (has_location_coords) {
      return Location({location_index, parse_coordinates(v, "location")});
    }
    return Location(location_index);
  }
  check_location(v, task_type);
  return Location(parse_coordinates(v, "location"));
}

inline Job get_job(const rapidjson::Value& json_job, unsigned amount_size) {
  check_id(json_job, "job");

  // Only for retro-compatibility: when no pickup and delivery keys
  // are defined and (deprecated) amount key is present, it should be
  // interpreted as a delivery.
  const bool need_amount_compat = json_job.HasMember("amount") &&
                                  !json_job.HasMember("delivery") &&
                                  !json_job.HasMember("pickup");

  return Job(json_job["id"].GetUint64(),
             get_task_location(json_job, "job"),
             get_duration(json_job, "setup"),
             get_duration(json_job, "service"),
             need_amount_compat ? get_amount(json_job, "amount", amount_size)
                                : get_amount(json_job, "delivery", amount_size),
             get_amount(json_job, "pickup", amount_size),
             get_skills(json_job),
             get_priority(json_job),
             get_time_windows(json_job, "job"),
             get_string(json_job, "description"),
             get_duration_per_type(json_job, "setup_per_type", "job"),
             get_duration_per_type(json_job, "service_per_type", "job"));
}

template <class T> inline Matrix<T> get_matrix(rapidjson::Value& m) {
  if (!m.IsArray()) {
    throw InputException("Invalid matrix.");
  }
  // Load custom matrix while checking if it is square.
  const rapidjson::SizeType matrix_size = m.Size();

  Matrix<T> matrix(matrix_size);
  for (rapidjson::SizeType i = 0; i < matrix_size; ++i) {
    if (!m[i].IsArray() || m[i].Size() != matrix_size) {
      throw InputException("Unexpected matrix line length.");
    }
    const rapidjson::Document::Array mi = m[i].GetArray();
    for (rapidjson::SizeType j = 0; j < matrix_size; ++j) {
      if (!mi[j].IsUint()) {
        throw InputException("Invalid matrix entry.");
      }
      matrix[i][j] = mi[j].GetUint();
    }
  }

  return matrix;
}

void parse(Input& input, const std::string& input_str, bool geometry) {
  // Input json object.
  rapidjson::Document json_input;

  // Parsing input string to populate the input object.
  if (json_input.Parse(input_str.c_str()).HasParseError()) {
    const std::string error_msg =
      std::format("{} (offset: {})",
                  rapidjson::GetParseError_En(json_input.GetParseError()),
                  json_input.GetErrorOffset());
    throw InputException(error_msg);
  }

  // Main checks for valid json input.
  if (!json_input.IsObject()) {
    throw InputException("Input root is not an object.");
  }

  if (!json_input.HasMember("vehicles") || !json_input["vehicles"].IsArray()) {
    throw InputException("Invalid vehicles.");
  }
  if (json_input["vehicles"].Empty()) {
    // This is tested upstream upon solving but we still need to do it
    // here to access first vehicle and retrieve amount_size.
    throw InputException("No vehicle defined.");
  }

  const auto& first_vehicle = json_input["vehicles"][0];
  check_id(first_vehicle, "vehicle");
  const bool first_vehicle_has_capacity =
    (first_vehicle.HasMember("capacity") &&
     first_vehicle["capacity"].IsArray() && !first_vehicle["capacity"].Empty());
  const unsigned amount_size =
    first_vehicle_has_capacity ? first_vehicle["capacity"].Size() : 0;

  input.set_geometry(geometry);

  // Add all vehicles.
  for (rapidjson::SizeType i = 0; i < json_input["vehicles"].Size(); ++i) {
    auto& json_vehicle = json_input["vehicles"][i];

    input.add_vehicle(get_vehicle(json_vehicle, amount_size));
  }

  if (json_input.HasMember("jobs")) {
    if (!json_input["jobs"].IsArray()) {
      throw InputException("Invalid jobs.");
    }

    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      input.add_job(get_job(json_input["jobs"][i], amount_size));
    }
  }

  if (json_input.HasMember("shipments")) {
    if (!json_input["shipments"].IsArray()) {
      throw InputException("Invalid shipments.");
    }

    for (rapidjson::SizeType i = 0; i < json_input["shipments"].Size(); ++i) {
      auto& json_shipment = json_input["shipments"][i];
      check_shipment(json_shipment);

      // Retrieve common stuff for both pickup and delivery.
      auto amount = get_amount(json_shipment, "amount", amount_size);
      auto skills = get_skills(json_shipment);
      auto priority = get_priority(json_shipment);

      // Defining pickup job.
      auto& json_pickup = json_shipment["pickup"];
      check_id(json_pickup, "pickup");

      const Job pickup(json_pickup["id"].GetUint64(),
                       JOB_TYPE::PICKUP,
                       get_task_location(json_pickup, "pickup"),
                       get_duration(json_pickup, "setup"),
                       get_duration(json_pickup, "service"),
                       amount,
                       skills,
                       priority,
                       get_time_windows(json_pickup, "pickup"),
                       get_string(json_pickup, "description"),
                       get_duration_per_type(json_pickup,
                                             "setup_per_type",
                                             "pickup"),
                       get_duration_per_type(json_pickup,
                                             "service_per_type",
                                             "pickup"));

      // Defining delivery job.
      auto& json_delivery = json_shipment["delivery"];
      check_id(json_delivery, "delivery");

      const Job delivery(json_delivery["id"].GetUint64(),
                         JOB_TYPE::DELIVERY,
                         get_task_location(json_delivery, "delivery"),
                         get_duration(json_delivery, "setup"),
                         get_duration(json_delivery, "service"),
                         amount,
                         skills,
                         priority,
                         get_time_windows(json_delivery, "delivery"),
                         get_string(json_delivery, "description"),
                         get_duration_per_type(json_delivery,
                                               "setup_per_type",
                                               "delivery"),
                         get_duration_per_type(json_delivery,
                                               "service_per_type",
                                               "delivery"));

      input.add_shipment(pickup, delivery);
    }
  }

  if (json_input.HasMember("matrices")) {
    if (!json_input["matrices"].IsObject()) {
      throw InputException("Unexpected matrices value.");
    }
    for (auto& profile_entry : json_input["matrices"].GetObject()) {
      if (profile_entry.value.IsObject()) {
        if (profile_entry.value.HasMember("durations")) {
          input.set_durations_matrix(profile_entry.name.GetString(),
                                     get_matrix<UserDuration>(
                                       profile_entry.value["durations"]));
        }
        if (profile_entry.value.HasMember("distances")) {
          input.set_distances_matrix(profile_entry.name.GetString(),
                                     get_matrix<UserDistance>(
                                       profile_entry.value["distances"]));
        }
        if (profile_entry.value.HasMember("costs")) {
          input.set_costs_matrix(profile_entry.name.GetString(),
                                 get_matrix<UserCost>(
                                   profile_entry.value["costs"]));
        }
      }
    }
  } else {
    // Deprecated `matrix` key still interpreted as
    // `matrices.DEFAULT_PROFILE.duration` for retro-compatibility.
    if (json_input.HasMember("matrix")) {
      input.set_durations_matrix(DEFAULT_PROFILE,
                                 get_matrix<UserDuration>(
                                   json_input["matrix"]));
    }
  }
}

} // namespace vroom::io
