/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./input_parser.h"

// Helper to get optional array of coordinates.
inline std::array<coordinate_t, 2>
parse_coordinates(const rapidjson::Value& object, const char* key) {
  if (!object[key].IsArray() or (object[key].Size() < 2) or
      !object[key][0].IsNumber() or !object[key][1].IsNumber()) {
    throw custom_exception("Invalid " + std::string(key) + " array.");
  }
  return {object[key][0].GetDouble(), object[key][1].GetDouble()};
}

inline amount_t parse_amount(const rapidjson::Value& object, const char* key) {
  if (!object[key].IsArray()) {
    throw custom_exception("Invalid " + std::string(key) + " array.");
  }
  amount_t amount;
  for (rapidjson::SizeType i = 0; i < object[key].Size(); ++i) {
    if (!object[key][i].IsInt64()) {
      throw custom_exception("Invalid " + std::string(key) + " value.");
    }
    amount.push_back(object[key][i].GetInt64());
  }

  return amount;
}

inline boost::optional<amount_t> get_amount(const rapidjson::Value& object,
                                            const char* key) {
  if (!object.HasMember(key)) {
    return boost::none;
  }

  if (!object[key].IsArray()) {
    throw custom_exception("Invalid " + std::string(key) + " array.");
  }
  amount_t amount;
  for (rapidjson::SizeType i = 0; i < object[key].Size(); ++i) {
    if (!object[key][i].IsInt64()) {
      throw custom_exception("Invalid " + std::string(key) + " value.");
    }
    amount.push_back(object[key][i].GetInt64());
  }

  return amount;
}

inline bool valid_vehicle(const rapidjson::Value& v) {
  return v.IsObject() and v.HasMember("id") and v["id"].IsUint64();
}

input parse(const cl_args_t& cl_args) {
  BOOST_LOG_TRIVIAL(info) << "[Loading] Parsing input.";

  // Set relevant wrapper to retrieve the matrix and geometry.
  std::unique_ptr<routing_io<cost_t>> routing_wrapper;
  if (!cl_args.use_libosrm) {
    // Use osrm-routed.
    routing_wrapper = std::make_unique<routed_wrapper>(cl_args.osrm_address,
                                                       cl_args.osrm_port,
                                                       cl_args.osrm_profile);
  } else {
#if LIBOSRM
    // Use libosrm.
    if (cl_args.osrm_profile.empty()) {
      throw custom_exception("-l flag requires -m.");
    }
    routing_wrapper = std::make_unique<libosrm_wrapper>(cl_args.osrm_profile);
#else
    throw custom_exception("libosrm must be installed to use -l.");
#endif
  }

  // Custom input object embedding jobs, vehicles and matrix.
  input input_data(std::move(routing_wrapper), cl_args.geometry);

  // Input json object.
  rapidjson::Document json_input;

  // Parsing input string to populate the input object.
  if (json_input.Parse(cl_args.input.c_str()).HasParseError()) {
    std::string error_msg =
      std::string(rapidjson::GetParseError_En(json_input.GetParseError())) +
      " (offset: " + std::to_string(json_input.GetErrorOffset()) + ")";
    throw custom_exception(error_msg);
  }

  // Main Checks for valid json input.
  if (!json_input.HasMember("jobs") or !json_input["jobs"].IsArray()) {
    throw custom_exception("Invalid jobs.");
  }

  if (!json_input.HasMember("vehicles") or !json_input["vehicles"].IsArray() or
      json_input["vehicles"].Empty()) {
    throw custom_exception("Invalid vehicles.");
  }

  // Switch input type: explicit matrix or using OSRM.
  if (json_input.HasMember("matrix")) {
    if (!json_input["matrix"].IsArray()) {
      throw custom_exception("Invalid matrix.");
    }

    // Load custom matrix while checking if it is square.
    rapidjson::SizeType matrix_size = json_input["matrix"].Size();

    input_data._max_cost_per_line.assign(matrix_size, 0);
    input_data._max_cost_per_column.assign(matrix_size, 0);

    matrix<cost_t> matrix_input(matrix_size);
    for (rapidjson::SizeType i = 0; i < matrix_size; ++i) {
      if (!json_input["matrix"][i].IsArray() or
          (json_input["matrix"][i].Size() != matrix_size)) {
        throw custom_exception("Invalid matrix line " + std::to_string(i) +
                               ".");
      }
      for (rapidjson::SizeType j = 0; j < matrix_size; ++j) {
        if (!json_input["matrix"][i][j].IsUint()) {
          throw custom_exception("Invalid matrix entry (" + std::to_string(i) +
                                 "," + std::to_string(j) + ").");
        }
        cost_t cost = json_input["matrix"][i][j].GetUint();
        matrix_input[i][j] = cost;
        input_data._max_cost_per_line[i] =
          std::max(input_data._max_cost_per_line[i], cost);
        input_data._max_cost_per_column[j] =
          std::max(input_data._max_cost_per_column[j], cost);
      }
    }
    input_data._matrix = matrix_input;

    // Add all vehicles.
    for (rapidjson::SizeType i = 0; i < json_input["vehicles"].Size(); ++i) {
      if (!valid_vehicle(json_input["vehicles"][i])) {
        throw custom_exception("Invalid vehicle at " + std::to_string(i) + ".");
      }
      auto v_id = json_input["vehicles"][i]["id"].GetUint();

      // Check if vehicle has start_index or end_index.
      bool has_start_index = json_input["vehicles"][i].HasMember("start_index");
      index_t start_index = 0; // Initial value actually never used.
      if (has_start_index) {
        if (!json_input["vehicles"][i]["start_index"].IsUint()) {
          throw custom_exception("Invalid start_index for vehicle " +
                                 std::to_string(v_id) + ".");
        }
        start_index = json_input["vehicles"][i]["start_index"].GetUint();

        if (matrix_size <= start_index) {
          throw custom_exception(
            "start_index exceeding matrix size for vehicle" +
            std::to_string(v_id) + ".");
        }
      }

      bool has_start_coords = json_input["vehicles"][i].HasMember("start");

      bool has_end_index = json_input["vehicles"][i].HasMember("end_index");
      index_t end_index = 0; // Initial value actually never used.
      if (has_end_index) {
        if (!json_input["vehicles"][i]["end_index"].IsUint()) {
          throw custom_exception("Invalid end_index for vehicle" +
                                 std::to_string(v_id) + ".");
        }
        end_index = json_input["vehicles"][i]["end_index"].GetUint();

        if (matrix_size <= end_index) {
          throw custom_exception("end_index exceeding matrix size for vehicle" +
                                 std::to_string(v_id) + ".");
        }
      }

      bool has_end_coords = json_input["vehicles"][i].HasMember("end");

      // Add vehicle to input
      boost::optional<location_t> start;
      if (has_start_index) {
        if (has_start_coords) {
          start = boost::optional<location_t>(location_t(
            {start_index,
             parse_coordinates(json_input["vehicles"][i], "start")}));
        } else {
          start = boost::optional<location_t>(start_index);
        }
      }

      boost::optional<location_t> end;
      if (has_end_index) {
        if (has_end_coords) {
          end = boost::optional<location_t>(location_t(
            {end_index, parse_coordinates(json_input["vehicles"][i], "end")}));
        } else {
          end = boost::optional<location_t>(end_index);
        }
      }

      vehicle_t current_v(v_id,
                          start,
                          end,
                          get_amount(json_input["vehicles"][i], "capacity"));

      input_data.add_vehicle(current_v);
    }

    // Add the jobs
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      if (!json_input["jobs"][i].IsObject()) {
        throw custom_exception("Invalid job.");
      }
      if (!json_input["jobs"][i].HasMember("id") or
          !json_input["jobs"][i]["id"].IsUint64()) {
        throw custom_exception("Invalid id for job at " + std::to_string(i) +
                               ".");
      }
      auto j_id = json_input["jobs"][i]["id"].GetUint64();
      if (!json_input["jobs"][i].HasMember("location_index") or
          !json_input["jobs"][i]["location_index"].IsUint()) {
        throw custom_exception("Invalid location_index for job " +
                               std::to_string(j_id) + ".");
      }
      if (matrix_size <= json_input["jobs"][i]["location_index"].GetUint()) {
        throw custom_exception("location_index exceeding matrix size for job " +
                               std::to_string(j_id) + ".");
      }

      if (json_input["jobs"][i].HasMember("location")) {
        job_t current_job(j_id,
                          get_amount(json_input["jobs"][i], "amount"),
                          json_input["jobs"][i]["location_index"].GetUint(),
                          parse_coordinates(json_input["jobs"][i], "location"));
        input_data.add_job(current_job);
      } else {
        job_t current_job(json_input["jobs"][i]["id"].GetUint64(),
                          get_amount(json_input["jobs"][i], "amount"),
                          json_input["jobs"][i]["location_index"].GetUint());
        input_data.add_job(current_job);
      }
    }
  } else {
    // Adding vehicles and jobs only, matrix will be computed using
    // OSRM upon solving.

    // All vehicles.
    for (rapidjson::SizeType i = 0; i < json_input["vehicles"].Size(); ++i) {
      if (!valid_vehicle(json_input["vehicles"][i])) {
        throw custom_exception("Invalid vehicle at " + std::to_string(i) + ".");
      }

      // Start def is a ugly workaround as using plain:
      //
      // boost::optional<location_t> start;
      //
      // will raise a false positive -Wmaybe-uninitialized with gcc,
      // see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=47679

      auto start([]() -> boost::optional<location_t> { return boost::none; }());
      if (json_input["vehicles"][i].HasMember("start")) {
        start = boost::optional<location_t>(
          parse_coordinates(json_input["vehicles"][i], "start"));
      }

      boost::optional<location_t> end;
      if (json_input["vehicles"][i].HasMember("end")) {
        end = boost::optional<location_t>(
          parse_coordinates(json_input["vehicles"][i], "end"));
      }

      vehicle_t current_v(json_input["vehicles"][i]["id"].GetUint(),
                          start,
                          end,
                          get_amount(json_input["vehicles"][i], "capacity"));

      input_data.add_vehicle(current_v);
    }

    // Getting jobs.
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      if (!json_input["jobs"][i].IsObject()) {
        throw custom_exception("Invalid job.");
      }
      if (!json_input["jobs"][i].HasMember("id") or
          !json_input["jobs"][i]["id"].IsUint64()) {
        throw custom_exception("Invalid id for job at " + std::to_string(i) +
                               ".");
      }
      auto j_id = json_input["jobs"][i]["id"].GetUint64();
      if (!json_input["jobs"][i].HasMember("location") or
          !json_input["jobs"][i]["location"].IsArray()) {
        throw custom_exception("Invalid location for job " +
                               std::to_string(j_id) + ".");
      }

      job_t current_job(j_id,
                        get_amount(json_input["jobs"][i], "amount"),
                        parse_coordinates(json_input["jobs"][i], "location"));

      input_data.add_job(current_job);
    }
  }

  if (input_data._locations.size() <= 1) {
    throw custom_exception("At least two locations required!");
  }

  return input_data;
}
