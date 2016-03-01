#ifndef TSPLIB_LOADER_H
#define TSPLIB_LOADER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <cassert>
#include <boost/regex.hpp>
#include <sstream>
#include <cmath>
#include "./problem_io.h"
#include "../structures/typedefs.h"
#include "../structures/matrix.h"

class tsplib_loader : public problem_io<distance_t>{

private:
  // Supported EDGE_WEIGHT_TYPE values.
  enum class EWT {NONE, EXPLICIT, EUC_2D, CEIL_2D, GEO, ATT};
  // Supported EDGE_WEIGHT_FORMAT values.
  enum class EWF {NONE, FULL_MATRIX, UPPER_ROW, UPPER_DIAG_ROW, LOWER_DIAG_ROW};

  struct Node {index_t index; double x; double y;};

  static distance_t nint(double x){
    return (int) (x + 0.5);
  }

  static distance_t euc_2D(Node i, Node j){
    double xd = i.x - j.x;
    double yd = i.y - j.y;
    return nint(std::sqrt(xd*xd + yd*yd));
  }

  static distance_t ceil_2D(Node i, Node j){
    double xd = i.x - j.x;
    double yd = i.y - j.y;
    return std::ceil(std::sqrt(xd*xd + yd*yd));
  }

  static distance_t att(Node i, Node j){
    double xd = i.x - j.x;
    double yd = i.y - j.y;
    double r = std::sqrt((xd*xd + yd*yd) / 10.0);
    distance_t t = nint(r);
    distance_t result;
    if(t < r){
      result = t + 1;
    }
    else{
      result = t;
    }
    return result;
  }

  static double constexpr PI = 3.141592;

  static distance_t geo(Node i, Node j){
    // Geographical latitude and longitude in radians for i.
    int deg = (int) i.x;
    double min = i.x - deg;
    double lat_i = PI * (deg + 5.0 * min / 3.0 ) / 180.0;
    deg = (int) i.y;
    min = i.y - deg;
    double lon_i = PI * (deg + 5.0 * min / 3.0 ) / 180.0;
    // Geographical latitude and longitude in radians for j.
    deg = (int) j.x;
    min = j.x - deg;
    double lat_j = PI * (deg + 5.0 * min / 3.0 ) / 180.0;
    deg = (int) j.y;
    min = j.y - deg;
    double lon_j = PI * (deg + 5.0 * min / 3.0 ) / 180.0;
    // Computing distance.
    double q1 = std::cos(lon_i - lon_j);
    double q2 = std::cos(lat_i - lat_j);
    double q3 = std::cos(lat_i + lat_j);
    return (int) (6378.388 * std::acos(0.5*((1.0+q1)*q2 - (1.0-q1)*q3)) + 1.0);
  }

  std::size_t _dimension;
  EWT _ewt;                     // Edge weight type.
  EWF _ewf;                     // Edge weight format.
  std::string _data_section;    // either NODE_COORD_SECTION or
                                // EDGE_WEIGHT_SECTION content.
  std::vector<Node> _nodes;     // Nodes with coords.

public:
  tsplib_loader(std::string input):
    _ewt(EWT::NONE),
    _ewf(EWF::NONE) {
    // 1. Get problem dimension.
    boost::regex dim_rgx ("DIMENSION[[:space:]]*:[[:space:]]*([0-9]+)[[:space:]]");
    boost::smatch dim_match;
    if(!boost::regex_search(input, dim_match, dim_rgx)){
      throw custom_exception("Incorrect \"DIMENSION\" key.");
    }
    _dimension = std::stoul(dim_match[1].str());
    if(_dimension <= 1){
      throw custom_exception("At least two locations required!");
    }

    // 2. Get edge weight type.
    boost::regex ewt_rgx ("EDGE_WEIGHT_TYPE[[:space:]]*:[[:space:]]*([A-Z]+(_2D)?)[[:space:]]");
    boost::smatch ewt_match;
    if(!boost::regex_search(input, ewt_match, ewt_rgx)){
      throw custom_exception("Incorrect \"EDGE_WEIGHT_TYPE\".");
    }
    std::string type = ewt_match[1].str();
    if(type == "EXPLICIT"){
      _ewt = EWT::EXPLICIT;
    }
    if(type == "EUC_2D"){
      _ewt = EWT::EUC_2D;
    }
    if(type == "CEIL_2D"){
      _ewt = EWT::CEIL_2D;
    }
    if(type == "GEO"){
      _ewt = EWT::GEO;
    }
    if(type == "ATT"){
      _ewt = EWT::ATT;
    }
    if(_ewt == EWT::NONE){
     throw custom_exception("Unsupported \"EDGE_WEIGHT_TYPE\" value: "
                            + type +".");
    }
    // 2. Get edge weight format if required.
    if(_ewt == EWT::EXPLICIT){
      boost::regex ewf_rgx ("EDGE_WEIGHT_FORMAT[[:space:]]*:[[:space:]]*([A-Z]+(_[A-Z]+){1,2})[[:space:]]");
      boost::smatch ewf_match;
      if(!boost::regex_search(input, ewf_match, ewf_rgx)){
        throw custom_exception("Incorrect \"EDGE_WEIGHT_FORMAT\".");
      }
      std::string format = ewf_match[1].str();
      if(format == "FULL_MATRIX"){
        _ewf = EWF::FULL_MATRIX;
      }
      if(format == "UPPER_ROW"){
        _ewf = EWF::UPPER_ROW;
      }
      if(format == "UPPER_DIAG_ROW"){
        _ewf = EWF::UPPER_DIAG_ROW;
      }
      if(format == "LOWER_DIAG_ROW"){
        _ewf = EWF::LOWER_DIAG_ROW;
      }
      if(_ewf == EWF::NONE){
        throw custom_exception("Unsupported \"EDGE_WEIGHT_FORMAT\" value: "
                               + format +".");
      }
    }
    // 3. Getting data section.
    if(_ewt == EWT::EXPLICIT){
      // Looking for an edge weight section.
      boost::regex ews_rgx ("EDGE_WEIGHT_SECTION[[:space:]]*(.+)[[:space:]]*(EOF)?");
      boost::smatch ews_match;
      if(!boost::regex_search(input, ews_match, ews_rgx)){
        throw custom_exception("Incorrect \"EDGE_WEIGHT_SECTION\".");
      }
      _data_section = ews_match[1].str();
    }
    else{
      // Looking for a node coord section.
      boost::regex ews_rgx ("NODE_COORD_SECTION[[:space:]]*(.+)[[:space:]]*(EOF)?");
      boost::smatch ews_match;
      if(!boost::regex_search(input, ews_match, ews_rgx)){
        throw custom_exception("Incorrect \"NODE_COORD_SECTION\".");
      }
      _data_section = ews_match[1].str();
    }

    if(_ewt != EWT::EXPLICIT){
      // Parsing nodes.
      std::istringstream data (_data_section);
      for(std::size_t i = 0; i < _dimension; ++i){
        index_t index;
        double x,y;
        data >> index >> x >> y;
        _nodes.push_back({index, x, y});
      }
    }
  }

  virtual matrix<distance_t> get_matrix() const override{
    matrix<distance_t> m {_dimension};

    std::istringstream data (_data_section);

    if(_ewt == EWT::EXPLICIT){
      switch (_ewf){
      case EWF::FULL_MATRIX: {
        // Reading from input data.
        for(std::size_t i = 0; i < _dimension; ++i){
          for(std::size_t j = 0; j < _dimension; ++j){
            data >> m[i][j];
          }
        }
        // Zeros on the diagonal for further undirected graph build.
        for(std::size_t i = 0; i < _dimension; ++i){
          m[i][i] = 0;
        }
        break;
      }
      case EWF::UPPER_ROW: {
        // Reading from input data.
        distance_t current_value;              
        for(std::size_t i = 0; i < _dimension - 1; ++i){
          for(std::size_t j = i + 1; j < _dimension; ++j){
            data >> current_value;
            m[i][j] = current_value;
            m[j][i] = current_value;
          }
        }
        // Zeros on the diagonal for further undirected graph build.
        for(std::size_t i = 0; i < _dimension; ++i){
          m[i][i] = 0;
        }
        break;
      }
      case EWF::UPPER_DIAG_ROW:{
        // Reading from input data.
        distance_t current_value;              
        for(std::size_t i = 0; i < _dimension; ++i){
          for(std::size_t j = i; j < _dimension; ++j){
            data >> current_value;
            m[i][j] = current_value;
            m[j][i] = current_value;
          }
        }
        // Zeros on the diagonal for further undirected graph build.
        for(std::size_t i = 0; i < _dimension; ++i){
          m[i][i] = 0;
        }
        break;
      }
      case EWF::LOWER_DIAG_ROW:{
        // Reading from input data.
        distance_t current_value;              
        for(std::size_t i = 0; i < _dimension; ++i){
          for(std::size_t j = 0; j <= i ; ++j){
            data >> current_value;
            m[i][j] = current_value;
            m[j][i] = current_value;
          }
        }
        // Zeros on the diagonal for further undirected graph build.
        for(std::size_t i = 0; i < _dimension; ++i){
          m[i][i] = 0;
        }
        break;
      }
      case EWF::NONE:
        // Should not happen!
        assert(false);
        break;
      }
    }
    else{
      // Using a pointer to the appropriate member function for
      // distance computing.
      distance_t (*dist_f_ptr) (Node, Node)
        = &tsplib_loader::euc_2D;
      switch (_ewt){
      case EWT::EUC_2D:
        // dist_f_ptr already initialized.
        break;
      case EWT::CEIL_2D:
        dist_f_ptr = &tsplib_loader::ceil_2D;
        break;
      case EWT::GEO:
        dist_f_ptr = &tsplib_loader::geo;
        break;
      case EWT::ATT:
        dist_f_ptr = &tsplib_loader::att;
        break;
      default:
        // Should not happen!
        assert(false);
      }
      // Computing symmetric matrix.
      distance_t current_value;              
      for(std::size_t i = 0; i < _dimension; ++i){
        m[i][i] = 0;
        for(std::size_t j = i + 1; j < _dimension ; ++j){
          current_value = (*dist_f_ptr)(_nodes[i], _nodes[j]);
          m[i][j] = current_value;
          m[j][i] = current_value;
        }
      }
    }
    return m;
  }

  virtual void get_route(const std::list<index_t>& tour,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value route_array(rapidjson::kArrayType);
    if((_ewt != EWT::NONE) and (_ewt != EWT::EXPLICIT)){
      // The key "route" is only added if the matrix has been computed
      // from the detailed list of nodes, in that case contained in
      // _nodes.
      for(auto const& step: tour){
        route_array
          .PushBack(rapidjson::Value(rapidjson::kArrayType)
                    .PushBack(_nodes[step].x, allocator)
                    .PushBack(_nodes[step].y, allocator),
                    allocator);
      }
    }
    value.Swap(route_array);
  }

  virtual void get_tour(const std::list<index_t>& tour,
                        rapidjson::Value& value,
                        rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value tour_array(rapidjson::kArrayType);
    for(auto const& step: tour){
      if(_ewt == EWT::EXPLICIT){
        // Using step when matrix is explicit.
        tour_array.PushBack(step, allocator);
      }
      else{
        // Using index provided in the file to describe places.
        tour_array.PushBack(_nodes[step].index, allocator);
      }
    }
    value.Swap(tour_array);
  }

  virtual void get_route_infos(const std::list<index_t>& tour,
                               rapidjson::Document& output) const{}
};

#endif
