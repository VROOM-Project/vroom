/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TSPLIB_LOADER_H
#define TSPLIB_LOADER_H
#include <vector>
// #include <cmath>
#include <regex>
#include "./problem_io.h"
#include "../structures/typedefs.h"
#include "../structures/matrix.h"

class tsplib_loader : public problem_io<distance_t>{

private:
  // Supported EDGE_WEIGHT_TYPE values.
  enum class EWT {NONE, EXPLICIT, EUC_2D, CEIL_2D, GEO};
  // Supported EDGE_WEIGHT_FORMAT values.
  enum class EWF {NONE, FULL_MATRIX, UPPER_ROW, UPPER_DIAG_ROW};

  static distance_t nint(double x){
    return static_cast<distance_t>(x + 0.5);
  }

  std::size_t _dimension;
  EWT _ewt;                     // Edge weight type.
  EWF _ewf;                     // Edge weight format.
  std::string _data_section;    // either NODE_COORD_SECTION or
                                // EDGE_WEIGHT_SECTION content.
  
public:
  tsplib_loader(std::string input):
    _ewt(EWT::NONE),
    _ewf(EWF::NONE){
    // 1. Get problem dimension.
    std::regex dim_rgx ("DIMENSION[[:space:]]*:[[:space:]]*([0-9]+)[[:space:]]");
    std::smatch dim_match;
    std::regex_search(input, dim_match, dim_rgx);
    if(dim_match.size() != 2){
      throw custom_exception("incorrect \"DIMENSION\" key.");
    }
    _dimension = std::stoul(dim_match[1].str());

    // 2. Get edge weight type.
    std::regex ewt_rgx ("EDGE_WEIGHT_TYPE[[:space:]]*:[[:space:]]*([A-Z]+(_2D)?)[[:space:]]");
    std::smatch ewt_match;
    if(!std::regex_search(input, ewt_match, ewt_rgx)){
      throw custom_exception("incorrect \"EDGE_WEIGHT_TYPE\".");
    }
    std::string type = ewt_match[1].str();
    if(type == "EXPLICIT"){
      _ewt = EWT::EXPLICIT;
      std::cout << "EDGE_WEIGHT_TYPE explicit" << std::endl;
    }
    if(type == "EUC_2D"){
      _ewt = EWT::EUC_2D;
      std::cout << "EDGE_WEIGHT_TYPE euc_2D" << std::endl;
    }
    if(type == "CEIL_2D"){
      _ewt = EWT::CEIL_2D;
      std::cout << "EDGE_WEIGHT_TYPE ceil_2D" << std::endl;
    }
    if(type == "GEO"){
      _ewt = EWT::GEO;
      std::cout << "EDGE_WEIGHT_TYPE geo" << std::endl;
    }
    if(_ewt == EWT::NONE){
     throw custom_exception("unsupported \"EDGE_WEIGHT_TYPE\" value: "
                            + type +".");
    }
    // 2. Get edge weight format if required.
    if(_ewt == EWT::EXPLICIT){
      std::regex ewf_rgx ("EDGE_WEIGHT_FORMAT[[:space:]]*:[[:space:]]*([A-Z]+(_[A-Z]+){1,2})[[:space:]]");
      std::smatch ewf_match;
      if(!std::regex_search(input, ewf_match, ewf_rgx)){
        throw custom_exception("incorrect \"EDGE_WEIGHT_FORMAT\".");
      }
      std::string format = ewf_match[1].str();
      if(format == "FULL_MATRIX"){
        _ewf = EWF::FULL_MATRIX;
        std::cout << "EDGE_WEIGHT_FORMAT full_matrix" << std::endl;
      }
      if(format == "UPPER_ROW"){
        _ewf = EWF::UPPER_ROW;
        std::cout << "EDGE_WEIGHT_FORMAT upper_row" << std::endl;
      }
      if(format == "UPPER_DIAG_ROW"){
        _ewf = EWF::UPPER_DIAG_ROW;
        std::cout << "EDGE_WEIGHT_FORMAT upper_diag_row" << std::endl;
      }
      if(_ewf == EWF::NONE){
        throw custom_exception("unsupported \"EDGE_WEIGHT_FORMAT\" value: "
                               + format +".");
      }
    }
    // 3. Getting data section.
    if(_ewt == EWT::EXPLICIT){
      // Looking for an edge weight section.
      std::regex ews_rgx ("EDGE_WEIGHT_SECTION[[:space:]]*(([0-9]+[[:space:]]+)+)");
      std::smatch ews_match;
      if(!std::regex_search(input, ews_match, ews_rgx)){
        throw custom_exception("incorrect \"EDGE_WEIGHT_SECTION\".");
      }
      _data_section = ews_match[1].str();
    }
    else{
      // Looking for a node coord section.
      std::regex ews_rgx ("NODE_COORD_SECTION[[:space:]]+(([0-9]+[[:space:]]+(-?[0-9]*[.][0-9]*[[:space:]]+){2})+)");
      std::smatch ews_match;
      if(!std::regex_search(input, ews_match, ews_rgx)){
        throw custom_exception("incorrect \"NODE_COORD_SECTION\".");
      }
      _data_section = ews_match[1].str();
    }
    
    
    std::cout << _data_section << std::endl;
    exit(0);
  }

  virtual matrix<distance_t> get_matrix() const override{
    matrix<distance_t> m {_dimension};

    return m;
  }

  virtual std::string get_route(const std::list<index_t>& tour) const override{
    return "";
  }

  virtual std::string get_route_geometry(const std::list<index_t>& tour) const{
    return "";
  }
};

#endif
