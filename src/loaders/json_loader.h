#ifndef JSON_LOADER_H
#define JSON_LOADER_H

/*

This file is a provided patch for VROOM by PattyDePuh.

*/

#include <vector>
#include <cassert>
#include <boost/regex.hpp>
#include <sstream>
#include <cmath>
#include "./problem_io.h"
#include "../structures/typedefs.h"
#include "../structures/matrix.h"
#include "../../include/rapidjson/document.h"

class json_loader : public problem_io<distance_t>{

private:

  std::size_t _dimension;       // Number of Nodes
  rapidjson::Document _dom;                // the JSON object itself

public:
  json_loader(const std::string& input){
    //
    if(_dom.Parse(input.c_str()).HasParseError()){
      std::string error_msg = std::string(rapidjson::GetParseError_En(_dom.GetParseError()))
        + " (offset: "
        + std::to_string(_dom.GetErrorOffset())
        + ")";
      throw custom_exception(error_msg);
    }
    //Quadartic Test.
    rapidjson::SizeType rows = _dom["matrix"].Size();
    for (rapidjson::SizeType i = 0; i < rows; i++){
      if ( _dom["matrix"][i].Size() != rows ){
        std::string error_msg = "[Error] Supplied matrix is not complete!";
        throw custom_exception(error_msg);
      }
    }
    _dimension = (std::size_t)rows;
    //Vehicle-id for ... don't know?
    _vehicle_id = _dom["vehicles"][0]["id"].GetUint();
    //Check for strict start/end point.
    if(_dom["vehicles"][0]["start"].IsUint()){
      _pbl_context.force_start = true;
      _pbl_context.start = _dom["vehicles"][0]["start"].GetUint();
    }
    if(_dom["vehicles"][0]["start"].IsUint()){
      _pbl_context.force_end = true;
      _pbl_context.end = _dom["vehicles"][0]["end"].GetUint();
    }  
  }


  virtual matrix<distance_t> get_matrix() const override{
    matrix<distance_t> m {_dimension};

    // Reading from DOM
    bool is_uint = _dom["matrix"][0][0].IsUint();
    for(std::size_t i = 0; i < _dimension; ++i){
      for(std::size_t j = 0; j < _dimension; ++j){
        if(is_uint)
          m[i][j] = _dom["matrix"][i][j].GetUint(); //Hier ansetzen wegen Fließkommata
        else
          m[i][j] = _dom["matrix"][i][j].GetDouble();
      }
    }
    return m;
  }


  virtual void get_steps(const std::list<index_t>& steps,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    for( index_t step : steps){
      value.PushBack( step, allocator );
    }
  }

  virtual void get_route_infos(const std::list<index_t>&,
                               rapidjson::Value&,
                               rapidjson::Document::AllocatorType&) const{

  }
};

#endif