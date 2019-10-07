/*
 * File: xviz_metadata_builder.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:52:29 pm
 */

#include "platform/xviz/xviz_metadata_builder.h"


using namespace mellocolate;
using Json = nlohmann::json;

XVIZMetaDataBuilder& XVIZMetaDataBuilder::AddStream(metadata::Stream stream) {
  streams_.push_back(std::move(stream));
  return *this;
}

XVIZMetaDataBuilder& XVIZMetaDataBuilder::AddUIConfig(metadata::UIConfig ui_config) {
  ui_config_ = std::move(ui_config);
}

std::string XVIZMetaDataBuilder::GetMetaData() {
  Json json;
  json["type"] = "xviz/metadata";
  json["data"]["version"] = version_;
  if (map_ != boost::none) {
    json["data"]["map"] = map_.value();
  }
  if (ui_config_ != boost::none) {
    json["data"]["ui_config"] = ui_config_.value().GetMetaData();
  }
  for (const auto& stream : streams_) {
    //std::string stream_name = stream.GetName();
    json["data"]["streams"][stream.GetName()] = stream.GetMetaData();
    /*
    if (stream.category_ != boost::none) {
      std::string category = stream.category_.value();
      json["data"]["streams"][stream.stream_name_]["category"] = category;
      if (category == "primitive" || category == "PRIMITIVE") {
        if (stream.type_ != boost::none) {
          json["data"]["streams"][stream.stream_name_]["primitive_type"] = stream.type_.value();
        }
      } else if (category == "variable" || category == "VARIABLE" ||
                 category == "time_series" || category == "TIME_SERIES") {
        if (stream.type_ != boost::none) {
          json["data"]["streams"][stream.stream_name_]["scalar_type"] = stream.type_.value();
        }
      }
    }
    if (stream.coordinate_ != boost::none) {
      json["data"]["streams"][stream.stream_name_]["coordinate"] = stream.coordinate_.value();
    }
    if (stream.stream_style_ != boost::none) {
      auto stream_style = stream.stream_style_.value();
      if (stream_style.fill_color_ != boost::none) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["fill_color"] = stream_style.fill_color_.value();
      }
      if (stream_style.height_ != boost::none) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["height"] = stream_style.height_.value();
      }
      if (stream_style.stroked_ != boost::none) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["stroked"] = (stream_style.stroked_.value() ? "true" : "false");
      }
      if (stream_style.extruded_ != boost::none) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["extruded"] = (stream_style.extruded_.value() ? "true" : "false");
      }
    }
    */
  }
  return json.dump();
}

XVIZMetaDataBuilder& XVIZMetaDataBuilder::SetVersion(std::string version) {
  version_ = version;
  return *this;
}

XVIZMetaDataBuilder& XVIZMetaDataBuilder::SetTime(double start_time, double end_time) {
  start_time_ = start_time;
  end_time_ = end_time;
  return *this;
}

XVIZMetaDataBuilder& XVIZMetaDataBuilder::SetMap(std::string map) {
  map_ = std::move(map);
  return *this;
}