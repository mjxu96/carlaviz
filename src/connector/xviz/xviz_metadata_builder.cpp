
#include "connector/xviz/xviz_metadata_builder.h"


using namespace mellocolate;
using Json = nlohmann::json;

XVIZMetaDataBuilder& XVIZMetaDataBuilder::AddStream(metadata::Stream stream) {
  streams_.push_back(std::move(stream));
  return *this;
}

std::string XVIZMetaDataBuilder::GetMetaData() {
  Json json;
  json["type"] = "xviz/metadata";
  json["data"]["version"] = version_;
  if (map_ != std::nullopt) {
    json["data"]["map"] = map_.value();
  }
  for (const auto& stream : streams_) {
    if (stream.categroy_ != std::nullopt) {
      std::string category = stream.categroy_.value();
      json["data"]["streams"][stream.stream_name_]["category"] = category;
      if (category == "primitive" || category == "PRIMITIVE") {
        if (stream.type_ != std::nullopt) {
          json["data"]["streams"][stream.stream_name_]["primitive_type"] = stream.type_.value();
        }
      } else if (category == "variable" || category == "VARIABLE" ||
                 category == "time_series" || category == "TIME_SERIES") {
        if (stream.type_ != std::nullopt) {
          json["data"]["streams"][stream.stream_name_]["scalar_type"] = stream.type_.value();
        }
      }
    }
    if (stream.coordinate_ != std::nullopt) {
      json["data"]["streams"][stream.stream_name_]["coordinate"] = stream.coordinate_.value();
    }
    if (stream.stream_style_ != std::nullopt) {
      auto stream_style = stream.stream_style_.value();
      if (stream_style.fill_color != std::nullopt) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["fill_color"] = stream_style.fill_color.value();
      }
      if (stream_style.height != std::nullopt) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["height"] = stream_style.height.value();
      }
      if (stream_style.stroked != std::nullopt) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["stroked"] = (stream_style.stroked.value() ? "true" : "false");
      }
      if (stream_style.extruded != std::nullopt) {
        json["data"]["streams"][stream.stream_name_]["stream_style"]["extruded"] = (stream_style.extruded.value() ? "true" : "false");
      }
    }
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