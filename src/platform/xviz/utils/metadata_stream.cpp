/*
 * File: metadata_stream.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:52:29 pm
 */

#include "platform/xviz/utils/metadata_stream.h"

using namespace mellocolate::metadata;
using Json = nlohmann::json;

StreamStyle& StreamStyle::AddExtruded(bool is_extruded) {
  extruded_ = is_extruded;
  return *this;
}

StreamStyle& StreamStyle::AddStroked(bool is_stroked) {
  stroked_ = is_stroked;
  return *this;
}

StreamStyle& StreamStyle::AddFillColor(std::string fill_color) {
  fill_color_ = std::move(fill_color);
  return *this;
}

StreamStyle& StreamStyle::AddStrokeColor(std::string stroke_color) {
  stroke_color_ = std::move(stroke_color);
  return *this;
}

StreamStyle& StreamStyle::AddStrokeWidth(double width) {
  stroke_width_ = width;
  return *this;
}

StreamStyle& StreamStyle::AddHeight(double height) {
  height_ = height;
  return *this;
}

// Stream style for point cloud
StreamStyle& StreamStyle::AddPointCloudMode(std::string point_cloud_mode) {
  point_cloud_mode_ = std::move(point_cloud_mode);
  return *this;
}

StreamStyle& StreamStyle::AddRadiusPixels(double radius_pixels) {
  radius_pixels_ = radius_pixels;
  return *this;
}

Json StreamStyle::GetMetaData() const {
  Json json;
  if (fill_color_ != boost::none) {
    json["fill_color"] = fill_color_.value();
  }
  if (stroke_color_ != boost::none) {
    json["stroke_color"] = stroke_color_.value();
  }
  if (stroke_width_ != boost::none) {
    json["stroke_width"] = stroke_width_.value();
  }
  if (height_ != boost::none) {
    json["height"] = height_.value();
  }
  if (stroked_ != boost::none) {
    json["stroked"] = (stroked_.value() ? "true" : "false");
  }
  if (extruded_ != boost::none) {
    json["extruded"] = (extruded_.value() ? "true" : "false");
  }
  if (point_cloud_mode_ != boost::none) {
    json["point_cloud_mode"] = point_cloud_mode_.value();
  }
  if (radius_pixels_ != boost::none) {
    json["radius_pixels"] = radius_pixels_.value();
  }
  return json;
}

Stream::Stream(std::string stream_name) :
  stream_name_(std::move(stream_name)) {}

Stream& Stream::AddCategory(std::string category) {
  category_ = std::move(category);
  return *this;
}

Stream& Stream::AddCoordinate(std::string coordinate) {
  coordinate_ = std::move(coordinate);
  return *this;
}

Stream& Stream::AddType(std::string type) {
  type_ = std::move(type);
  return *this;
}


Stream& Stream::AddStreamStyle(StreamStyle stream_style) {
  stream_style_ = std::move(stream_style);
  return *this;
}

Stream& Stream::AddUnits(std::string units) {
  units_ = std::move(units);
  return *this;
}

Stream& Stream::AddScalarType(std::string scalar_type) {
  scalar_type_ = std::move(scalar_type);
  return *this;
}

std::string Stream::GetName() const {
  return stream_name_;
}

Json Stream::GetMetaData() const {
  Json json;
  if (category_ == boost::none) {
    return json.dump();
  }
  std::string category = category_.value();
  json["category"] = category;
  if (category == "primitive" || category == "PRIMITIVE") {
    if (type_ != boost::none) {
      json["primitive_type"] = type_.value();
    }
  } else if (category == "variable" || category == "VARIABLE" ||
             category == "time_series" || category == "TIME_SERIES") {
    if (type_ != boost::none) {
      json["scalar_type"] = type_.value();
    }
  }
  if (coordinate_ != boost::none) {
    json["coordinate"] = coordinate_.value();
  }
  if (stream_style_ != boost::none) {
    json["stream_style"] = stream_style_.value().GetMetaData();
  }
  if (units_ != boost::none) {
    json["units"] = units_.value();
  }
  if (scalar_type_ != boost::none) {
    json["scalar_type"] = scalar_type_.value();
  }
  return json;
}