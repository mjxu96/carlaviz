/*
 * File: metadata_stream.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 8:53:32 pm
 */

#ifndef MELLOCOLATE_XVIZ_UTILS_METADATA_STREAM_H_
#define MELLOCOLATE_XVIZ_UTILS_METADATA_STREAM_H_

#include <string>
#include <boost/optional.hpp>

#include "platform/utils/json.hpp"

namespace mellocolate {

namespace metadata {

namespace stream {

enum class Coordinate {
  GEOGRAPHIC,
  VEHICLE_RELATIVE,
  IDENTITY,
  DYNAMIC,
  NONE,
};

enum class Category {
  POSE,
  PRIMITIVES,
  TIME_SERIES,
  NONE,
};

enum class PrimitiveType {
  POINT,
  POLYGON,
  POLYLINE,
  CIRCLE,
  STADIUM,
  TEXT,
  IMAGE,
  NONE,
};


} // namespace stream

/*
struct StreamStyle {
  boost::optional<bool> stroked = boost::none;
  boost::optional<bool> extruded = boost::none;
  boost::optional<std::string> fill_color = boost::none;
  boost::optional<double> height = boost::none;
};
*/

class StreamStyle {
public:
  StreamStyle() = default;
  StreamStyle& AddExtruded(bool is_extruded);
  StreamStyle& AddStroked(bool is_stroked);
  StreamStyle& AddFillColor(std::string fill_color);
  StreamStyle& AddStrokeColor(std::string stroke_color);
  StreamStyle& AddStrokeWidth(double width);
  StreamStyle& AddHeight(double height);

  nlohmann::json GetMetaData() const;

  boost::optional<bool> stroked_ = boost::none;
  boost::optional<bool> extruded_ = boost::none;
  boost::optional<std::string> fill_color_ = boost::none;
  boost::optional<std::string> stroke_color_ = boost::none;
  boost::optional<double> stroke_width_ = boost::none;
  boost::optional<double> height_ = boost::none;
  
  // Stream style for point cloud
  StreamStyle& AddPointCloudMode(std::string point_cloud_mode); 
  StreamStyle& AddRadiusPixels(double radius_pixels); 
  boost::optional<std::string> point_cloud_mode_ = boost::none;
  boost::optional<double> radius_pixels_ = boost::none;
};


class Stream {
public:
  Stream(std::string stream_name);
  Stream& AddCategory(std::string category);
  Stream& AddCoordinate(std::string coordinate);
  Stream& AddType(std::string type);
  Stream& AddStreamStyle(StreamStyle stream_style);
  Stream& AddUnits(std::string units);
  Stream& AddScalarType(std::string scalar_type);

  std::string GetName() const;
  nlohmann::json GetMetaData() const;

  std::string stream_name_{""};
  boost::optional<std::string> category_ = boost::none;
  boost::optional<std::string> coordinate_ = boost::none;
  boost::optional<std::string> type_ = boost::none;
  boost::optional<StreamStyle> stream_style_ = boost::none;
  boost::optional<std::string> units_ = boost::none;
  boost::optional<std::string> scalar_type_ = boost::none;
  //metadata::stream::Category categroy_ = metadata::stream::Category::NONE;
};
  
} // namespace metadata
} // namespace mellocolate


#endif