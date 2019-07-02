#ifndef MELLOCOLATE_XVIZ_UTILS_METADATA_STREAM_H_
#define MELLOCOLATE_XVIZ_UTILS_METADATA_STREAM_H_

#include <string>
#include <optional>

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

struct StreamStyle {
  std::optional<bool> stroked = std::nullopt;
  std::optional<bool> extruded = std::nullopt;
  std::optional<std::string> fill_color = std::nullopt;
  std::optional<double> height = std::nullopt;
};


class Stream {
public:
  Stream(std::string stream_name);
  Stream& AddCategory(std::string category);
  Stream& AddCoordinate(std::string coordinate);
  Stream& AddType(std::string type);
  Stream& AddStreamStyle(StreamStyle stream_style);

  std::string stream_name_{""};
  std::optional<std::string> categroy_ = std::nullopt;
  std::optional<std::string> coordinate_ = std::nullopt;
  std::optional<std::string> type_ = std::nullopt;
  std::optional<StreamStyle> stream_style_ = std::nullopt;
  //metadata::stream::Category categroy_ = metadata::stream::Category::NONE;
};
  
} // namespace metadata
} // namespace mellocolate


#endif