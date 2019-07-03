
#include "connector/xviz/utils/metadata_stream.h"

using namespace mellocolate::metadata;

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

StreamStyle& StreamStyle::AddHeight(double height) {
  height_ = height;
  return *this;
}


Stream::Stream(std::string stream_name) :
  stream_name_(std::move(stream_name)) {}

Stream& Stream::AddCategory(std::string category) {
  categroy_ = std::move(category);
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