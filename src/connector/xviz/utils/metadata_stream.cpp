
#include "connector/xviz/utils/metadata_stream.h"

using namespace mellocolate::metadata;

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