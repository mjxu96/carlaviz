#ifndef ROTHEBERG_XODR_GEOJSON_CONVERTER_H_
#define ROTHEBERG_XODR_GEOJSON_CONVERTER_H_

#include "carla/opendrive/OpenDriveParser.h"
#include "carla/road/element/RoadInfoGeometry.h"
#include <iostream>
#include <fstream>

namespace rothberg {
namespace utils {

class XodrGeojsonConverter {
public:
  static std::string Convert(std::string xodr);

private:


};

} // namespace utils
} // namespace rothberg


#endif