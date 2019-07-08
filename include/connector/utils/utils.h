/*
 * File: utils.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 7th July 2019 3:18:48 pm
 */

#ifndef MELLOCOLATE_UTILS_H_
#define MELLOCOLATE_UTILS_H_

#include "connector/utils/def.h"

#include "carla/geom/Location.h"
#include "carla/geom/Transform.h"
#include "carla/sensor/SensorData.h"
#include "carla/client/Sensor.h"

#include <tuple>

namespace mellocolate {
namespace utils {

/*
class SensorDataPackage {
public:
  SensorDataPackage() = delete;
  SensorDataPackage(carla::sensor::SensorData sensor_data, carla::geom::Location location,
    carla::geom::Transform transform) :
    sensor_data_(std::move(sensor_data)),
    location_(std::move(location)),
    transform_(std::move(transform)) {}

private:
  carla::sensor::SensorData sensor_data_;
  carla::geom::Location location_;
  carla::geom::Transform transform_;
};
*/

// rotate with yaw in radian
point_3d_t GetOffsetAfterTransform(const point_3d_t& origin, double yaw) {
  double x = origin.get<0>();
  double y = origin.get<1>();
  return point_3d_t(std::cos(yaw)*x - std::sin(yaw)*y,
                    std::sin(yaw)*x + std::cos(yaw)*y,
                    origin.get<2>());
}

}  // namespace utils
}  // namespace mellocolate

#endif