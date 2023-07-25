/*
 * Project: carlaviz
 * Description: Carla Visulization in Browser
 * Author: Minjun Xu (mjxu96@outlook.com)
 * -----
 * MIT License
 * Copyright (c) 2023 Minjun Xu
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#pragma once

#include <array>
#include <cmath>

#ifdef CARLAVIZ_SIMULATOR_CARLA
#include <carla/geom/Location.h>
#endif

namespace carlaviz::utils {

using ScaleType = std::array<double, 3>;

struct Location {
  Location() = default;
  Location(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
  Location(const std::array<double, 3> &loc)
      : x(loc[0]), y(loc[1]), z(loc[2]) {}
  Location(const std::array<float, 3> &loc) : x(loc[0]), y(loc[1]), z(loc[2]) {}
#ifdef CARLAVIZ_SIMULATOR_CARLA
  Location(const carla::geom::Location &loc) : x(loc.x), y(loc.y), z(loc.z) {}
#endif

  std::array<float, 3> ToArray() const {
    return {(float)x, (float)y, (float)z};
  }

  void ApplyScale(const ScaleType &scale) {
    x = scale[0] * x;
    y = scale[1] * y;
    z = scale[2] * z;
  }

  Location &operator+=(const Location &rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  friend Location operator+(Location lhs, const Location &rhs) {
    lhs += rhs;
    return lhs;
  }

  Location &operator-=(const Location &rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  friend Location operator-(Location lhs, const Location &rhs) {
    lhs -= rhs;
    return lhs;
  }

  Location &operator*=(double rhs) {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }

  friend Location operator*(Location lhs, double rhs) {
    lhs *= rhs;
    return lhs;
  }

  friend Location operator*(double lhs, Location rhs) {
    rhs *= lhs;
    return rhs;
  }

  Location &operator/=(double rhs) {
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
  }

  friend Location operator/(Location lhs, double rhs) {
    lhs /= rhs;
    return lhs;
  }

  friend Location operator/(double lhs, Location rhs) {
    rhs /= lhs;
    return rhs;
  }

  bool operator==(const Location &rhs) const {
    return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
  }

  bool operator!=(const Location &rhs) const { return !(*this == rhs); }

  double x = 0;
  double y = 0;
  double z = 0;
};

struct Rotation {
  Rotation() = default;
  Rotation(double p, double y, double r) : pitch(p), yaw(y), roll(r) {}

  void Rotate(Location &in_point) const {
    // Rotates Rz(yaw) * Ry(pitch) * Rx(roll) = first x, then y, then z.
    const float cy = std::cos(yaw);
    const float sy = std::sin(yaw);
    const float cr = std::cos(roll);
    const float sr = std::sin(roll);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);

    Location out_point;
    out_point.x = in_point.x * (cp * cy) +
                  in_point.y * (cy * sp * sr - sy * cr) +
                  in_point.z * (-cy * sp * cr - sy * sr);

    out_point.y = in_point.x * (cp * sy) +
                  in_point.y * (sy * sp * sr + cy * cr) +
                  in_point.z * (-sy * sp * cr + cy * sr);

    out_point.z =
        in_point.x * (sp) + in_point.y * (-cp * sr) + in_point.z * (cp * cr);

    in_point = out_point;
  }

  // in radius
  double pitch = 0.0;
  double yaw = 0.0;
  double roll = 0.0;
};

class Transform {
 public:
  Location location;
  Rotation rotation;

  Transform() = default;
  Transform(const Location &loc, const Rotation &rot)
      : location(loc), rotation(rot) {}

  void TransformLocation(Location &in_direction) const {
    auto out_location = in_direction;
    rotation.Rotate(out_location);
    out_location += location;
    in_direction = out_location;
  }
};

}  // namespace carlaviz::utils
