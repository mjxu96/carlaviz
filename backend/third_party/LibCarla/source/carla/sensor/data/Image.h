// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/sensor/data/Color.h"
#include "carla/sensor/data/ImageTmpl.h"

namespace carla {
namespace sensor {
namespace data {

  /// An image of 32-bit BGRA colors (8-bit channels)
  using Image = ImageTmpl<Color>;

  /// An image of 64-bit BGRA colors (16-bit channels)
  using OpticalFlowImage = ImageTmpl<OpticalFlowPixel>;

} // namespace data
} // namespace sensor
} // namespace carla
