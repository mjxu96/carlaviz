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

#include <carlaviz/utils/definitions.h>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace carlaviz::sensor {

// sensor object definition
using SensorIDType = std::uint64_t;

enum class SensorType { CAMERA = 0u, LIDAR = 1u, UNKNOWN };

std::string SensorTypeToString(SensorType type);

enum class ImageColorType { GREY = 0, RGB = 1, RGBA = 2, BGRA = 3, UNKNOWN };

std::string ImageColorTypeToString(ImageColorType type);

struct SensorInfo {
  SensorIDType id = 0;
  SensorType type = SensorType::UNKNOWN;
  std::string name = "unknown";
};

struct Image {
  SensorInfo info = {.type = SensorType::CAMERA};
  ImageColorType image_color_type = ImageColorType::UNKNOWN;
  std::vector<uint8_t> data;
  std::optional<std::size_t> width = std::nullopt;
  std::optional<std::size_t> height = std::nullopt;
};

struct LidarMeasurement {
  SensorInfo info = {.type = SensorType::LIDAR};
  // flattened
  std::vector<float> points;
  std::vector<uint8_t> colors;
};

using SensorData = std::variant<Image, LidarMeasurement>;

// sensor utility functions
// convert the raw data to the image that translation can accept
// len = width * height * #channel
template <ImageColorType InType, ImageColorType OutType>
bool ConvertRawDataToImage(Image& out, const uint8_t* in, std::size_t width,
                           std::size_t height);

template <ImageColorType InType, ImageColorType OutType>
bool ConvertRawDataToImage(Image& out, const std::vector<uint8_t>& in,
                           std::size_t width, std::size_t height) {
  return ConvertRawDataToImage<InType, OutType>(out, in.data(), width, height);
}

#ifdef CARLAVIZ_SIMULATOR_CARLA
// util struct for carla simulator
struct LidarPointsWithTimestamp {
  double timestamp = 0.0;
  std::vector<float> points;
  std::vector<uint8_t> colors;
};
#endif
};  // namespace carlaviz::sensor
