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

#include <carlaviz/utils/definitions.h>
#include <carlaviz/utils/logging.h>
#include <carlaviz/utils/sensor.h>

#ifdef CARLAVIZ_FRONTEND_XVIZ
#include <lodepng.h>
#endif
#include <stdexcept>

namespace carlaviz::sensor {

std::string SensorTypeToString(SensorType type) {
  switch (type) {
    case SensorType::CAMERA:
      return "CAMERA";
    case SensorType::LIDAR:
      return "LIDAR";
    default:
      return "UNKNOWN";
  }
}

std::string ImageColorTypeToString(ImageColorType type) {
  switch (type) {
    case ImageColorType::RGB:
      return "RGB";
    case ImageColorType::RGBA:
      return "RGBA";
    case ImageColorType::BGRA:
      return "BGRA";
    case ImageColorType::GREY:
      return "GREY";
    default:
      return "UNKNOWN";
  }
}

#ifdef CARLAVIZ_FRONTEND_XVIZ

::LodePNGColorType FromInternalPNGColorTypeToLodePNGType(ImageColorType type) {
  switch (type) {
    case ImageColorType::GREY:
      return LodePNGColorType::LCT_GREY;
    case ImageColorType::RGB:
      return LodePNGColorType::LCT_RGB;
    case ImageColorType::RGBA:
      return LodePNGColorType::LCT_RGBA;
    default:
      break;
  }
  throw std::runtime_error(
      std::format("Unknown PNG Color Type {}", ImageColorTypeToString(type)));
}

struct PNGState {
  PNGState(ImageColorType ctype, std::uint32_t bitdepth) {
    ::lodepng_state_init(&state);
    state.info_raw.colortype = FromInternalPNGColorTypeToLodePNGType(ctype);
    state.info_raw.bitdepth = bitdepth;
    state.info_png.color.colortype =
        FromInternalPNGColorTypeToLodePNGType(ctype);
    state.info_png.color.bitdepth = bitdepth;

    // TODO tune below params
    auto settings = &(state.encoder.zlibsettings);
    settings->btype = 2;
    settings->use_lz77 = 0;
    settings->windowsize = 32;
    settings->minmatch = 3;
    settings->nicematch = 16;
    settings->lazymatching = 0;

    settings->custom_zlib = 0;
    settings->custom_deflate = 0;
    settings->custom_context = 0;

    logging::LogInfo(
        "Initialize the PNG state for color type {} and bitdepth {}",
        ImageColorTypeToString(ctype), bitdepth);
  }
  lodepng::State state;
};

template <ImageColorType CType, std::uint32_t BitDepth = 8u>
lodepng::State& GetPNGState() {
  static PNGState state(CType, BitDepth);
  return state.state;
}

template <ImageColorType OutType>
bool InternalConvertToImage(Image& out, const uint8_t* in, std::size_t width,
                            std::size_t height) {
  auto error =
      lodepng::encode(out.data, in, width, height, GetPNGState<OutType>());
  if (error) {
    logging::LogError(
        "Convertion from raw data to PNG data encounters an error {}",
        ::lodepng_error_text(error));
    return false;
  }
  out.image_color_type = OutType;
  return true;
}

template <ImageColorType OutType>
bool InternalConvertToImage(Image& out, const std::vector<uint8_t>& in,
                            std::size_t width, std::size_t height) {
  return InternalConvertToImage<OutType>(out, in.data(), width, height);
}

template <>
bool ConvertRawDataToImage<ImageColorType::BGRA, ImageColorType::RGB>(
    Image& out, const uint8_t* in, std::size_t width, std::size_t height) {
  std::vector<uint8_t> in_data;
  assert(in);
  assert(width * height != 0);
  in_data.resize(width * height * 3);
  for (auto i = 0; i < width * height; i++) {
    in_data[i * 3] = in[i * 4 + 2u];
    in_data[i * 3 + 1u] = in[i * 4 + 1u];
    in_data[i * 3 + 2u] = in[i * 4];
  }
  return InternalConvertToImage<ImageColorType::RGB>(out, in_data, width,
                                                     height);
}

template <>
bool ConvertRawDataToImage<ImageColorType::RGB, ImageColorType::RGB>(
    Image& out, const uint8_t* in, std::size_t width, std::size_t height) {
  assert(in);
  assert(width * height != 0);
  return InternalConvertToImage<ImageColorType::RGB>(out, in, width, height);
}

template <>
bool ConvertRawDataToImage<ImageColorType::GREY, ImageColorType::GREY>(
    Image& out, const uint8_t* in, std::size_t width, std::size_t height) {
  assert(in);
  assert(width * height != 0);
  return InternalConvertToImage<ImageColorType::GREY>(out, in, width, height);
}

#endif

}  // namespace carlaviz::sensor
