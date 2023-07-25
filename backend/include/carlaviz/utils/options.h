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

#include <cstdint>
#include <limits>
#include <string>

namespace carlaviz::utils {

struct SimulatorOption {
  std::string host = "127.0.0.1";
  uint16_t port = 2000u;
  std::size_t timeout_seconds = 10;
  std::size_t retry_times_after_disconnection =
      std::numeric_limits<std::size_t>::max();
  std::size_t retry_interval_seconds = 1;
  std::size_t sleep_between_updates_milliseconds = 0;
  std::string ego_vehicle_name = "ego";
  uint32_t sensor_max_lag_frame = 10;
};

struct TranslationOption {
  bool allow_static_objects = true;
};

struct ConnectorOption {
  std::string host = "0.0.0.0";
  uint16_t port = 8081u;
  std::size_t update_interval_milliseconds = 500;
};

}  // namespace carlaviz::utils
