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

#include <carlaviz/utils/map.h>

// for PI
#include <numbers>

namespace carlaviz::map {

const std::vector<std::vector<Point>> StopSign::boarder = {
    {{StopSign::stop_sign_length, 0.02, StopSign::stop_sign_height},
     {StopSign::stop_sign_length, 0,
      StopSign::stop_sign_height + StopSign::stop_sign_length},
     {-StopSign::stop_sign_length, 0,
      StopSign::stop_sign_height + StopSign::stop_sign_length},
     {-StopSign::stop_sign_length, 0.02, StopSign::stop_sign_height},
     {StopSign::stop_sign_length, 0.02, StopSign::stop_sign_height}},
    {{0, 0, 0}, {0, 0.02, StopSign::stop_sign_height}}};

const std::vector<std::vector<Point>> StopSign::word = {
    // S
    {{-1.2 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {-2.2 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {-2.2 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 1.2 * StopSign::stop_sign_stroke_length},
     {-1.2 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 1.2 * StopSign::stop_sign_stroke_length},
     {-1.2 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 0.2 * StopSign::stop_sign_stroke_length},
     {-2.2 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 0.2 * StopSign::stop_sign_stroke_length}},
    // T-1
    {{-1.1 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {-0.1 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length}},
    // T-2
    {{-0.6 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {-0.6 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 0.2 * StopSign::stop_sign_stroke_length}},
    // O
    {{0.1 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {1.1 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {1.1 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 0.2 * StopSign::stop_sign_stroke_length},
     {0.1 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 0.2 * StopSign::stop_sign_stroke_length},
     {0.1 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length}},
    // P
    {{1.3 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 0.2 * StopSign::stop_sign_stroke_length},
     {1.3 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {2.3 * StopSign::stop_sign_stroke_length, 0,
      StopSign::stop_sign_height + 2.2 * StopSign::stop_sign_stroke_length},
     {2.3 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 1.2 * StopSign::stop_sign_stroke_length},
     {1.3 * StopSign::stop_sign_stroke_length, 0.02,
      StopSign::stop_sign_height + 1.2 * StopSign::stop_sign_stroke_length}}};

const utils::Transform StopSign::on_road_transform = {
    {0, 0, 0}, {0, 0, -std::numbers::pi / 2.0}};

std::string TrafficLightStatusToString(TrafficLightStatus status) {
  switch (status) {
    case TrafficLightStatus::RED:
      return "red";
    case TrafficLightStatus::YELLOW:
      return "yellow";
    case TrafficLightStatus::GREEN:
      return "green";
    default:
      return "unknown";
  }
}

std::vector<std::array<float, 3>> PointsVectorToArrayVector(
    const std::vector<Point>& input) {
  std::vector<std::array<float, 3>> out;
  for (const auto& point : input) {
    out.push_back(point.ToArray());
  }
  return out;
}
std::vector<Point> ArrayVectorToPointsVector(
    const std::vector<std::array<float, 3>>& input) {
  std::vector<Point> out;
  for (const auto& p : input) {
    out.emplace_back(p);
  }
  return out;
}

}  // namespace carlaviz::map
