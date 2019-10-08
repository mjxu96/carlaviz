/*
 * File: ui_config.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Wednesday, 2nd October 2019 1:15:42 pm
 */

#include "platform/xviz/utils/ui_config.h"

using namespace mellocolate;
using namespace metadata;
using Json = nlohmann::json;

Json UIConfig::GetMetaData() const {
  Json json;
  if (!camera_stream_names_.empty()) {
    json["Camera"]["type"] = "panel";
    json["Camera"]["name"] = "Camera";
    json["Camera"]["children"][0]["type"] = "video";
    size_t id = 0u;
    for (const auto& camera_stream_name : camera_stream_names_) {
      json["Camera"]["children"][0]["cameras"][id] = camera_stream_name;
    }
  }
  return json;
}

void UIConfig::AddCamera(std::string camera_stream_name) {
  camera_stream_names_.emplace_back(std::move(camera_stream_name));
}