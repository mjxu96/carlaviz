/*
 * File: ui_config.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Wednesday, 2nd October 2019 1:10:36 pm
 */

#include <string>
#include "platform/utils/json.hpp"

namespace mellocolate {

namespace metadata {

class UIConfig {
public:
  UIConfig() = default;
  void AddCamera(std::string camera_stream_name);
  void AddMetric(std::string metric_stream_name);
  nlohmann::json GetMetaData() const;


private:
  std::vector<std::string> camera_stream_names_;
  std::vector<std::string> metric_stream_names_;

};

}

}