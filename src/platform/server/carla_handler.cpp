/*
 * File: carla_handler.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:39 pm
 */

#include "platform/server/carla_handler.h"

using namespace mellocolate;
using namespace xviz;


CarlaHandler::CarlaHandler(const std::shared_ptr<CarlaProxy>& carla_proxy_ptr,
  const std::shared_ptr<DrawingProxy>& drawing_proxy_ptr, uint64_t interval_ms) :
    carla_proxy_ptr_(carla_proxy_ptr), drawing_proxy_ptr_(drawing_proxy_ptr), interval_ms_(interval_ms) {}

std::shared_ptr<xviz::XVIZBaseSession> CarlaHandler::GetSession(const std::unordered_map<std::string, std::string>& params,
  std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr) {
  return std::make_shared<CarlaSession>(conn_ptr, carla_proxy_ptr_, drawing_proxy_ptr_, interval_ms_);
}