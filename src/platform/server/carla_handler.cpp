/*
 * File: carla_handler.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:39 pm
 */

#include "platform/server/carla_session.h"
#include "platform/server/carla_handler.h"

using namespace mellocolate;
using namespace xviz;


CarlaHandler::CarlaHandler(const std::shared_ptr<CarlaProxy>& carla_proxy_ptr,
  const std::shared_ptr<DrawingProxy>& drawing_proxy_ptr, uint64_t interval_ms) :
    carla_proxy_ptr_(carla_proxy_ptr), drawing_proxy_ptr_(drawing_proxy_ptr), interval_ms_(interval_ms) {
  metadata_str_ = carla_proxy_ptr_->GetMetaData();
  std::thread t(std::bind(
    &CarlaHandler::StartReadData, this
  ));
  t.detach();
}

std::shared_ptr<xviz::XVIZBaseSession> CarlaHandler::GetSession(const std::unordered_map<std::string, std::string>& params,
  std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr) {
  return std::make_shared<CarlaSession>(conn_ptr, weak_from_this(), interval_ms_);
}

void CarlaHandler::StartReadData() {
  while (true) {
    // auto start = std::chrono::high_resolution_clock::now();
    auto xviz_builder = carla_proxy_ptr_->GetUpdateData();
    drawing_proxy_ptr_->AddDrawings(xviz_builder);
    std::string output;
    xviz::XVIZGLBWriter writer;
    writer.WriteMessage(output, xviz_builder.GetMessage());

    auto data_end = std::chrono::high_resolution_clock::now();

    std::lock_guard lock_g(update_lock_);
    update_str_ = std::move(output);
    // auto end = std::chrono::high_resolution_clock::now();
    // LOG_INFO("Data: %.3f   lock: %.3f", std::chrono::duration<double, std::milli>(data_end - start).count(),
      // std::chrono::duration<double, std::milli>(end - data_end).count());
  }
}

std::string CarlaHandler::GetMetaData() {
  std::lock_guard lock_g(metadata_lock_);
  return metadata_str_;
}

std::string CarlaHandler::GetUpdateData() {
  std::lock_guard lock_g(update_lock_);
  return update_str_;
}
