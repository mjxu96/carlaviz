/*
 * File: carla_session.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:50 pm
 */

#include "platform/server/carla_session.h"

using namespace mellocolate;
using namespace xviz;

CarlaSession::CarlaSession(std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr,
    std::shared_ptr<CarlaProxy> carla_proxy_ptr, std::shared_ptr<DrawingProxy> drawing_proxy_ptr, uint64_t interval_ms) : 
    XVIZBaseSession(conn_ptr), carla_proxy_ptr_(carla_proxy_ptr), drawing_proxy_ptr_(drawing_proxy_ptr), interval_ms_(interval_ms) {
}

void CarlaSession::OnConnect() {
  if (carla_proxy_ptr_ == nullptr) {
    LOG_ERROR("Carla proxy is nullptr");
    is_error_ = true;
    return;
  }
  std::string metadata = carla_proxy_ptr_->GetMetaData();
  auto err_code = conn_ptr_->send(std::move(metadata));
  if (err_code) {
    LOG_INFO("Cannot send metadata");
    is_error_ = true;
    return;
  }
  LOG_INFO("Frontend connected");
}

void CarlaSession::Main() {
  if (is_error_) {
    return;
  }
  if (drawing_proxy_ptr_ == nullptr) {
    LOG_ERROR("Drawing proxy is nullptr");
    return;
  }
  while (true) {
    // LOG_ERROR("Drawing once");
    auto builder = carla_proxy_ptr_->GetUpdateData();
    drawing_proxy_ptr_->AddDrawings(builder);
    std::string output;
    XVIZGLBWriter writer;
    writer.WriteMessage(output, builder.GetMessage());
    auto err_code = conn_ptr_->send(std::move(output), websocketpp::frame::opcode::BINARY);
    if (err_code) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
  }
}

void CarlaSession::OnDisconnect() {
  LOG_INFO("Frontend disconnected");
}