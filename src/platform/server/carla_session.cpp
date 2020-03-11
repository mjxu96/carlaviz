/*
 * File: carla_session.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Thursday, 20th February 2020 2:52:50 pm
 */

#include "platform/server/carla_session.h"

using namespace mellocolate;
using namespace xviz;

CarlaSession::CarlaSession(std::shared_ptr<websocketpp::connection<websocketpp::config::asio>> conn_ptr,
    std::weak_ptr<mellocolate::CarlaHandler> handler_weak_ptr, uint64_t interval_ms) : 
    XVIZBaseSession(conn_ptr), handler_weak_ptr_(handler_weak_ptr), interval_ms_(interval_ms) {
}

void CarlaSession::OnConnect() {
  auto err_code = conn_ptr_->send(handler_weak_ptr_.lock()->GetMetaData());
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
  while (true) {
    // auto start = std::chrono::high_resolution_clock::now();
    auto data = handler_weak_ptr_.lock()->GetUpdateData();
    // LOG_INFO("Send %ud data", data.length());
    auto err_code = conn_ptr_->send(std::move(data), websocketpp::frame::opcode::BINARY);
    if (err_code) {
      LOG_ERROR("Cannot send update data");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    // auto end = std::chrono::high_resolution_clock::now();
    // LOG_INFO("All spend %.3f", std::chrono::duration<double, std::milli>(end - start).count());
  }
}

void CarlaSession::OnDisconnect() {
  LOG_INFO("Frontend disconnected");
}