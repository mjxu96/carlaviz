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
  auto err_code = conn_ptr_->send(handler_weak_ptr_.lock()->GetMetadataWithMap());
  if (err_code) {
    LOG_INFO("Cannot send metadata");
    is_error_ = true;
    return;
  }
  LOG_INFO("Frontend connected");
  std::lock_guard lock_g(send_lock_);
  is_metadata_send_ = true;
}

void CarlaSession::Main() {
  if (is_error_) {
    return;
  }
  while (true) {
    send_lock_.lock();
    if (!is_metadata_send_) {
      is_metadata_send_ = true;
      auto err_c = conn_ptr_->send(handler_weak_ptr_.lock()->GetMetadata());
      if (err_c) {
        LOG_INFO("Cannot send metadata without map");
        send_lock_.unlock();
        is_error_ = true;
        return;
      }
    }
    send_lock_.unlock();

    auto err_code = conn_ptr_->send(handler_weak_ptr_.lock()->GetUpdateData(), websocketpp::frame::opcode::BINARY);
    if (err_code) {
      LOG_ERROR("Cannot send update data");
      is_error_ = true;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
  }
}

void CarlaSession::OnDisconnect() {
  is_error_ = true;
  LOG_INFO("Frontend disconnected");
}

bool CarlaSession::SetSendStatus(bool new_status) {
  std::lock_guard lock_g(send_lock_);
  is_metadata_send_ = new_status;
  return is_error_;
}